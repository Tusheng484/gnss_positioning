/**
 * @file NovatelDecoder.cpp
 * @brief NovAtel OEM 协议解析模块实现
 * @author Unknown
 * @date Unknown
 *
 * 核心逻辑：
 * 1. 帧同步：搜索 0xAA 0x44 0x12 同步序列。
 * 2. 头解析：提取消息 ID、负载长度、GPS 周与周内秒（TOW）。
 * 3. 负载解析：针对 RANGE (ID 43), GPSEPHEM (ID 7), BDSEPHEMERIS (ID 1696) 进行二进制解析。
 * 4. 校验：采用 32 位 CRC 校验确保数据在传输过程中未损坏。
 */

#include "gnss_positioning/NovatelDecoder.h"
#include <cstring>
#include <iostream>

namespace gnss {

/**
 * @brief 预定义的 NovAtel 消息 ID
 *
 * 该处常量用于将协议层消息类型与解析函数解耦，便于扩展与维护。
 */
constexpr uint16_t ID_RANGE = 43;        ///< 原始观测值 (包含伪距、载波相位等)
constexpr uint16_t ID_GPSEPHEM = 7;      ///< GPS L1 C/A 星历
constexpr uint16_t ID_BDSEPHEMERIS = 1696; ///< BDS B1I 星历

NovatelDecoder::NovatelDecoder()
    : status_(DecodeStatus::SYNC1), bufIdx_(0), headerLen_(28), dataLen_(0), msgId_(0), week_(0), tow_(0.0) {
    obsList_.reserve(64);
}

/**
 * @brief 字节流同步与帧构建状态机
 * @param byte 输入字节
 * @return 是否成功完成一帧解码
 */
bool NovatelDecoder::inputData(uint8_t byte) {
    switch (status_) {
        case DecodeStatus::SYNC1:
            if (byte == 0xAA) { buffer_[0] = byte; bufIdx_ = 1; status_ = DecodeStatus::SYNC2; }
            break;
        case DecodeStatus::SYNC2:
            if (byte == 0x44) { buffer_[bufIdx_++] = byte; status_ = DecodeStatus::SYNC3; }
            else { status_ = DecodeStatus::SYNC1; bufIdx_ = 0; }
            break;
        case DecodeStatus::SYNC3:
            if (byte == 0x12) { buffer_[bufIdx_++] = byte; status_ = DecodeStatus::HEADER; }
            else { status_ = DecodeStatus::SYNC1; bufIdx_ = 0; }
            break;
        case DecodeStatus::HEADER:
            buffer_[bufIdx_++] = byte;
            // 消息头长度由头部第 4 字节给出，后续按该长度读取完整头部。
            if (bufIdx_ == 4) {
                headerLen_ = byte;
                if (headerLen_ < 28 || headerLen_ > 128) { // 协议约束：消息头至少 28 字节。
                    status_ = DecodeStatus::SYNC1;
                    bufIdx_ = 0;
                }
            } else if (bufIdx_ == headerLen_) {
                // 提取消息头关键字段：消息类型、负载长度与时间标签。
                msgId_ = getUint16(&buffer_[4]);
                dataLen_ = getUint16(&buffer_[8]);
                week_ = getUint16(&buffer_[14]);
                tow_ = getUint32(&buffer_[16]) / 1000.0; // 协议定义单位为毫秒。
                
                if (dataLen_ == 0) {
                    status_ = DecodeStatus::CRC; 
                } else if (dataLen_ > 8000) { // 负载上限保护：避免缓冲区溢出风险。
                    status_ = DecodeStatus::SYNC1;
                    bufIdx_ = 0;
                } else {
                    status_ = DecodeStatus::DATA;
                }
            }
            break;
        case DecodeStatus::DATA:
            buffer_[bufIdx_++] = byte;
            if (bufIdx_ == headerLen_ + dataLen_) {
                status_ = DecodeStatus::CRC;
            }
            break;
        case DecodeStatus::CRC:
            buffer_[bufIdx_++] = byte;
            if (bufIdx_ == headerLen_ + dataLen_ + 4) {
                // CRC 校验覆盖消息头与负载段。
                uint32_t calcCrc = calculateCrc32(buffer_, headerLen_ + dataLen_);
                uint32_t recvCrc = getUint32(&buffer_[headerLen_ + dataLen_]);
                
                status_ = DecodeStatus::SYNC1; 
                bufIdx_ = 0; 

                if (calcCrc == recvCrc) {
                    return decode(); 
                }
            }
            break;
    }
    return false;
}

/**
 * @brief 根据消息 ID 分发解码任务
 * @return true: 已解析并更新缓存；false: 当前消息类型不支持
 */
bool NovatelDecoder::decode() {
    switch (msgId_) {
        case ID_RANGE: decodeRange(); return true;
        case ID_GPSEPHEM: decodeGpsEphem(); return true;
        case ID_BDSEPHEMERIS: decodeBdsEphem(); return true;
        default: break;
    }
    return false;
}

/**
 * @brief 解析 RANGE 消息 (ID 43)
 * @return 无
 */
void NovatelDecoder::decodeRange() {
    obsList_.clear();
    const uint8_t* p = &buffer_[headerLen_];
    uint32_t nObs = getUint32(p);
    p += 4;

    for (uint32_t i = 0; i < nObs; ++i) {
        ObsData obs;
        obs.prn = getUint16(p);                    // 字节偏移 0
        obs.pseudorange = getDouble(p + 4);        // 字节偏移 4，单位：m
        obs.carrierPhase = getDouble(p + 16);      // 字节偏移 16，单位：cycle
        obs.doppler = getFloat(p + 28);            // 字节偏移 28，单位：Hz
        obs.snr = getFloat(p + 32);                // 字节偏移 32，单位：dB-Hz
        uint32_t channelStatus = getUint32(p + 40); // 字节偏移 40：通道跟踪状态

        // 通道状态位解析：
        // - system: bits 16-18
        // - signal type: bits 21-25
        uint32_t system = (channelStatus >> 16) & 0x07;
        uint32_t sigType = (channelStatus >> 21) & 0x1F;

        bool supported = false;
        if (system == 0) { // GPS
            obs.system = GnssSystem::GPS;
            if (sigType == 0) { obs.freqIdx = 0; supported = true; }      // L1 C/A
            else if (sigType == 9) { obs.freqIdx = 1; supported = true; } // L2 P(Y)
        } else if (system == 4) { // BDS
            obs.system = GnssSystem::BDS;
            if (sigType == 0 || sigType == 4) { obs.freqIdx = 0; supported = true; } // B1I
            else if (sigType == 2 || sigType == 6) { obs.freqIdx = 1; supported = true; } // B3I
        }

        if (supported) {
            obsList_.push_back(obs);
        }
        p += 44; 
    }
}

/**
 * @brief 解析 GPSEPHEM 消息 (ID 7)
 * 字节偏移量严格遵循 NovAtel OEM7 二进制协议定义
 * @return 无
 */
void NovatelDecoder::decodeGpsEphem() {
    const uint8_t* p = &buffer_[headerLen_];
    lastGpsEphem_.prn = getUint32(p);             // 卫星 PRN
    lastGpsEphem_.tow = getDouble(p + 4);         // 周内秒 (s)
    lastGpsEphem_.health = getUint32(p + 12);     // 健康状态
    lastGpsEphem_.iode1 = getUint32(p + 16);      // IODE 1
    lastGpsEphem_.iode2 = getUint32(p + 20);      // IODE 2
    lastGpsEphem_.week = getUint32(p + 24);       // GPS 周
    lastGpsEphem_.zweek = getUint32(p + 28);      // Z-Count 周
    lastGpsEphem_.toe = getDouble(p + 32);        // TOE (s)
    lastGpsEphem_.majaxis = getDouble(p + 40);    // 轨道长半轴 A (m)
    lastGpsEphem_.dN = getDouble(p + 48);         // 平均运动改正 Δn (rad/s)
    lastGpsEphem_.ma = getDouble(p + 56);         // M0 (rad)
    lastGpsEphem_.ecc = getDouble(p + 64);        // e
    lastGpsEphem_.argper = getDouble(p + 72);     // ω (rad)
    lastGpsEphem_.cuc = getDouble(p + 80);        // Cuc (rad)
    lastGpsEphem_.cus = getDouble(p + 88);        // Cus (rad)
    lastGpsEphem_.crc = getDouble(p + 96);        // Crc (m)
    lastGpsEphem_.crs = getDouble(p + 104);       // Crs (m)
    lastGpsEphem_.cic = getDouble(p + 112);       // Cic (rad)
    lastGpsEphem_.cis = getDouble(p + 120);       // Cis (rad)
    lastGpsEphem_.inc = getDouble(p + 128);       // i0 (rad)
    lastGpsEphem_.inc_dot = getDouble(p + 136);   // IDOT (rad/s)
    lastGpsEphem_.omegazero = getDouble(p + 144); // Ω0 (rad)
    lastGpsEphem_.omegadot = getDouble(p + 152);  // Ωdot (rad/s)
    lastGpsEphem_.iodc = getUint32(p + 160);      // IODC
    lastGpsEphem_.toc = getDouble(p + 164);       // TOC (s)
    lastGpsEphem_.tgd = getDouble(p + 172);       // TGD (s)
    lastGpsEphem_.af0 = getDouble(p + 180);       // af0 (s)
    lastGpsEphem_.af1 = getDouble(p + 188);       // af1 (s/s)
    lastGpsEphem_.af2 = getDouble(p + 196);       // af2 (s/s^2)
}

/**
 * @brief 解析 BDSEPHEMERIS 消息 (ID 1696)
 * 字节偏移量严格遵循 NovAtel OEM7 二进制协议定义
 * @return 无
 */
void NovatelDecoder::decodeBdsEphem() {
    const uint8_t* p = &buffer_[headerLen_];
    lastBdsEphem_.prn = getUint32(p);             // 卫星编号
    lastBdsEphem_.week = getUint32(p + 4);        // BDT 周
    lastBdsEphem_.ura = getDouble(p + 8);         // URA (m)
    lastBdsEphem_.health = getUint32(p + 16);     // 健康状态
    lastBdsEphem_.tgd1 = getDouble(p + 20);       // TGD1 (s)
    lastBdsEphem_.tgd2 = getDouble(p + 28);       // TGD2 (s)
    lastBdsEphem_.aodc = getUint32(p + 36);       // AODC
    lastBdsEphem_.toc = getUint32(p + 40);        // TOC (s)
    lastBdsEphem_.af0 = getDouble(p + 44);        // a0 (s)
    lastBdsEphem_.af1 = getDouble(p + 52);        // a1 (s/s)
    lastBdsEphem_.af2 = getDouble(p + 60);        // a2 (s/s^2)
    lastBdsEphem_.aode = getUint32(p + 68);       // AODE
    lastBdsEphem_.toe = getUint32(p + 72);        // TOE (s)
    lastBdsEphem_.rootA = getDouble(p + 76);      // sqrt(A) (sqrt(m))
    lastBdsEphem_.ecc = getDouble(p + 84);        // e
    lastBdsEphem_.omega = getDouble(p + 92);      // ω (rad)
    lastBdsEphem_.dN = getDouble(p + 100);        // 平均运动改正 Δn (rad/s)
    lastBdsEphem_.m0 = getDouble(p + 108);        // M0 (rad)
    lastBdsEphem_.omegazero = getDouble(p + 116); // Ω0 (rad)
    lastBdsEphem_.omegadot = getDouble(p + 124);  // Ωdot (rad/s)
    lastBdsEphem_.i0 = getDouble(p + 132);        // i0 (rad)
    lastBdsEphem_.idot = getDouble(p + 140);      // IDOT (rad/s)
    lastBdsEphem_.cuc = getDouble(p + 148);       // Cuc (rad)
    lastBdsEphem_.cus = getDouble(p + 156);       // Cus (rad)
    lastBdsEphem_.crc = getDouble(p + 164);       // Crc (m)
    lastBdsEphem_.crs = getDouble(p + 172);       // Crs (m)
    lastBdsEphem_.cic = getDouble(p + 180);       // Cic (rad)
    lastBdsEphem_.cis = getDouble(p + 188);       // Cis (rad)
}

/**
 * @brief NovAtel CRC32 算法实现
 * 采用标准 CRC-32 (IEEE 802.3) 生成多项式: 0xEDB88320
 * @param buffer 输入数据指针
 * @param len 数据长度，单位：字节
 * @return 计算得到的 CRC32
 */
uint32_t NovatelDecoder::calculateCrc32(const uint8_t* buffer, uint32_t len) {
    uint32_t crc = 0;
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= buffer[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else crc >>= 1;
        }
    }
    return crc;
}

} // namespace gnss
