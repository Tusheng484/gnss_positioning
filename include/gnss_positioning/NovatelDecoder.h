/**
 * @file NovatelDecoder.h
 * @brief NovAtel OEM 接收机数据解码模块头文件
 * @author Jiaoyang Wu
 * @date 2026-03-20
 *
 * 本模块实现对 NovAtel OEM4/6/7 系列接收机 OEM 二进制协议的解析。
 * 主要功能包括：
 * 1. 基于状态机的流式字节同步。
 * 2. 消息 ID 分发与 CRC32 完整性校验。
 * 3. 观测数据 (RANGE) 与星历数据 (GPSEPHEM, BDSEPHEMERIS) 的二进制负载解析。
 */

#ifndef NOVATEL_DECODER_H
#define NOVATEL_DECODER_H

#include <cstdint>
#include <vector>
#include <string>

namespace gnss {

/**
 * @brief GNSS 系统类型枚举
 */
enum class GnssSystem {
    UNKNOWN = 0, ///< 未知系统
    GPS = 1,     ///< 全球定位系统
    BDS = 2,     ///< 北斗卫星导航系统
    GLO = 3,     ///< 格洛纳斯系统
    GAL = 4,     ///< 伽利略系统
    QZSS = 5     ///< 准天顶卫星系统
};

/**
 * @brief 原始观测值数据结构
 */
struct ObsData {
    GnssSystem system;      ///< 卫星所属系统
    uint32_t prn;           ///< 卫星 PRN 编号
    double pseudorange;     ///< 伪距，单位：米 (m)
    double carrierPhase;    ///< 载波相位，单位：周 (cycle)
    float doppler;          ///< 多普勒频移，单位：赫兹 (Hz)
    float snr;              ///< 载噪比，单位：dB-Hz
    int freqIdx;            ///< 频点索引（0: 第一频点 L1/B1I，1: 第二频点 L2/B3I）
    uint32_t status;        ///< 通道跟踪状态位
};

/**
 * @brief GPS 导航星历结构体 (GPSEPHEM ID: 7)
 * 对应 GPS L1 C/A 解调后的导航电文参数。
 */
struct GpsEphem {
    uint32_t prn;           ///< 卫星 PRN 编号
    double tow;             ///< 消息发布时间（周内秒），单位：秒 (s)
    uint32_t health;        ///< 卫星健康状态（0: 健康）
    uint32_t iode1;         ///< 星历数据发布序号 1（IODE）
    uint32_t iode2;         ///< 星历数据发布序号 2（IODE）
    uint32_t week;          ///< 连续 GPS 周
    uint32_t zweek;         ///< Z-Count 周
    double toe;             ///< 星历参考时刻（TOE），单位：秒 (s)
    double majaxis;         ///< 轨道长半轴 (A)，单位：米 (m)
    double dN;              ///< 平均运动变化率 (Mean Motion Difference)，单位：rad/s
    double ma;              ///< 参考时刻平均近点角 (M0)，单位：弧度 (rad)
    double ecc;             ///< 轨道离心率 (e)
    double argper;          ///< 近地点角，单位：弧度 (rad)
    double cuc, cus;        ///< 纬度幅度的余弦/正弦摄动项
    double crc, crs;        ///< 轨道半径的余弦/正弦摄动项
    double cic, cis;        ///< 轨道倾角的余弦/正弦摄动项
    double inc;             ///< 参考时刻轨道倾角 (i0)，单位：弧度 (rad)
    double inc_dot;         ///< 轨道倾角变化率 (IDOT)，单位：rad/s
    double omegazero;       ///< 参考时刻升交点赤经 (Omega0)，单位：弧度 (rad)
    double omegadot;        ///< 升交点赤经变化率 (Omega Dot)，单位：rad/s
    uint32_t iodc;          ///< 钟差数据发布序号（IODC）
    double toc;             ///< 钟差参考时刻（TOC），单位：秒 (s)
    double tgd;             ///< 群延迟（TGD），单位：秒 (s)
    double af0, af1, af2;   ///< 卫星钟偏差、漂移、漂移率系数
};

/**
 * @brief 北斗 (BDS) 导航星历结构体 (BDSEPHEMERIS ID: 1696)
 * 对应北斗 B1I 信号解调后的导航电文参数。
 */
struct BdsEphem {
    uint32_t prn;           ///< 卫星 PRN 编号
    uint32_t week;          ///< 北斗周 (BDT Week)
    double ura;             ///< 用户测距精度（URA），单位：米 (m)
    uint32_t health;        ///< 卫星健康状态 (0: 健康)
    double tgd1;            ///< B1 信号群延迟 (s)
    double tgd2;            ///< B2 信号群延迟 (s)
    uint32_t aodc;          ///< 钟差数据龄期（AODC）
    uint32_t toc;           ///< 钟差参考时刻 (s)
    double af0, af1, af2;   ///< 卫星钟差系数
    uint32_t aode;          ///< 星历数据龄期（AODE）
    uint32_t toe;           ///< 星历参考时刻 (s)
    double rootA;           ///< 轨道长半轴平方根 (sqrt(A))
    double ecc;             ///< 轨道离心率 (e)
    double omega;           ///< 近地点角 (rad)
    double dN;              ///< 平均运动变化率 (rad/s)
    double m0;              ///< 平均近点角 (rad)
    double omegazero;       ///< 参考时刻升交点赤经 (rad)
    double omegadot;        ///< 升交点赤经变化率 (rad/s)
    double i0;              ///< 参考时刻轨道倾角 (rad)
    double idot;            ///< 轨道倾角变化率 (rad/s)
    double cuc, cus, crc, crs, cic, cis; ///< 摄动项
};

/**
 * @class NovatelDecoder
 * @brief NovAtel OEM 协议解析类
 */
class NovatelDecoder {
public:
    /**
     * @brief 构造解码器并初始化状态机
     * @return 无
     */
    NovatelDecoder();
    ~NovatelDecoder() = default;

    /**
     * @brief 输入字节流进行解析
     * @param byte 输入的原始字节
     * @return true: 成功解码出一帧完整消息；false: 同步中或校验失败
     */
    bool inputData(uint8_t byte);

    /**
     * @brief 获取最新解析出的观测值列表
     * @return 观测值列表引用
     */
    const std::vector<ObsData>& getObsData() const { return obsList_; }
    /**
     * @brief 获取最新解析出的 GPS 星历
     * @return GPS 星历引用
     */
    const GpsEphem& getGpsEphem() const { return lastGpsEphem_; }
    /**
     * @brief 获取最新解析出的 BDS 星历
     * @return BDS 星历引用
     */
    const BdsEphem& getBdsEphem() const { return lastBdsEphem_; }
    /**
     * @brief 获取当前消息的 ID（例如 43、7、1696）
     * @return 消息 ID
     */
    uint16_t getMsgId() const { return msgId_; }
    /**
     * @brief 获取当前消息头中的 GPS 周
     * @return GPS 周
     */
    uint16_t getWeek() const { return week_; }
    /**
     * @brief 获取当前消息头中的周内秒（TOW）
     * @return 周内秒，单位：秒 (s)
     */
    double getTow() const { return tow_; }

private:
    /**
     * @brief 解码状态机阶段
     */
    enum class DecodeStatus {
        SYNC1,      ///< 等待同步头 0xAA
        SYNC2,      ///< 等待同步头 0x44
        SYNC3,      ///< 等待同步头 0x12
        HEADER,     ///< 读取变长消息头
        DATA,       ///< 读取消息负载
        CRC         ///< 读取 4 字节 CRC32 校验码
    };

    /**
     * @brief 根据当前消息 ID 分发并解析负载
     * @return true: 已解析并更新缓存；false: 当前消息类型不支持
     */
    bool decode();
    /**
     * @brief 解析 RANGE 消息并更新观测缓存
     * @return 无
     */
    void decodeRange();
    /**
     * @brief 解析 GPSEPHEM 消息并更新 GPS 星历缓存
     * @return 无
     */
    void decodeGpsEphem();
    /**
     * @brief 解析 BDSEPHEMERIS 消息并更新 BDS 星历缓存
     * @return 无
     */
    void decodeBdsEphem();

    /**
     * @brief 计算 CRC32 校验值（IEEE 802.3）
     * @param buffer 输入数据指针
     * @param len 数据长度，单位：字节
     * @return 计算得到的 CRC32
     */
    uint32_t calculateCrc32(const uint8_t* buffer, uint32_t len);
    
    // 字节流解析辅助函数（小端模式，直接按目标类型解释字节序列）。
    /**
     * @brief 从缓冲区读取 uint32_t（小端）
     * @param p 输入指针
     * @return 解析后的 uint32_t 值
     */
    uint32_t getUint32(const uint8_t* p) { return *(reinterpret_cast<const uint32_t*>(p)); }
    /**
     * @brief 从缓冲区读取 uint16_t（小端）
     * @param p 输入指针
     * @return 解析后的 uint16_t 值
     */
    uint16_t getUint16(const uint8_t* p) { return *(reinterpret_cast<const uint16_t*>(p)); }
    /**
     * @brief 从缓冲区读取 double（小端）
     * @param p 输入指针
     * @return 解析后的 double 值
     */
    double getDouble(const uint8_t* p) { return *(reinterpret_cast<const double*>(p)); }
    /**
     * @brief 从缓冲区读取 float（小端）
     * @param p 输入指针
     * @return 解析后的 float 值
     */
    float getFloat(const uint8_t* p) { return *(reinterpret_cast<const float*>(p)); }

    DecodeStatus status_;       ///< 当前解码器状态
    uint8_t buffer_[8192];      ///< 内部数据缓冲区
    uint32_t bufIdx_;           ///< 缓冲区当前填充位置
    uint32_t headerLen_;        ///< 消息头长度
    uint16_t dataLen_;          ///< 消息负载长度
    uint16_t msgId_;            ///< 消息 ID
    uint16_t week_;             ///< 消息头提取的 GPS 周
    double tow_;                ///< 消息头提取的周内秒 (s)

    std::vector<ObsData> obsList_; ///< 观测值缓存
    GpsEphem lastGpsEphem_;        ///< 最新 GPS 星历缓存
    BdsEphem lastBdsEphem_;        ///< 最新 BDS 星历缓存
};

} // namespace gnss

#endif // NOVATEL_DECODER_H
