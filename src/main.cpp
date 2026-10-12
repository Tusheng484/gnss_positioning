/**
 * @file main.cpp
 * @brief GNSS 数据流驱动的实时 SPP 单点定位程序（模块化架构）
 * @author Jiaoyang Wu
 * @date 2026-04-20
 *
 * 修改历史：
 * - 2026-03-30：建立模块化主流程（配置管理/终端输出/文件管理/核心解算），支持星历与观测落盘。
 * - 2026-04-05：扩展 SPP 输出字段（PDOP/测速/钟漂等）并同步落盘格式（由 SPP 模块提供数据）。
 *
 * 工程职责划分：
 * - 核心 SPP 算法模块：单点定位解算
 * - 终端输出模块：实时显示处理进度与关键结果
 * - 文件管理模块：数据持久化（结果、卫星状态、星历、观测）
 * - 配置管理模块：运行时开关与参数管理（支持配置文件热更新）
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <fstream>
#include <array>
#include <map>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <cctype>

#include <gnss_positioning/Config.h>
#include <gnss_positioning/Logging.h>
#include <gnss_positioning/NavMath.h>
#include <gnss_positioning/NovatelDecoder.h>
#include <gnss_positioning/SocketStream.h>
#include <gnss_positioning/SPP.h>
#include <gnss_positioning/SatPosCalc.h>
#include <gnss_positioning/ErrorCorrection.h>

namespace {

/**
 * @brief 将“系统 + PRN”组合为可用于容器索引的键
 * @param sys `gnss::GnssSystem` 类型卫星系统枚举，当前支持 GPS/BDS 等系统
 * @param prn `uint32_t` 类型卫星 PRN 编号，取值范围应为各系统的有效 PRN 区间
 * @return `uint32_t` 类型组合键，可直接用于 `std::map` / `std::unordered_map` 索引
 * @exception 本函数不主动抛出异常
 */
uint32_t MakeKey(gnss::GnssSystem sys, uint32_t prn) {
    return static_cast<uint32_t>(sys) * 1000U + prn;
}

/**
 * @brief GPST 周内秒转 BDT 周内秒
 * @param t_gpst `double` 类型 GPST 周内秒，合法范围通常为 `[0, 604800)`
 * @return `double` 类型 BDT 周内秒，结果被归一化到 `[0, 604800)` 区间
 * @exception 本函数不主动抛出异常
 *
 * 工程口径：
 * - BDT = GPST - 14s（周内秒环绕处理）
 * - 用于调用 BDS 广播星历卫星状态计算接口（其入口时标为 BDT）
 */
double GpstToBdtSec(double t_gpst) {
    double out = t_gpst - gnss::SppConst::BDT_GPS_OFFSET;
    while (out >= gnss::SppConst::WEEK_SEC) out -= gnss::SppConst::WEEK_SEC;
    while (out < 0.0) out += gnss::SppConst::WEEK_SEC;
    return out;
}

/**
 * @brief 将 GPST 时间格式化为标准“年月日时分秒”字符串
 * @param gps_time `gnss::GPSTIME` 类型输入时间，包含 GPS 周和周内秒
 * @param out_date `std::string&` 类型输出参数，成功时写为 `YYYY-MM-DD`
 * @param out_time `std::string&` 类型输出参数，成功时写为 `HH:MM:SS`
 * @return `true` 表示转换成功；`false` 表示底层时间转换失败
 * @exception 若字符串流分配失败，则由标准库传播异常；本函数不显式抛出业务异常
 */
bool FormatGpsTimeYmdHms(const gnss::GPSTIME& gps_time, std::string& out_date, std::string& out_time) {
    gnss::COMMONTIME ct{};
    if (gnss::GpsTimeToCommonTime(gps_time, ct) != 0) return false;

    std::ostringstream oss_date;
    oss_date << std::setw(4) << std::setfill('0') << ct.Year << "-"
             << std::setw(2) << std::setfill('0') << ct.Month << "-"
             << std::setw(2) << std::setfill('0') << ct.Day;

    int sec_i = static_cast<int>(ct.Second);
    if (sec_i < 0) sec_i = 0;
    if (sec_i > 59) sec_i = 59;

    std::ostringstream oss_time;
    oss_time << std::setw(2) << std::setfill('0') << ct.Hour << ":"
             << std::setw(2) << std::setfill('0') << ct.Minute << ":"
             << std::setw(2) << std::setfill('0') << sec_i;

    out_date = oss_date.str();
    out_time = oss_time.str();
    return true;
}

/**
 * @brief 解析命令行中的配置文件路径
 * @param argc `int` 类型参数数量，取值范围应不小于 1
 * @param argv `char**` 类型参数数组，允许为 `nullptr`，但在 `argc > 0` 时通常应包含可读字符串
 * @return 若存在 `--config=...` 参数则返回其值；否则返回 `gnss::Config::kDefaultConfigPath`
 * @exception 本函数不主动抛出异常；若命令行字符串构造失败，则由标准库传播异常
 */
std::string ResolveConfigPathFromArgs(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i] ? std::string(argv[i]) : std::string();
        if (arg.rfind("--config=", 0) == 0) {
            return arg.substr(9);
        }
    }
    return gnss::Config::kDefaultConfigPath;
}

/**
 * @brief 将四元素浮点数组格式化为日志友好的字符串
 * @param values `std::array<double, 4>` 类型数组，表示 Klobuchar `alpha` 或 `beta` 参数
 * @return 形如 `[a0, a1, a2, a3]` 的字符串
 * @exception 若字符串流写入失败，则由标准库流状态记录错误，不主动抛出异常
 */
std::string FormatArray4(const std::array<double, 4>& values) {
    std::ostringstream oss;
    oss << "[" << values[0] << ", " << values[1] << ", " << values[2] << ", " << values[3] << "]";
    return oss.str();
}

/**
 * @brief 判断指定导航系统是否在当前配置下启用
 * @param systems `gnss::GnssSystemOptions` 类型系统开关配置
 * @param sys `gnss::GnssSystem` 类型卫星系统枚举
 * @return `true` 表示允许该系统进入后续卫星状态/SPP 处理链路
 */
bool IsSystemEnabled(const gnss::GnssSystemOptions& systems, gnss::GnssSystem sys) {
    if (sys == gnss::GnssSystem::GPS) return systems.enable_gps;
    if (sys == gnss::GnssSystem::BDS) return systems.enable_bds;
    return false;
}

/**
 * @brief 过滤当前配置允许进入处理链路的观测
 * @param obs_list 当前历元原始观测列表
 * @param systems `gnss::GnssSystemOptions` 类型系统开关配置
 * @return 仅包含启用系统观测的线性列表
 */
std::vector<gnss::ObsData> FilterEnabledObservations(const std::vector<gnss::ObsData>& obs_list,
                                                     const gnss::GnssSystemOptions& systems) {
    std::vector<gnss::ObsData> filtered;
    filtered.reserve(obs_list.size());
    for (const auto& obs : obs_list) {
        if (!IsSystemEnabled(systems, obs.system)) continue;
        filtered.push_back(obs);
    }
    return filtered;
}

/**
 * @brief 收集当前配置允许参与解算的星历列表
 * @param eph_store `std::map<uint32_t, gnss::Ephemeris>` 类型星历缓存
 * @param systems `gnss::GnssSystemOptions` 类型系统开关配置
 * @return 仅包含启用系统星历的线性列表
 */
std::vector<gnss::Ephemeris> CollectEnabledEphemeris(const std::map<uint32_t, gnss::Ephemeris>& eph_store,
                                                     const gnss::GnssSystemOptions& systems) {
    std::vector<gnss::Ephemeris> eph_list;
    eph_list.reserve(eph_store.size());
    for (const auto& kv : eph_store) {
        if (!IsSystemEnabled(systems, kv.second.system)) continue;
        eph_list.push_back(kv.second);
    }
    return eph_list;
}

/**
 * @brief 输出最终生效的关键配置参数
 * @param cfg `gnss::Config` 常量引用，表示已完成加载与校验的只读配置
 * @return 无
 * @exception 若日志流写入失败，则由标准输出流状态记录错误，不主动抛出异常
 *
 * 说明：
 * - 该函数专门用于运维确认启动参数是否与配置文件一致；
 * - Klobuchar 相关开关、`alpha/beta` 参数会被完整打印。
 */
void LogEffectiveConfig(const gnss::Config& cfg) {
    const auto& spp = cfg.sppOptions();
    const auto& ref = cfg.referenceEcef();

    LOG(INFO) << "CONFIG_PATH=" << cfg.sourcePath();
    LOG(INFO) << "INPUT_MODE=" << gnss::ToString(cfg.inputMode());
    if (cfg.fileInputEnabled()) {
        LOG(INFO) << "INPUT_PATH=" << cfg.inputPath();
    } else {
        LOG(INFO) << "NET_ENDPOINT=" << cfg.netIp() << ":" << cfg.netPort();
    }
    LOG(INFO) << "OUTPUT_DIR=" << cfg.outputDir();
    LOG(INFO) << "TERMINAL_OUTPUT=" << (cfg.terminalOutputEnabled() ? "true" : "false");
    LOG(INFO) << "FILE_OUTPUT=" << (cfg.fileOutputEnabled() ? "true" : "false");
    LOG(INFO) << "ENABLE_SPP=" << (cfg.sppEnabled() ? "true" : "false");
    LOG(INFO) << "ENABLE_SAT_STATE=" << (cfg.satStateEnabled() ? "true" : "false");
    LOG(INFO) << "ENABLE_GPS=" << (cfg.gnssSystems().enable_gps ? "true" : "false");
    LOG(INFO) << "ENABLE_BDS=" << (cfg.gnssSystems().enable_bds ? "true" : "false");
    LOG(INFO) << "SPP_MODE=" << gnss::ToString(spp.frequency_mode);
    LOG(INFO) << "SPP_ENABLE_TROPO=" << (spp.enable_tropo ? "true" : "false");
    LOG(INFO) << "SPP_ENABLE_TGD=" << (spp.enable_tgd ? "true" : "false");
    LOG(INFO) << "SPP_ENABLE_KLOBUCHAR_MODEL=" << (spp.enable_klobuchar_model ? "true" : "false");
    LOG(INFO) << "SPP_ELEV_MASK_DEG=" << spp.elev_mask_deg;
    LOG(INFO) << "SPP_SNR_MASK_DBHZ=" << spp.snr_mask_dbhz;
    LOG(INFO) << "GPS_KLOBUCHAR_ENABLED=" << (spp.gps_klobuchar.enabled ? "true" : "false");
    LOG(INFO) << "GPS_KLOBUCHAR_ALPHA=" << FormatArray4(spp.gps_klobuchar.alpha);
    LOG(INFO) << "GPS_KLOBUCHAR_BETA=" << FormatArray4(spp.gps_klobuchar.beta);
    LOG(INFO) << "BDS_KLOBUCHAR_ENABLED=" << (spp.bds_klobuchar.enabled ? "true" : "false");
    LOG(INFO) << "BDS_KLOBUCHAR_ALPHA=" << FormatArray4(spp.bds_klobuchar.alpha);
    LOG(INFO) << "BDS_KLOBUCHAR_BETA=" << FormatArray4(spp.bds_klobuchar.beta);
    LOG(INFO) << "REFERENCE_ECEF_XYZ=" << ref.x << "," << ref.y << "," << ref.z;
}

/**
 * @brief 终端输出模块：负责运行过程中的可读性输出
 *
 * 说明：
 * - 该模块仅做“展示”，不参与任何定位领域逻辑判断；
 * - 输出开关由 `gnss::Config::terminalOutputEnabled()` 控制。
 */
class TerminalOutput {
public:
    /**
     * @brief 设置终端输出是否启用
     * @param enabled `bool` 类型开关，`true` 表示允许输出，`false` 表示静默
     * @return 无
     * @exception 本函数不主动抛出异常
     */
    void setEnabled(bool enabled) { enabled_ = enabled; }

    /**
     * @brief 输出程序启动摘要
     * @param cfg `gnss::Config` 常量引用，表示已校验通过的只读配置
     * @return 无
     * @exception 本函数不主动抛出异常；若标准输出失败，则由流状态记录错误
     */
    void printStartup(const gnss::Config& cfg) {
        if (!enabled_) return;
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "GNSS SPP 定位程序" << std::endl;
        std::cout << "INPUT_MODE=" << gnss::ToString(cfg.inputMode()) << std::endl;
        if (cfg.fileInputEnabled()) {
            std::cout << "INPUT=" << cfg.inputPath() << std::endl;
        } else {
            std::cout << "NET_ENDPOINT=" << cfg.netIp() << ":" << cfg.netPort() << std::endl;
        }
        std::cout << "OUTPUT_DIR=" << cfg.outputDir() << std::endl;
        std::cout << "SPP_MODE=" << gnss::ToString(cfg.sppOptions().frequency_mode) << std::endl;
        std::cout << "CONFIG=" << cfg.sourcePath() << std::endl;
    }

    /**
     * @brief 输出单历元摘要（用于观察解算是否稳定、卫星匹配是否满足）
     * @param gps_time `gnss::GPSTIME` 类型当前历元时间，周内秒通常位于 `[0, 604800)`
     * @param report `gnss::SppEpochReport` 常量引用，包含本历元解算摘要
     * @param ok `bool` 类型解算标志，`true` 表示成功，`false` 表示失败
     * @return 无
     * @exception 本函数不主动抛出异常；若输出流失败，则由流状态记录
     */
    void printEpoch(const gnss::GPSTIME& gps_time, const gnss::SppEpochReport& report, bool ok) {
        if (!enabled_) return;
        if (!report.attempted) return;
        std::string date;
        std::string time;
        if (!FormatGpsTimeYmdHms(gps_time, date, time)) {
            date = "0000-00-00";
            time = "00:00:00";
        }

        std::cout << "TIME=" << date << " " << time
                  << " TOW=" << std::fixed << std::setprecision(3) << report.tow_gpst
                  << " OK=" << (ok ? 1 : 0)
                  << " OBS(G/B)=" << report.obs_g << "/" << report.obs_b
                  << " MATCH(G/B)=" << report.match_g << "/" << report.match_b
                  << " CLK(m)=" << report.clk_gps_m
                  << " ISB(m)=" << report.isb_bds_m
                  << " ECEF-X/m=" << std::fixed << std::setprecision(4) << report.pos.x
                  << " ECEF-Y/m=" << std::fixed << std::setprecision(4) << report.pos.y
                  << " ECEF-Z/m=" << std::fixed << std::setprecision(4) << report.pos.z
                  << std::fixed << std::setprecision(3)
                  << " PDOP=" << report.pdop
                  << std::endl;
    }

    /**
     * @brief 输出全流程统计摘要（用于处理量级与异常跳过原因分析）
     * @param total_bytes `uint64_t` 类型累计读取字节数，适用于长时间实时流统计
     * @param range_frames `uint32_t` 类型 RANGE 历元数量，消息 ID 为 43
     * @param gps_eph `uint32_t` 类型 GPS 星历数量，消息 ID 为 7
     * @param bds_eph `uint32_t` 类型 BDS 星历数量，消息 ID 为 1696
     * @param solve_attempt `uint32_t` 类型进入 SPP 解算的历元数量
     * @param spp_ok `uint32_t` 类型解算成功历元数量
     * @param spp_fail `uint32_t` 类型解算失败历元数量
     * @param skip_no_eph `uint32_t` 类型因无星历跳过的历元数量
     * @param skip_low_match `uint32_t` 类型因匹配不足跳过的历元数量
     * @param sat_total `uint64_t` 类型卫星状态计算尝试总次数
     * @param sat_no_eph `uint64_t` 类型因无星历跳过的卫星状态次数
     * @param sat_ok `uint64_t` 类型卫星状态计算成功次数
     * @param sat_valid `uint64_t` 类型卫星状态数值有效次数
     * @return 无
     * @exception 本函数不主动抛出异常；若输出流失败，则由流状态记录
     */
    void printSummary(uint64_t total_bytes,
                      uint32_t range_frames,
                      uint32_t gps_eph,
                      uint32_t bds_eph,
                      uint32_t solve_attempt,
                      uint32_t spp_ok,
                      uint32_t spp_fail,
                      uint32_t skip_no_eph,
                      uint32_t skip_low_match,
                      uint64_t sat_total,
                      uint64_t sat_no_eph,
                      uint64_t sat_ok,
                      uint64_t sat_valid) {
        if (!enabled_) return;
        std::cout << "SUMMARY bytes=" << total_bytes
                  << " RANGE=" << range_frames
                  << " GPSEPHEM=" << gps_eph
                  << " BDSEPHEM=" << bds_eph
                  << " SOLVE_ATTEMPT=" << solve_attempt
                  << " SPP_OK=" << spp_ok
                  << " SPP_FAIL=" << spp_fail
                  << " SKIP_NO_EPH=" << skip_no_eph
                  << " SKIP_LOW_MATCH=" << skip_low_match
                  << " SAT_TOTAL=" << sat_total
                  << " SAT_NO_EPH=" << sat_no_eph
                  << " SAT_OK=" << sat_ok
                  << " SAT_VALID=" << sat_valid
                  << std::endl;
    }

private:
    bool enabled_ = true;
};

/**
 * @brief 文件管理模块：统一管理输出文件的创建、表头与逐历元写入
 *
 * 设计要点：
 * - openAll() 负责创建输出目录并按开关打开文件与写入表头；
 * - writeXxx() 负责在主循环中按需写入，所有写入函数均为“可重入安全”早返回形式；
 * - 该模块不做任何定位领域计算，仅做格式化与落盘。
 */
class FileManager {
public:
    /**
     * @brief 构造文件管理器
     * @param cfg `gnss::Config` 常量引用，表示已完成加载和校验的全局配置
     */
    explicit FileManager(const gnss::Config& cfg) : cfg_(cfg) {}

    /**
     * @brief 打开所有启用的输出文件并写入表头
     * @return `true` 表示输出目录和目标文件均已准备完成；`false` 表示目录创建或文件打开失败
     * @exception 本函数不主动抛出异常；文件系统错误通过返回值和流状态表达
     */
    bool openAll() {
        // 总落盘关闭时直接返回，避免触发任何目录或文件副作用。
        if (!cfg_.fileOutputEnabled()) return true;

        std::error_code ec;
        std::filesystem::create_directories(cfg_.outputDir(), ec);
        if (ec) return false;

        if (cfg_.fileSppSolutionEnabled()) {
            spp_path_ = (std::filesystem::path(cfg_.outputDir()) / "spp_solution.txt").string();
            spp_file_.open(spp_path_);
            if (!spp_file_.is_open()) return false;
            spp_file_.setf(std::ios::fixed, std::ios::floatfield);
            spp_file_ << "Wk SOW ECEF-X/m ECEF-Y/m ECEF-Z/m REF-ECEF-X/m REF-ECEF-Y/m REF-ECEF-Z/m EAST/m NORTH/m UP/m B/deg L/deg H/m VX/m VY/m VZ/m PDOP SigmaP SigmaV GS BS n" << std::endl;
        }

        if (cfg_.fileSatStatesEnabled()) {
            sat_path_ = (std::filesystem::path(cfg_.outputDir()) / "sat_states.txt").string();
            sat_file_.open(sat_path_);
            if (!sat_file_.is_open()) return false;
            sat_file_ << "* 通用时 (计算时刻)" << std::endl;
            sat_file_ << "* GPS周，周内秒 (计算时刻)" << std::endl;
            sat_file_ << "卫星号 POS_X(km) POS_Y(km) POS_Z(km) VEL_X(m/s) VEL_Y(m/s) VEL_Z(m/s) CLK(us) CLK_VEL(us/sec)" << std::endl;
            sat_file_ << "注意：卫星号前“B”表示广播星历，卫星位置、钟差等均未经任何改正" << std::endl;
        }

        if (cfg_.fileEphemerisEnabled()) {
            gps_eph_path_ = (std::filesystem::path(cfg_.outputDir()) / "gps_ephem.txt").string();
            bds_eph_path_ = (std::filesystem::path(cfg_.outputDir()) / "bds_ephem.txt").string();
            gps_eph_file_.open(gps_eph_path_);
            bds_eph_file_.open(bds_eph_path_);
            if (!gps_eph_file_.is_open() || !bds_eph_file_.is_open()) return false;
            gps_eph_file_.setf(std::ios::fixed, std::ios::floatfield);
            bds_eph_file_.setf(std::ios::fixed, std::ios::floatfield);
            gps_eph_file_ << std::setprecision(16);
            bds_eph_file_ << std::setprecision(16);
            gps_eph_file_ << "TOW(s) PRN HEALTH IODE1 IODE2 IODC WEEK ZWEEK TOE(s) TOC(s) "
                             "A(m) DN(rad/s) M0(rad) E OMEGA(rad) "
                             "CUC CUS CRC CRS CIC CIS "
                             "I0(rad) IDOT(rad/s) OMEGA0(rad) OMEGADOT(rad/s) "
                             "AF0(s) AF1(s/s) AF2(s/s^2) TGD(s)"
                          << std::endl;
            bds_eph_file_ << "PRN WEEK URA(m) HEALTH TGD1(s) TGD2(s) AODC TOC(s) "
                             "AF0(s) AF1(s/s) AF2(s/s^2) AODE TOE(s) SQRT_A(sqrt(m)) "
                             "E OMEGA(rad) DN(rad/s) M0(rad) OMEGA0(rad) OMEGADOT(rad/s) "
                             "I0(rad) IDOT(rad/s) CUC CUS CRC CRS CIC CIS"
                          << std::endl;
        }

        if (cfg_.fileObservationsEnabled()) {
            obs_path_ = (std::filesystem::path(cfg_.outputDir()) / "range_obs.txt").string();
            obs_file_.open(obs_path_);
            if (!obs_file_.is_open()) return false;
            obs_file_.setf(std::ios::fixed, std::ios::floatfield);
            obs_file_ << std::setprecision(3);
            obs_file_ << "WEEK TOW SYS PRN FREQIDX P(m) L(cycle) D(Hz) SNR(dBHz)" << std::endl;
        }

        if (cfg_.networkInputEnabled()) {
            raw_stream_path_ = (std::filesystem::path(cfg_.outputDir()) / "realtime_input.log").string();
            raw_stream_file_.open(raw_stream_path_, std::ios::binary);
            if (!raw_stream_file_.is_open()) return false;
        }

        return true;
    }

    /**
     * @brief 写入单历元 SPP 报告（spp_solution.txt）
     * @param gps_time `gnss::GPSTIME` 类型当前历元时间
     * @param report `gnss::SppEpochReport` 常量引用，包含本历元定位结果和统计字段
     * @return 无
     * @exception 本函数不主动抛出异常；当文件未打开或落盘关闭时直接返回
     *
     * 说明：
     * - report.attempted / report.ok 等字段由 SPP 模块负责填充；
     * - 此处仅按表头顺序输出，便于后处理脚本解析。
     */
    void writeSppReport(const gnss::GPSTIME& gps_time, const gnss::SppEpochReport& report) {
        // 任一文件开关关闭时直接返回，保持调用端无需额外判断。
        if (!cfg_.fileOutputEnabled() || !cfg_.fileSppSolutionEnabled()) return;
        if (!spp_file_.is_open()) return;
        const auto& ref = cfg_.referenceEcef();
        if (!has_ref_ && (ref.x != 0.0 || ref.y != 0.0 || ref.z != 0.0)) {
            ref_xyz_.x = ref.x;
            ref_xyz_.y = ref.y;
            ref_xyz_.z = ref.z;
            has_ref_ = true;
        }
        if (!has_ref_ && report.ok) {
            ref_xyz_ = report.pos;
            has_ref_ = true;
        }

        double east = 0.0, north = 0.0, up = 0.0;
        if (has_ref_) {
            (void)gnss::XyzToEnu(ref_xyz_, report.pos, east, north, up);
        }

        gnss::BLH blh{};
        double b_deg = 0.0, l_deg = 0.0, h_m = 0.0;
        if (gnss::XYZToBLH(report.pos, blh) == 0) {
            b_deg = blh.latitude * 180.0 / M_PI;
            l_deg = blh.longitude * 180.0 / M_PI;
            h_m = blh.height;
        }

        const double sigma_p = report.sigma0 * report.pdop;
        const double sigma_v = report.sigma_v;

        spp_file_ << std::setw(4) << gps_time.Week << " "
                  << std::setw(11) << std::fixed << std::setprecision(3) << gps_time.SecOfWeek << " "
                  << std::setw(14) << std::fixed << std::setprecision(4) << report.pos.x << " "
                  << std::setw(14) << std::fixed << std::setprecision(4) << report.pos.y << " "
                  << std::setw(14) << std::fixed << std::setprecision(4) << report.pos.z << " "
                  << std::setw(14) << std::fixed << std::setprecision(4) << (has_ref_ ? ref_xyz_.x : 0.0) << " "
                  << std::setw(14) << std::fixed << std::setprecision(4) << (has_ref_ ? ref_xyz_.y : 0.0) << " "
                  << std::setw(14) << std::fixed << std::setprecision(4) << (has_ref_ ? ref_xyz_.z : 0.0) << " "
                  << std::setw(10) << std::fixed << std::setprecision(4) << east << " "
                  << std::setw(10) << std::fixed << std::setprecision(4) << north << " "
                  << std::setw(10) << std::fixed << std::setprecision(4) << up << " "
                  << std::setw(13) << std::fixed << std::setprecision(9) << b_deg << " "
                  << std::setw(13) << std::fixed << std::setprecision(9) << l_deg << " "
                  << std::setw(11) << std::fixed << std::setprecision(4) << h_m << " "
                  << std::setw(10) << std::fixed << std::setprecision(4) << report.vel.x << " "
                  << std::setw(10) << std::fixed << std::setprecision(4) << report.vel.y << " "
                  << std::setw(10) << std::fixed << std::setprecision(4) << report.vel.z << " "
                  << std::setw(6) << std::fixed << std::setprecision(3) << report.pdop << " "
                  << std::setw(7) << std::fixed << std::setprecision(3) << sigma_p << " "
                  << std::setw(7) << std::fixed << std::setprecision(3) << sigma_v << " "
                  << std::setw(2) << report.gs_used << " "
                  << std::setw(2) << report.bs_used << " "
                  << report.sat_used
                  << std::endl;
    }

    /**
     * @brief 写入卫星状态文件的历元分隔头（通用时 + GPS 周/周内秒）
     * @param gps_time `gnss::GPSTIME` 类型当前历元时间，包含周号和周内秒
     * @return 无
     * @exception 本函数不主动抛出异常；若文件未打开则直接返回
     */
    void writeSatEpochHeader(const gnss::GPSTIME& gps_time) {
        if (!cfg_.fileOutputEnabled() || !cfg_.fileSatStatesEnabled()) return;
        if (!sat_file_.is_open()) return;
        gnss::COMMONTIME ct;
        gnss::GpsTimeToCommonTime(gps_time, ct);
        sat_file_ << std::endl;
        sat_file_ << "* " << ct.Year << " " << ct.Month << " " << ct.Day << " " << ct.Hour << " " << ct.Minute << " "
                 << std::fixed << std::setprecision(8) << ct.Second << std::endl;
        sat_file_ << "* " << gps_time.Week << " " << std::fixed << std::setprecision(8) << gps_time.SecOfWeek << std::endl;
    }

    /**
     * @brief 写入单颗卫星的广播星历状态（sat_states.txt）
     * @param sys `gnss::GnssSystem` 类型卫星系统，当前按 GPS/BDS 输出
     * @param prn `uint32_t` 类型卫星 PRN 号，取值应落在系统合法区间
     * @param st `gnss::SatState` 常量引用，包含位置、速度、钟差与钟速
     * @return 无
     * @exception 本函数不主动抛出异常；若文件未打开则直接返回
     *
     * 注意：
     * - 输出文件用于验证卫星广播轨道计算口径；
     * - “B”前缀表示仅广播星历状态，不叠加 SPP 观测改正项。
     */
    void writeSatStateLine(gnss::GnssSystem sys, uint32_t prn, const gnss::SatState& st) {
        if (!cfg_.fileOutputEnabled() || !cfg_.fileSatStatesEnabled()) return;
        if (!sat_file_.is_open()) return;
        char sys_char = (sys == gnss::GnssSystem::GPS) ? 'G' : 'C';
        sat_file_ << "B" << sys_char << std::setw(2) << std::setfill('0') << prn << std::setfill(' ') << "  "
                  << std::fixed << std::setprecision(6)
                  << std::setw(12) << (st.pos_ecef.x / 1000.0) << " "
                  << std::setw(12) << (st.pos_ecef.y / 1000.0) << " "
                  << std::setw(12) << (st.pos_ecef.z / 1000.0) << " "
                  << std::setw(9) << std::setprecision(3) << st.vel_ecef.x << " "
                  << std::setw(9) << std::setprecision(3) << st.vel_ecef.y << " "
                  << std::setw(9) << std::setprecision(3) << st.vel_ecef.z << " "
                  << std::setw(13) << std::setprecision(6) << (st.clk_s * 1e6) << " "
                  << std::setw(13) << std::setprecision(6) << (st.clk_drift * 1e6)
                  << std::endl;
    }

    /**
     * @brief 写入 GPS 星历（gps_ephem.txt）
     * @param e `gnss::GpsEphem` 常量引用，来自解码器缓存的 GPS 广播星历
     * @return 无
     * @exception 本函数不主动抛出异常；若文件未打开则直接返回
     */
    void writeGpsEphem(const gnss::GpsEphem& e) {
        if (!cfg_.fileOutputEnabled() || !cfg_.fileEphemerisEnabled()) return;
        if (!gps_eph_file_.is_open()) return;
        gps_eph_file_ << e.tow << " "
                      << e.prn << " "
                      << e.health << " "
                      << e.iode1 << " "
                      << e.iode2 << " "
                      << e.iodc << " "
                      << e.week << " "
                      << e.zweek << " "
                      << e.toe << " "
                      << e.toc << " "
                      << e.majaxis << " "
                      << e.dN << " "
                      << e.ma << " "
                      << e.ecc << " "
                      << e.argper << " "
                      << e.cuc << " " << e.cus << " "
                      << e.crc << " " << e.crs << " "
                      << e.cic << " " << e.cis << " "
                      << e.inc << " "
                      << e.inc_dot << " "
                      << e.omegazero << " "
                      << e.omegadot << " "
                      << e.af0 << " "
                      << e.af1 << " "
                      << e.af2 << " "
                      << e.tgd
                      << std::endl;
    }

    /**
     * @brief 写入 BDS 星历（bds_ephem.txt）
     * @param e `gnss::BdsEphem` 常量引用，来自解码器缓存的 BDS 广播星历
     * @return 无
     * @exception 本函数不主动抛出异常；若文件未打开则直接返回
     */
    void writeBdsEphem(const gnss::BdsEphem& e) {
        if (!cfg_.fileOutputEnabled() || !cfg_.fileEphemerisEnabled()) return;
        if (!bds_eph_file_.is_open()) return;
        bds_eph_file_ << e.prn << " "
                      << e.week << " "
                      << e.ura << " "
                      << e.health << " "
                      << e.tgd1 << " "
                      << e.tgd2 << " "
                      << e.aodc << " "
                      << static_cast<double>(e.toc) << " "
                      << e.af0 << " "
                      << e.af1 << " "
                      << e.af2 << " "
                      << e.aode << " "
                      << static_cast<double>(e.toe) << " "
                      << e.rootA << " "
                      << e.ecc << " "
                      << e.omega << " "
                      << e.dN << " "
                      << e.m0 << " "
                      << e.omegazero << " "
                      << e.omegadot << " "
                      << e.i0 << " "
                      << e.idot << " "
                      << e.cuc << " " << e.cus << " "
                      << e.crc << " " << e.crs << " "
                      << e.cic << " " << e.cis
                      << std::endl;
    }

    /**
     * @brief 写入 RANGE 观测（range_obs.txt）
     * @param week `uint16_t` 类型当前 GPS 周号，取值范围 `[0, 65535]`
     * @param tow `double` 类型当前 GPST 周内秒，通常位于 `[0, 604800)`
     * @param obs_list `std::vector<gnss::ObsData>` 常量引用，表示当前历元全部观测
     * @return 无
     * @exception 本函数不主动抛出异常；若文件未打开则直接返回
     */
    void writeObs(uint16_t week, double tow, const std::vector<gnss::ObsData>& obs_list) {
        if (!cfg_.fileOutputEnabled() || !cfg_.fileObservationsEnabled()) return;
        if (!obs_file_.is_open()) return;
        for (const auto& o : obs_list) {
            char sys_char = 'U';
            if (o.system == gnss::GnssSystem::GPS) sys_char = 'G';
            else if (o.system == gnss::GnssSystem::BDS) sys_char = 'C';
            obs_file_ << week << " " << tow << " " << sys_char << " " << o.prn << " " << o.freqIdx << " "
                      << o.pseudorange << " " << o.carrierPhase << " "
                      << static_cast<double>(o.doppler) << " " << static_cast<double>(o.snr) 
                      << std::endl;
        }
    }

    /**
     * @brief 写入网口原始二进制流（realtime_input.log）
     * @param data 原始字节数据首地址
     * @param size 待写入字节数
     * @return 无
     * @exception 本函数不主动抛出异常；若未启用落盘或文件未打开则直接返回
     */
    void writeRawStream(const uint8_t* data, std::size_t size) {
        if (!cfg_.fileOutputEnabled() || !cfg_.networkInputEnabled()) return;
        if (!raw_stream_file_.is_open() || data == nullptr || size == 0) return;
        raw_stream_file_.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
    }

    /**
     * @brief 获取 spp_solution.txt 输出路径（若未启用则为空字符串）
     * @return `const std::string&` 类型输出路径引用；未启用时可能为空字符串
     * @exception 本函数不主动抛出异常
     */
    const std::string& sppPath() const { return spp_path_; }
    /**
     * @brief 获取 sat_states.txt 输出路径（若未启用则为空字符串）
     * @return `const std::string&` 类型输出路径引用；未启用时可能为空字符串
     * @exception 本函数不主动抛出异常
     */
    const std::string& satPath() const { return sat_path_; }

private:
    /**
     * @brief 运行时配置引用（由 main() 生命周期托管）
     */
    const gnss::Config& cfg_;
    bool has_ref_ = false;
    gnss::XYZ ref_xyz_ = {0.0, 0.0, 0.0};

    std::ofstream spp_file_;
    std::ofstream sat_file_;
    std::ofstream gps_eph_file_;
    std::ofstream bds_eph_file_;
    std::ofstream obs_file_;
    std::ofstream raw_stream_file_;

    std::string spp_path_;
    std::string sat_path_;
    std::string gps_eph_path_;
    std::string bds_eph_path_;
    std::string obs_path_;
    std::string raw_stream_path_;
};

/**
 * @brief SPP 核心封装：隔离 main() 与 SPP 模块接口细节
 *
 * 设计目的：
 * - main() 负责数据流与 I/O；SPP 负责定位领域逻辑；
 * - 在此处集中注入运行时参数（高度角掩码、SNR 掩码、IF 开关等）。
 */
class SppCore {
public:
    /**
     * @brief 构造并初始化 SPP 解算器参数
     * @param cfg `gnss::Config` 常量引用，表示已完成校验的只读参数集合
     * @return 无
     * @exception 若底层 `SPP` 参数注入失败，则由标准库或下层代码传播异常
     */
    explicit SppCore(const gnss::Config& cfg) {
        applyConfig(cfg);
    }

    /**
     * @brief 将集中式配置应用到 SPP 解算器
     * @param cfg `gnss::Config` 常量引用，要求已经过范围与依赖关系校验
     * @return 无
     * @exception 本函数不主动抛出异常；若底层容器或字符串操作失败，则由标准库传播异常
     */
    void applyConfig(const gnss::Config& cfg) {
        const auto& systems = cfg.gnssSystems();
        const auto& spp = cfg.sppOptions();
        solver_.setElevationMaskDeg(spp.elev_mask_deg);
        solver_.setSnrMaskDbHz(spp.snr_mask_dbhz);
        solver_.setEnableIonoFree(spp.useIonoFree());
        solver_.setEnableTroposphere(spp.enable_tropo);
        solver_.setEnableKlobucharIono(spp.useKlobuchar());
        solver_.setEnableTgdCorrection(spp.enable_tgd);
        solver_.setEnableGps(systems.enable_gps);
        solver_.setEnableBds(systems.enable_bds);

        if (spp.gps_klobuchar.enabled) {
            solver_.setGpsKlobucharParams(spp.gps_klobuchar.alpha.data(),
                                          spp.gps_klobuchar.beta.data());
        } else {
            solver_.setGpsKlobucharParams(nullptr, nullptr);
        }

        if (spp.bds_klobuchar.enabled) {
            solver_.setBdsKlobucharParams(spp.bds_klobuchar.alpha.data(),
                                          spp.bds_klobuchar.beta.data());
        } else {
            solver_.setBdsKlobucharParams(nullptr, nullptr);
        }

        solver_.setVerbose(false);
    }

    /**
     * @brief 执行单历元 SPP 解算并生成报告
     * @param tow_gpst `double` 类型历元时刻，表示 GPST 周内秒，通常位于 `[0, 604800)`
     * @param obs_list `std::vector<gnss::ObsData>` 常量引用，表示当前历元观测列表
     * @param eph_list `std::vector<gnss::Ephemeris>` 常量引用，表示当前可用星历列表
     * @param report `gnss::SppEpochReport&` 输出参数，返回本历元解算报告
     * @return `true` 表示解算成功；`false` 表示数据不足或数值失败
     * @exception 本函数不主动抛出异常；异常由底层解算器或标准库传播
     */
    bool compute(double tow_gpst,
                 const std::vector<gnss::ObsData>& obs_list,
                 const std::vector<gnss::Ephemeris>& eph_list,
                 gnss::SppEpochReport& report) {
        return solver_.computeWithReport(tow_gpst, obs_list, eph_list, report);
    }

    /**
     * @brief 获取最近一次解算位置（ECEF）
     * @return `std::vector<double>` 类型三元素位置向量，单位为米
     * @exception 本函数不主动抛出异常
     */
    std::vector<double> lastPosition() const { return solver_.getPosition(); }

private:
    gnss::SPP solver_;
};

/**
 * @brief 卫星状态计算封装：基于广播星历输出卫星位置/速度/钟差/钟速
 *
 * 说明：
 * - 用于生成 sat_states.txt（验证广播轨道计算）；
 * - 与 SPP 定位解算解耦：即使不启用 SPP，也可单独输出卫星状态用于诊断。
 */
class SatStateCore {
public:
    /**
     * @brief 在指定 GPST 周内秒计算单颗卫星状态
     * @param sys `gnss::GnssSystem` 类型卫星系统
     * @param prn `uint32_t` 类型卫星 PRN 号
     * @param tow_gpst `double` 类型计算时刻，表示 GPST 周内秒
     * @param eph `gnss::Ephemeris` 常量引用，表示统一格式广播星历
     * @param st `gnss::SatState&` 输出参数，返回卫星位置、速度与钟差状态
     * @return `true` 表示计算成功；`false` 表示系统不支持或状态计算失败
     * @exception 本函数不主动抛出异常；底层异常由计算函数传播
     *
     * 时间口径：
     * - GPS：直接使用 GPST；
     * - BDS：ComputeBdsSatState 接口使用 BDT，因此在此处做 GPST->BDT 转换。
     */
    static bool computeAtTow(gnss::GnssSystem sys,
                             uint32_t prn,
                             double tow_gpst,
                             const gnss::Ephemeris& eph,
                             gnss::SatState& st) {
        if (sys == gnss::GnssSystem::GPS) {
            gnss::GpsEph e = toGpsEph(eph);
            return gnss::ComputeGpsSatState(e, tow_gpst, 0.0, st);
        }
        if (sys == gnss::GnssSystem::BDS) {
            gnss::BdsEph e = toBdsEph(eph);
            return gnss::ComputeBdsSatState(e, GpstToBdtSec(tow_gpst), 0.0, st);
        }
        return false;
    }

private:
    /**
     * @brief 统一星历转 GPS 广播星历输入结构体（字段搬运）
     * @param eph `gnss::Ephemeris` 常量引用，表示统一格式星历
     * @return `gnss::GpsEph` 类型 GPS 广播星历输入结构体
     * @exception 本函数不主动抛出异常
     */
    static gnss::GpsEph toGpsEph(const gnss::Ephemeris& eph) {
        gnss::GpsEph out;
        out.prn = eph.prn;
        out.toc = eph.toc;
        out.toe = eph.toe;
        out.af0 = eph.af0;
        out.af1 = eph.af1;
        out.af2 = eph.af2;
        out.tgd = eph.tgd;
        out.sqrtA = eph.rootA;
        out.e = eph.ecc;
        out.i0 = eph.inc0;
        out.iDot = eph.inc_dot;
        out.Omega0 = eph.omega0;
        out.OmegaDot = eph.omega_dot;
        out.omega = eph.argper;
        out.M0 = eph.m0;
        out.deltaN = eph.dN;
        out.cuc = eph.cuc;
        out.cus = eph.cus;
        out.crc = eph.crc;
        out.crs = eph.crs;
        out.cic = eph.cic;
        out.cis = eph.cis;
        return out;
    }

    /**
     * @brief 统一星历转 BDS 广播星历输入结构体（字段搬运）
     * @param eph `gnss::Ephemeris` 常量引用，表示统一格式星历
     * @return `gnss::BdsEph` 类型 BDS 广播星历输入结构体
     * @exception 本函数不主动抛出异常
     */
    static gnss::BdsEph toBdsEph(const gnss::Ephemeris& eph) {
        gnss::BdsEph out;
        out.prn = eph.prn;
        out.toc = eph.toc;
        out.toe = eph.toe;
        out.af0 = eph.af0;
        out.af1 = eph.af1;
        out.af2 = eph.af2;
        out.tgd = eph.tgd;
        out.sqrtA = eph.rootA;
        out.e = eph.ecc;
        out.i0 = eph.inc0;
        out.iDot = eph.inc_dot;
        out.Omega0 = eph.omega0;
        out.OmegaDot = eph.omega_dot;
        out.omega = eph.argper;
        out.M0 = eph.m0;
        out.deltaN = eph.dN;
        out.cuc = eph.cuc;
        out.cus = eph.cus;
        out.crc = eph.crc;
        out.crs = eph.crs;
        out.cic = eph.cic;
        out.cis = eph.cis;
        return out;
    }
};

constexpr std::size_t kInputChunkBytes = 4096;

/**
 * @brief 单次输入读取结果
 */
struct InputChunk {
    std::size_t size = 0;
    bool timed_out = false;
    bool source_closed = false;
};

/**
 * @brief 统一封装文件/网口两种输入方式
 *
 * 设计目的：
 * - 让主循环只关心“拿到一批字节”；
 * - 保持 OEM 流式解码与后续 SPP 逻辑完全复用。
 */
class InputReader {
public:
    explicit InputReader(const gnss::Config& cfg) : cfg_(cfg) {}

    /**
     * @brief 打开配置指定的输入源
     * @param error_message 失败时写入错误描述
     * @return true 表示打开成功
     */
    bool open(std::string& error_message) {
        error_message.clear();
        if (cfg_.fileInputEnabled()) {
            in_file_.open(cfg_.inputPath(), std::ios::binary);
            if (!in_file_.is_open()) {
                error_message = "无法打开输入文件: " + cfg_.inputPath();
                return false;
            }
            return true;
        }
        return socket_.open(cfg_.netIp(), cfg_.netPort(), 10000, error_message);
    }

    /**
     * @brief 从当前输入源读取一块字节流
     * @param buffer 输出缓冲区
     * @param chunk 输出的读块状态
     * @param error_message 失败时写入错误描述
     * @return true 表示本次调用成功返回（即使超时或已到流结束）
     */
    bool read(std::array<uint8_t, kInputChunkBytes>& buffer,
              InputChunk& chunk,
              std::string& error_message) {
        chunk = {};
        error_message.clear();

        if (cfg_.fileInputEnabled()) {
            if (!in_file_.is_open()) {
                error_message = "输入文件尚未打开: " + cfg_.inputPath();
                return false;
            }

            in_file_.read(reinterpret_cast<char*>(buffer.data()), static_cast<std::streamsize>(buffer.size()));
            const std::streamsize count = in_file_.gcount();
            if (count > 0) {
                chunk.size = static_cast<std::size_t>(count);
                return true;
            }
            if (in_file_.eof()) {
                chunk.source_closed = true;
                return true;
            }

            error_message = "读取输入文件失败: " + cfg_.inputPath();
            return false;
        }

        std::size_t bytes_received = 0;
        const gnss::SocketReadStatus status = socket_.receive(buffer.data(), buffer.size(), bytes_received, error_message);
        if (status == gnss::SocketReadStatus::OK) {
            chunk.size = bytes_received;
            return true;
        }
        if (status == gnss::SocketReadStatus::TIMEOUT) {
            chunk.timed_out = true;
            return true;
        }
        if (status == gnss::SocketReadStatus::CLOSED) {
            chunk.source_closed = true;
            return true;
        }
        return false;
    }

private:
    const gnss::Config& cfg_;
    std::ifstream in_file_;
    gnss::SocketClient socket_;
};

} // namespace

/**
 * @brief 程序入口：驱动数据解码、预处理、SPP 解算与结果输出
 * @param argc `int` 类型命令行参数数量，取值范围应不小于 1
 * @param argv `char**` 类型命令行参数数组，支持通过 `--config=...` 指定配置文件路径
 * @return `0` 表示正常退出；`1` 表示输入或输出初始化失败；`2` 表示运行期间抛出异常
 * @exception 本函数捕获全部 `std::exception` 派生异常，并转换为返回码 `2`
 *
 * 主流程（按数据流驱动组织）：
 * 1) 读取命令行并一次性加载配置文件
 * 2) 初始化终端输出、输入源与输出文件
 * 3) 流式读取 OEM 二进制数据，解码消息帧
 * 4) 星历消息：缓存并落盘
 * 5) 观测消息：落盘、可选卫星状态输出、可选 SPP 解算输出
 * 6) 输出最终统计摘要
 */
int main(int argc, char** argv) {
    try {
        const std::string config_path = ResolveConfigPathFromArgs(argc, argv);
        const gnss::Config cfg = gnss::Config::LoadFromYaml(config_path);
        LogEffectiveConfig(cfg);

        TerminalOutput term;
        term.setEnabled(cfg.terminalOutputEnabled());
        term.printStartup(cfg);

        // 打开输入源：支持已有 OEM 日志文件与网口实时二进制流两种模式。
        InputReader input(cfg);
        std::string input_error;
        if (!input.open(input_error)) {
            std::cerr << "[ERROR] " << input_error << std::endl;
            return 1;
        }

        // 初始化输出目录与各类落盘文件（由配置控制）。
        FileManager files(cfg);
        if (!files.openAll()) {
            std::cerr << "[ERROR] 输出文件初始化失败: " << cfg.outputDir() << std::endl;
            return 1;
        }

        // 初始化解码器与 SPP 解算核心。
        gnss::NovatelDecoder decoder;
        SppCore spp(cfg);

        // 星历缓存：key=(sys,prn) -> Ephemeris。
        // 生命周期：贯穿整个程序运行，随着消息流持续更新（用于后续历元的星历匹配）。
        std::map<uint32_t, gnss::Ephemeris> eph_store;

        // 统计变量：用于终端输出 SUMMARY，与性能无关，生命周期贯穿 main()。
        uint64_t total_bytes = 0;
        uint32_t range_frames = 0;
        uint32_t gps_eph_count = 0;
        uint32_t bds_eph_count = 0;
        uint32_t spp_ok = 0;
        uint32_t spp_fail = 0;
        uint32_t solve_attempt = 0;
        uint32_t skip_no_eph = 0;
        uint32_t skip_low_match = 0;
        uint64_t sat_total = 0;
        uint64_t sat_no_eph = 0;
        uint64_t sat_ok = 0;
        uint64_t sat_valid = 0;

        bool has_first_ok = false;
        double first_ok_tow = 0.0;
        double last_ok_tow = 0.0;
        std::vector<double> last_ok_pos;
        double last_ok_clk_m = 0.0; 
        // 主循环：按块读取原始字节流，再逐字节喂给解码器完成完整消息拼接、CRC 校验与分发。
        std::array<uint8_t, kInputChunkBytes> chunk_buffer{};
        while (true) {
            InputChunk chunk;
            std::string read_error;
            if (!input.read(chunk_buffer, chunk, read_error)) {
                std::cerr << "[ERROR] " << read_error << std::endl;
                return 1;
            }
            if (chunk.timed_out) continue;
            if (chunk.source_closed) {
                std::cerr << "[INFO] 连接关闭: " << read_error
                          << " (已接收 " << total_bytes << " 字节, "
                          << range_frames << " 历元)" << std::endl;
                break;
            }

            total_bytes += chunk.size;
            files.writeRawStream(chunk_buffer.data(), chunk.size);

            for (std::size_t i = 0; i < chunk.size; ++i) {
                const uint8_t byte = chunk_buffer[i];
                if (!decoder.inputData(byte)) continue;

                uint16_t msg_id = decoder.getMsgId();

                // 消息 ID=7：GPSEPHEM（GPS 广播星历）。
                if (msg_id == 7) {
                    const auto& gps_eph = decoder.getGpsEphem();
                    if (gps_eph.prn >= 1 && gps_eph.prn <= 32) {
                        eph_store[MakeKey(gnss::GnssSystem::GPS, gps_eph.prn)] = gnss::ConvertFromGpsEphem(gps_eph);
                        gps_eph_count++;
                        files.writeGpsEphem(gps_eph);
                    }
                    continue;
                }

                // 消息 ID=1696：BDSEPHEMERIS（BDS 广播星历）。
                if (msg_id == 1696) {
                    const auto& bds_eph = decoder.getBdsEphem();
                    if (bds_eph.prn >= 1 && bds_eph.prn <= 63) {
                        eph_store[MakeKey(gnss::GnssSystem::BDS, bds_eph.prn)] = gnss::ConvertFromBdsEphem(bds_eph);
                        bds_eph_count++;
                        files.writeBdsEphem(bds_eph);
                    }
                    continue;
                }

                // 消息 ID=43：RANGE（观测数据）。其他消息类型在当前主程序中忽略。
                if (msg_id != 43) continue;

                range_frames++;
                double tow = decoder.getTow();
                const auto& obs_list = decoder.getObsData();
                const auto filtered_obs_list = FilterEnabledObservations(obs_list, cfg.gnssSystems());
                files.writeObs(decoder.getWeek(), tow, obs_list);

                // 卫星状态文件按历元写入两行时间头，便于后处理按历元分段解析。
                gnss::GPSTIME gps_time;
                gps_time.Week = decoder.getWeek();
                gps_time.SecOfWeek = tow;
                files.writeSatEpochHeader(gps_time);

                // 卫星状态输出：对本历元出现的（GPS/BDS、freqIdx=0）卫星，若能匹配到星历则计算状态并写入。
                if (cfg.satStateEnabled() && cfg.fileOutputEnabled() && cfg.fileSatStatesEnabled()) {
                    std::map<uint32_t, gnss::ObsData> uniq_obs;
                    for (const auto& obs : filtered_obs_list) {
                        if (obs.freqIdx != 0) continue;
                        uniq_obs[MakeKey(obs.system, obs.prn)] = obs;
                    }

                    for (const auto& kv : uniq_obs) {
                        const auto& obs = kv.second;
                        sat_total++;
                        auto it = eph_store.find(MakeKey(obs.system, obs.prn));
                        if (it == eph_store.end()) {
                            sat_no_eph++;
                            continue;
                        }
                        gnss::SatState st;
                        if (!SatStateCore::computeAtTow(obs.system, obs.prn, tow, it->second, st)) continue;
                        sat_ok++;
                        if (st.valid) sat_valid++;
                        files.writeSatStateLine(obs.system, obs.prn, st);
                    }
                }

                // SPP 解算开关：禁用时跳过定位，仅完成解析与落盘。
                if (!cfg.sppEnabled()) continue;

                // 将当前缓存的全部星历转为线性列表（SPP 内部自行按 sys/prn 匹配）。
                std::vector<gnss::Ephemeris> eph_list = CollectEnabledEphemeris(eph_store, cfg.gnssSystems());

                // 单历元 SPP 解算：输出 report（用于终端摘要与落盘）。
                gnss::SppEpochReport report;
                bool ok = spp.compute(tow, filtered_obs_list, eph_list, report);
                term.printEpoch(gps_time, report, ok);

                // 统计与落盘：attempted 表示满足基本条件并进入解算流程。
                if (report.attempted) solve_attempt++;
                if (ok) spp_ok++;
                else spp_fail++;
                if (!report.attempted && (report.eph_g + report.eph_b) == 0) skip_no_eph++;
                if (!report.attempted && (report.eph_g + report.eph_b) > 0) skip_low_match++;

                files.writeSppReport(gps_time, report);

                // 记录首个成功历元与最近成功历元（用于终端快速定位结果范围）。
                if (ok) {
                    if (!has_first_ok) {
                        has_first_ok = true;
                        first_ok_tow = report.tow_gpst;
                    }
                    last_ok_tow = report.tow_gpst;
                    last_ok_pos = spp.lastPosition();
                    last_ok_clk_m = report.clk_gps_m;
                }
            }
        }

        // 终端输出：全流程统计摘要。
        term.printSummary(total_bytes,
                          range_frames,
                          gps_eph_count,
                          bds_eph_count,
                          solve_attempt,
                          spp_ok,
                          spp_fail,
                          skip_no_eph,
                          skip_low_match,
                          sat_total,
                          sat_no_eph,
                          sat_ok,
                          sat_valid);

        // 终端输出：首个成功历元与最后成功历元的快照（便于确认是否全程稳定）。
        if (cfg.terminalOutputEnabled() && has_first_ok && last_ok_pos.size() == 3) {
            std::cout << "FIRST_OK_TOW=" << first_ok_tow << " LAST_OK_TOW=" << last_ok_tow << std::endl;
            std::cout << "LAST_OK_XYZ=" << last_ok_pos[0] << "," << last_ok_pos[1] << "," << last_ok_pos[2]
                      << " LAST_OK_CLK(m)=" << last_ok_clk_m << std::endl;
        }

        // 终端输出：输出目录提示（便于脚本或人工定位输出文件位置）。
        if (cfg.terminalOutputEnabled() && cfg.fileOutputEnabled()) {
            std::cout << "OUTPUT_DIR=" << cfg.outputDir() << std::endl;
        }

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 异常退出: " << e.what() << std::endl;
        return 2;
    }
}
