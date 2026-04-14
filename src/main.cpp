/**
 * @file main.cpp
 * @brief GNSS 数据流驱动的实时 SPP 单点定位程序（模块化架构）
 * @author Jiaoyang Wu
 * @date 2026-03-30
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
#include <map>
#include <filesystem>
#include <sstream>
#include <system_error>

#include <gnss_positioning/NavMath.h>
#include <gnss_positioning/NovatelDecoder.h>
#include <gnss_positioning/SPP.h>
#include <gnss_positioning/SatPosCalc.h>
#include <gnss_positioning/ErrorCorrection.h>

namespace {

/**
 * @brief 将“系统 + PRN”组合为可用于容器索引的键
 * @param sys 卫星系统（GPS/BDS 等）
 * @param prn 卫星 PRN 编号
 * @return 键值（sys*1000 + prn），用于 map/unordered_map 等容器索引
 */
uint32_t MakeKey(gnss::GnssSystem sys, uint32_t prn) {
    return static_cast<uint32_t>(sys) * 1000U + prn;
}

/**
 * @brief GPST 周内秒转 BDT 周内秒
 * @param t_gpst GPST 周内秒，范围 [0, 604800)
 * @return BDT 周内秒，范围 [0, 604800)
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
 * @param gps_time GPS 时间（周+周内秒）
 * @param out_date 输出日期字符串，格式：YYYY-MM-DD
 * @param out_time 输出时间字符串，格式：HH:MM:SS
 * @return true: 转换成功；false: 转换失败
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
 * @brief 运行时配置项（由命令行与配置文件驱动）
 *
 * 设计目的：
 * - 将应用层“数据流驱动”与“算法/输出策略”解耦，避免在主循环中散落大量硬编码开关。
 * - 支持通过配置文件热更新（可选）改变输出与算法开关，减少重复编译/部署成本。
 */
struct RuntimeConfig {
    //=== INPUT/OUTPUT PATH CONFIGURATION ===================================//
    /**
     * @brief 输入数据路径（NovAtel OEM 二进制日志）
     *
     * 启用条件：
     * - 程序启动必须可打开该文件，否则直接退出。
     */
    std::string input_path = "D:/gnss_positioning/Novatel OEM7 Data/01/NovatelOEM20211114-01.log";
    /**
     * @brief 输出目录（落盘文件统一写入该目录）
     *
     * 启用条件：
     * - file_output 为 true 时生效；
     * - openAll() 会尝试创建该目录。
     */
    std::string output_dir = "D:/gnss_positioning/output";
    /**
     * @brief 配置文件路径（key=value）
     *
     * 说明：
     * - 为空表示不加载配置文件，仅使用命令行与默认值。
     */
    std::string config_path;
    /**
     * @brief 配置文件热更新开关
     *
     * 依赖关系：
     * - 仅当 config_path 非空时生效；
     * - 每次 decoder 解码出一帧完整消息后检查 last_write_time 是否变化。
     */
    bool watch_config = false;

    //=== TERMINAL OUTPUT CONFIGURATION =====================================//
    /**
     * @brief 终端输出开关
     *
     * 控制范围：
     * - 启动信息、每历元摘要、最终统计信息的输出。
     *
     * 依赖关系：
     * - 不影响文件落盘；
     * - 不影响算法执行，仅控制 std::cout 输出。
     */
    bool terminal_output = true;

    //=== FILE OUTPUT CONFIGURATION =========================================//
    /**
     * @brief 总落盘开关
     *
     * 控制范围：
     * - false：不创建 output_dir，不打开任何输出文件，writeXxx() 直接返回；
     * - true：按各子文件开关决定是否打开对应文件。
     */
    bool file_output = true;
    /**
     * @brief SPP 解算结果落盘开关（spp_solution.txt）
     *
     * 依赖关系：
     * - file_output 为 true；
     * - enable_spp 为 true 时才会持续写入内容（未解算仅会有表头）。
     */
    bool file_spp_solution = true;
    /**
     * @brief 卫星状态落盘开关（sat_states.txt）
     *
     * 依赖关系：
     * - file_output 为 true；
     * - enable_sat_state 为 true；
     * - 仅对每历元 freqIdx==0 的卫星输出一条状态记录。
     */
    bool file_sat_states = false;
    /**
     * @brief 星历落盘开关（gps_ephem.txt / bds_ephem.txt）
     *
     * 依赖关系：
     * - file_output 为 true；
     * - 仅当输入流中出现对应星历消息（ID=7/1696）时写入。
     */
    bool file_ephemeris = false;
    /**
     * @brief 原始观测落盘开关（range_obs.txt）
     *
     * 依赖关系：
     * - file_output 为 true；
     * - 仅当输入流中出现 RANGE 消息（ID=43）时写入。
     */
    bool file_observations = false;

    //=== ALGORITHM CONFIGURATION ===========================================//
    /**
     * @brief 启用 SPP 单点定位解算
     *
     * 依赖关系：
     * - true：每个 RANGE 历元调用 SPP::computeWithReport 输出定位结果；
     * - false：跳过定位解算，仅进行星历/观测解析与（可选）卫星状态输出。
     */
    bool enable_spp = true;
    /**
     * @brief 启用卫星状态计算与输出
     *
     * 依赖关系：
     * - true：每个 RANGE 历元为可匹配星历的卫星计算广播星历状态并落盘；
     * - false：不计算卫星状态（可降低运行成本）。
     */
    bool enable_sat_state = true;

    //=== QUALITY CONTROL CONFIGURATION =====================================//
    /**
     * @brief 高度角掩码（deg）
     *
     * 控制范围：
     * - SPP 解算内部将低高度角卫星剔除/降权；
     * - 该阈值过大将降低可用卫星数，过小将引入更强多路径/对流层残差影响。
     */
    double elev_mask_deg = 10.0;
    /**
     * @brief SNR 掩码（dB-Hz）
     *
     * 控制范围：
     * - SPP 解算仅使用 SNR 不低于该阈值的观测，降低低质量跟踪对解算的影响。
     */
    double snr_mask_dbhz = 30.0;
    /**
     * @brief 启用无电离层组合（IF）伪距
     *
     * 启用条件：
     * - 需要同一卫星同时存在 freqIdx=0 与 freqIdx=1 的有效伪距观测；
     * - 若缺少第二频点，SPP 内会按策略跳过该卫星（以满足“单频不参与本次解算”的工程约束）。
     */
    bool enable_iono_free = true;
    bool enable_tropo = true;
    bool enable_klobuchar = true;
    bool enable_tgd = true;

    double ref_ecef_x = -2267804.5260;
    double ref_ecef_y = 5009342.3720;
    double ref_ecef_z = 3220991.8630;

    /**
     * @brief 启用落盘（便捷开关）
     * @return 无
     */
    void enableFileOutput() { file_output = true; }
    /**
     * @brief 禁用落盘（便捷开关）
     * @return 无
     */
    void disableFileOutput() { file_output = false; }
    /**
     * @brief 启用终端输出（便捷开关）
     * @return 无
     */
    void enableTerminalOutput() { terminal_output = true; }
    /**
     * @brief 禁用终端输出（便捷开关）
     * @return 无
     */
    void disableTerminalOutput() { terminal_output = false; }
    /**
     * @brief 启用 SPP（便捷开关）
     * @return 无
     */
    void enableSpp() { enable_spp = true; }
    /**
     * @brief 禁用 SPP（便捷开关）
     * @return 无
     */
    void disableSpp() { enable_spp = false; }
    /**
     * @brief 启用卫星状态落盘（便捷开关）
     * @return 无
     */
    void enableSatStateOutput() { file_sat_states = true; }
    /**
     * @brief 禁用卫星状态落盘（便捷开关）
     * @return 无
     */
    void disableSatStateOutput() { file_sat_states = false; }
};

/**
 * @brief 配置管理器：命令行解析 + 配置文件加载 + 可选热更新
 *
 * 设计要点：
 * - 命令行参数优先级最高（先写入 cfg_），随后若提供 config_path 则读取配置文件进行覆盖。
 * - 热更新仅在 watch_config=true 且 config_path 非空时启用。
 */
class ConfigManager {
public:
    /**
     * @brief 获取当前配置（只读）
     * @return 配置只读引用
     */
    const RuntimeConfig& config() const { return cfg_; }
    /**
     * @brief 获取当前配置（可写）
     * @return 配置可写引用
     */
    RuntimeConfig& config() { return cfg_; }

    /**
     * @brief 从命令行参数加载配置
     * @param argc 参数数量
     * @param argv 参数数组
     * @return 无
     *
     * 支持的参数示例：
     * - --input=xxx
     * - --outdir=xxx
     * - --config=xxx
     * - --elev-mask=10.0
     * - --snr-mask=30.0
     * - --iono-free / --no-iono-free
     * - --no-terminal / --no-file / --no-spp / --no-sat 等
     */
    void loadFromArgs(int argc, char** argv) {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i] ? std::string(argv[i]) : std::string();
            if (arg.rfind("--input=", 0) == 0) cfg_.input_path = arg.substr(8);
            else if (arg.rfind("--outdir=", 0) == 0) cfg_.output_dir = arg.substr(9);
            else if (arg.rfind("--config=", 0) == 0) cfg_.config_path = arg.substr(9);
            else if (arg == "--no-terminal") cfg_.terminal_output = false;
            else if (arg == "--no-file") cfg_.file_output = false;
            else if (arg == "--no-spp") cfg_.enable_spp = false;
            else if (arg == "--no-sat") cfg_.enable_sat_state = false;
            else if (arg == "--no-spp-file") cfg_.file_spp_solution = false;
            else if (arg == "--no-sat-file") cfg_.file_sat_states = false;
            else if (arg == "--no-eph-file") cfg_.file_ephemeris = false;
            else if (arg == "--no-obs-file") cfg_.file_observations = false;
            else if (arg == "--no-config-watch") cfg_.watch_config = false;
            else if (arg.rfind("--elev-mask=", 0) == 0) cfg_.elev_mask_deg = std::stod(arg.substr(12));
            else if (arg.rfind("--snr-mask=", 0) == 0) cfg_.snr_mask_dbhz = std::stod(arg.substr(11));
            else if (arg == "--iono-free") cfg_.enable_iono_free = true;
            else if (arg == "--no-iono-free") cfg_.enable_iono_free = false;
            else if (arg == "--tropo") cfg_.enable_tropo = true;
            else if (arg == "--no-tropo") cfg_.enable_tropo = false;
            else if (arg == "--klobuchar") cfg_.enable_klobuchar = true;
            else if (arg == "--no-klobuchar") cfg_.enable_klobuchar = false;
            else if (arg == "--tgd") cfg_.enable_tgd = true;
            else if (arg == "--no-tgd") cfg_.enable_tgd = false;
            else if (arg.rfind("--ref-x=", 0) == 0) cfg_.ref_ecef_x = std::stod(arg.substr(8));
            else if (arg.rfind("--ref-y=", 0) == 0) cfg_.ref_ecef_y = std::stod(arg.substr(8));
            else if (arg.rfind("--ref-z=", 0) == 0) cfg_.ref_ecef_z = std::stod(arg.substr(8));
        }


        if (!cfg_.config_path.empty()) {
            loadFromFile(cfg_.config_path);
            updateConfigTimestamp();
        }
    }

    /**
     * @brief 配置文件热更新：检测文件时间戳变化后重新加载
     * @return 无
     *
     * 运行机制：
     * - 每当解码器解析到一帧完整消息后调用一次；
     * - 若 last_write_time 与上次记录不同，则重新加载并更新缓存时间戳。
     */
    void reloadIfChanged() {
        if (!cfg_.watch_config || cfg_.config_path.empty()) return;
        std::error_code ec;
        auto ts = std::filesystem::last_write_time(cfg_.config_path, ec);
        if (ec) return;
        if (!has_ts_ || ts != last_ts_) {
            loadFromFile(cfg_.config_path);
            last_ts_ = ts;
            has_ts_ = true;
        }
    }

private:
    /**
     * @brief 从配置文件加载 key=value 并写入 cfg_
     * @param path 配置文件路径
     * @return true: 加载成功；false: 文件不可读或打开失败
     */
    bool loadFromFile(const std::string& path) {
        std::ifstream in(path);
        if (!in.is_open()) return false;
        std::string line;
        while (std::getline(in, line)) {
            if (line.empty()) continue;
            if (line[0] == '#') continue;
            auto pos = line.find('=');
            if (pos == std::string::npos) continue;
            std::string key = line.substr(0, pos);
            std::string val = line.substr(pos + 1);
            trim(key);
            trim(val);

            if (key == "input_path") cfg_.input_path = val;
            else if (key == "output_dir") cfg_.output_dir = val;
            else if (key == "terminal_output") cfg_.terminal_output = (val == "1" || val == "true");
            else if (key == "file_output") cfg_.file_output = (val == "1" || val == "true");
            else if (key == "file_spp_solution") cfg_.file_spp_solution = (val == "1" || val == "true");
            else if (key == "file_sat_states") cfg_.file_sat_states = (val == "1" || val == "true");
            else if (key == "file_ephemeris") cfg_.file_ephemeris = (val == "1" || val == "true");
            else if (key == "file_observations") cfg_.file_observations = (val == "1" || val == "true");
            else if (key == "enable_spp") cfg_.enable_spp = (val == "1" || val == "true");
            else if (key == "enable_sat_state") cfg_.enable_sat_state = (val == "1" || val == "true");
            else if (key == "elev_mask_deg") cfg_.elev_mask_deg = std::stod(val);
            else if (key == "snr_mask_dbhz") cfg_.snr_mask_dbhz = std::stod(val);
            else if (key == "enable_iono_free") cfg_.enable_iono_free = (val == "1" || val == "true");
            else if (key == "enable_tropo") cfg_.enable_tropo = (val == "1" || val == "true");
            else if (key == "enable_klobuchar") cfg_.enable_klobuchar = (val == "1" || val == "true");
            else if (key == "enable_tgd") cfg_.enable_tgd = (val == "1" || val == "true");
            else if (key == "ref_ecef_x") cfg_.ref_ecef_x = std::stod(val);
            else if (key == "ref_ecef_y") cfg_.ref_ecef_y = std::stod(val);
            else if (key == "ref_ecef_z") cfg_.ref_ecef_z = std::stod(val);
        }
        return true;
    }

    /**
     * @brief 记录配置文件当前时间戳
     * @return 无
     */
    void updateConfigTimestamp() {
        std::error_code ec;
        last_ts_ = std::filesystem::last_write_time(cfg_.config_path, ec);
        has_ts_ = !ec;
    }

    /**
     * @brief 去除字符串首尾空白字符（空格/制表符/换行等）
     * @param s 输入输出字符串
     * @return 无
     */
    static void trim(std::string& s) {
        auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
        while (!s.empty() && is_space(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
        while (!s.empty() && is_space(static_cast<unsigned char>(s.back()))) s.pop_back();
    }

private:
    RuntimeConfig cfg_;
    std::filesystem::file_time_type last_ts_{};
    bool has_ts_ = false;
};

/**
 * @brief 终端输出模块：负责运行过程中的可读性输出
 *
 * 说明：
 * - 该模块仅做“展示”，不参与任何定位领域逻辑判断；
 * - 输出开关由 RuntimeConfig::terminal_output 控制。
 */
class TerminalOutput {
public:
    /**
     * @brief 设置终端输出是否启用
     * @param enabled true: 输出；false: 静默
     * @return 无
     */
    void setEnabled(bool enabled) { enabled_ = enabled; }

    /**
     * @brief 输出启动信息（输入/输出路径、配置文件等）
     * @param cfg 运行时配置
     * @return 无
     */
    void printStartup(const RuntimeConfig& cfg) {
        if (!enabled_) return;
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "GNSS SPP 定位程序" << std::endl;
        std::cout << "INPUT=" << cfg.input_path << std::endl;
        std::cout << "OUTPUT_DIR=" << cfg.output_dir << std::endl;
        if (!cfg.config_path.empty()) std::cout << "CONFIG=" << cfg.config_path << std::endl;
    }

    /**
     * @brief 输出单历元摘要（用于观察解算是否稳定、卫星匹配是否满足）
     * @param gps_time 当前历元 GPS 时间（用于格式化输出年月日时分秒）
     * @param report SPP 输出的历元报告
     * @param ok 解算结果标志（true: 成功；false: 失败）
     * @return 无
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
                  << " PDOP=" << report.pdop
                  << std::endl;
    }

    /**
     * @brief 输出全流程统计摘要（用于处理量级与异常跳过原因分析）
     * @param total_bytes 读取的总字节数
     * @param range_frames RANGE 历元数量（消息 ID=43）
     * @param gps_eph GPS 星历数量（消息 ID=7）
     * @param bds_eph BDS 星历数量（消息 ID=1696）
     * @param solve_attempt 实际进入 SPP 解算的历元数量
     * @param spp_ok SPP 解算成功历元数量
     * @param spp_fail SPP 解算失败历元数量
     * @param skip_no_eph 由于无星历导致未尝试解算的历元数量
     * @param skip_low_match 由于匹配星历不足导致未尝试解算的历元数量
     * @param sat_total 卫星状态计算尝试次数（按历元×卫星计数）
     * @param sat_no_eph 无星历导致未计算卫星状态次数
     * @param sat_ok 卫星状态计算成功次数（函数返回 true）
     * @param sat_valid 卫星状态数值有效次数（st.valid=true）
     * @return 无
     */
    void printSummary(uint32_t total_bytes,
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
     * @param cfg 运行时配置（引用保存，生命周期由调用方保证）
     */
    explicit FileManager(const RuntimeConfig& cfg) : cfg_(cfg) {}

    /**
     * @brief 打开所有启用的输出文件并写入表头
     * @return true: 成功；false: 目录创建或文件打开失败
     */
    bool openAll() {
        if (!cfg_.file_output) return true;

        std::error_code ec;
        std::filesystem::create_directories(cfg_.output_dir, ec);
        if (ec) return false;

        if (cfg_.file_spp_solution) {
            spp_path_ = (std::filesystem::path(cfg_.output_dir) / "spp_solution.txt").string();
            spp_file_.open(spp_path_);
            if (!spp_file_.is_open()) return false;
            spp_file_.setf(std::ios::fixed, std::ios::floatfield);
            spp_file_ << "Wk SOW ECEF-X/m ECEF-Y/m ECEF-Z/m REF-ECEF-X/m REF-ECEF-Y/m REF-ECEF-Z/m EAST/m NORTH/m UP/m B/deg L/deg H/m VX/m VY/m VZ/m PDOP SigmaP SigmaV GS BS n" << std::endl;
        }

        if (cfg_.file_sat_states) {
            sat_path_ = (std::filesystem::path(cfg_.output_dir) / "sat_states.txt").string();
            sat_file_.open(sat_path_);
            if (!sat_file_.is_open()) return false;
            sat_file_ << "* 通用时 (计算时刻)" << std::endl;
            sat_file_ << "* GPS周，周内秒 (计算时刻)" << std::endl;
            sat_file_ << "卫星号 POS_X(km) POS_Y(km) POS_Z(km) VEL_X(m/s) VEL_Y(m/s) VEL_Z(m/s) CLK(us) CLK_VEL(us/sec)" << std::endl;
            sat_file_ << "注意：卫星号前“B”表示广播星历，卫星位置、钟差等均未经任何改正" << std::endl;
        }

        if (cfg_.file_ephemeris) {
            gps_eph_path_ = (std::filesystem::path(cfg_.output_dir) / "gps_ephem.txt").string();
            bds_eph_path_ = (std::filesystem::path(cfg_.output_dir) / "bds_ephem.txt").string();
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

        if (cfg_.file_observations) {
            obs_path_ = (std::filesystem::path(cfg_.output_dir) / "range_obs.txt").string();
            obs_file_.open(obs_path_);
            if (!obs_file_.is_open()) return false;
            obs_file_.setf(std::ios::fixed, std::ios::floatfield);
            obs_file_ << std::setprecision(3);
            obs_file_ << "WEEK TOW SYS PRN FREQIDX P(m) L(cycle) D(Hz) SNR(dBHz)" << std::endl;
        }

        return true;
    }

    /**
     * @brief 写入单历元 SPP 报告（spp_solution.txt）
     * @param gps_time 当前历元 GPS 时间（用于格式化输出年月日时分秒）
     * @param report SPP 历元报告
     * @return 无
     *
     * 说明：
     * - report.attempted / report.ok 等字段由 SPP 模块负责填充；
     * - 此处仅按表头顺序输出，便于后处理脚本解析。
     */
    void writeSppReport(const gnss::GPSTIME& gps_time, const gnss::SppEpochReport& report) {
        if (!cfg_.file_output || !cfg_.file_spp_solution) return;
        if (!spp_file_.is_open()) return;
        if (!has_ref_ && (cfg_.ref_ecef_x != 0.0 || cfg_.ref_ecef_y != 0.0 || cfg_.ref_ecef_z != 0.0)) {
            ref_xyz_.x = cfg_.ref_ecef_x;
            ref_xyz_.y = cfg_.ref_ecef_y;
            ref_xyz_.z = cfg_.ref_ecef_z;
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
     * @param gps_time 当前历元的 GPS 时间（周+周内秒）
     * @return 无
     */
    void writeSatEpochHeader(const gnss::GPSTIME& gps_time) {
        if (!cfg_.file_output || !cfg_.file_sat_states) return;
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
     * @param sys 卫星系统（GPS/BDS）
     * @param prn 卫星 PRN
     * @param st 卫星状态（位置/速度/钟差/钟速）
     * @return 无
     *
     * 注意：
     * - 输出文件用于验证卫星广播轨道计算口径；
     * - “B”前缀表示仅广播星历状态，不叠加 SPP 观测改正项。
     */
    void writeSatStateLine(gnss::GnssSystem sys, uint32_t prn, const gnss::SatState& st) {
        if (!cfg_.file_output || !cfg_.file_sat_states) return;
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
     * @param e GPS 星历结构体（来自解码器缓存）
     * @return 无
     */
    void writeGpsEphem(const gnss::GpsEphem& e) {
        if (!cfg_.file_output || !cfg_.file_ephemeris) return;
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
     * @param e BDS 星历结构体（来自解码器缓存）
     * @return 无
     */
    void writeBdsEphem(const gnss::BdsEphem& e) {
        if (!cfg_.file_output || !cfg_.file_ephemeris) return;
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
     * @param week 当前 GPS 周
     * @param tow 当前周内秒（GPST）
     * @param obs_list 当前历元观测列表
     * @return 无
     */
    void writeObs(uint16_t week, double tow, const std::vector<gnss::ObsData>& obs_list) {
        if (!cfg_.file_output || !cfg_.file_observations) return;
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
     * @brief 获取 spp_solution.txt 输出路径（若未启用则为空字符串）
     * @return 输出路径引用
     */
    const std::string& sppPath() const { return spp_path_; }
    /**
     * @brief 获取 sat_states.txt 输出路径（若未启用则为空字符串）
     * @return 输出路径引用
     */
    const std::string& satPath() const { return sat_path_; }

private:
    /**
     * @brief 运行时配置引用（由 main() 生命周期托管）
     */
    const RuntimeConfig& cfg_;
    bool has_ref_ = false;
    gnss::XYZ ref_xyz_ = {0.0, 0.0, 0.0};

    std::ofstream spp_file_;
    std::ofstream sat_file_;
    std::ofstream gps_eph_file_;
    std::ofstream bds_eph_file_;
    std::ofstream obs_file_;

    std::string spp_path_;
    std::string sat_path_;
    std::string gps_eph_path_;
    std::string bds_eph_path_;
    std::string obs_path_;
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
     * @param cfg 运行时配置
     */
    explicit SppCore(const RuntimeConfig& cfg) {
        solver_.setElevationMaskDeg(cfg.elev_mask_deg);
        solver_.setSnrMaskDbHz(cfg.snr_mask_dbhz);
        solver_.setEnableIonoFree(cfg.enable_iono_free);
        solver_.setEnableTroposphere(cfg.enable_tropo);
        solver_.setEnableKlobucharIono(cfg.enable_klobuchar);
        solver_.setEnableTgdCorrection(cfg.enable_tgd);
        solver_.setVerbose(false);
    }

    /**
     * @brief 执行单历元 SPP 解算并生成报告
     * @param tow_gpst 历元时刻（GPST 周内秒）
     * @param obs_list 当前历元观测列表
     * @param eph_list 当前可用星历列表
     * @param report 输出的历元报告
     * @return true: 解算成功；false: 解算失败或数据不足
     */
    bool compute(double tow_gpst,
                 const std::vector<gnss::ObsData>& obs_list,
                 const std::vector<gnss::Ephemeris>& eph_list,
                 gnss::SppEpochReport& report) {
        return solver_.computeWithReport(tow_gpst, obs_list, eph_list, report);
    }

    /**
     * @brief 获取最近一次解算位置（ECEF）
     * @return {X,Y,Z} 米
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
     * @param sys 卫星系统
     * @param prn 卫星 PRN
     * @param tow_gpst 计算时刻（GPST 周内秒）
     * @param eph 统一格式星历（由解码器转换得到）
     * @param st 输出卫星状态
     * @return true: 计算成功；false: 不支持系统或计算失败
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
     * @param eph 统一星历
     * @return GPS 星历输入结构体
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
     * @param eph 统一星历
     * @return BDS 星历输入结构体
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

} // namespace

/**
 * @brief 程序入口：驱动数据解码、预处理、SPP 解算与结果输出
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 0: 正常退出；非 0: 异常退出
 *
 * 主流程（按数据流驱动组织）：
 * 1) 读取命令行与配置文件（可选热更新）
 * 2) 初始化终端输出与输出文件
 * 3) 流式读取 OEM 二进制数据，解码消息帧
 * 4) 星历消息：缓存并落盘
 * 5) 观测消息：落盘、可选卫星状态输出、可选 SPP 解算输出
 * 6) 输出最终统计摘要
 */
int main(int argc, char** argv) {
    try {
        ConfigManager cfg_mgr;
        cfg_mgr.loadFromArgs(argc, argv);

        TerminalOutput term;
        term.setEnabled(cfg_mgr.config().terminal_output);
        term.printStartup(cfg_mgr.config());

        // 打开输入文件（OEM 二进制日志）。
        std::ifstream in_file(cfg_mgr.config().input_path, std::ios::binary);
        if (!in_file.is_open()) {
            std::cerr << "[ERROR] 无法打开输入文件: " << cfg_mgr.config().input_path << std::endl;
            return 1;
        }

        // 初始化输出目录与各类落盘文件（由配置控制）。
        FileManager files(cfg_mgr.config());
        if (!files.openAll()) {
            std::cerr << "[ERROR] 输出文件初始化失败: " << cfg_mgr.config().output_dir << std::endl;
            return 1;
        }

        // 初始化解码器与 SPP 解算核心。
        gnss::NovatelDecoder decoder;
        SppCore spp(cfg_mgr.config());

        // 星历缓存：key=(sys,prn) -> Ephemeris。
        // 生命周期：贯穿整个程序运行，随着消息流持续更新（用于后续历元的星历匹配）。
        std::map<uint32_t, gnss::Ephemeris> eph_store;

        // 统计变量：用于终端输出 SUMMARY，与性能无关，生命周期贯穿 main()。
        uint32_t total_bytes = 0;
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

        // 主循环：逐字节喂给解码器，解码出完整消息帧后按消息类型分发处理。
        uint8_t byte = 0;
        while (in_file.read(reinterpret_cast<char*>(&byte), 1)) {
            total_bytes++;
            if (!decoder.inputData(byte)) continue;

            // 配置热更新：仅在 watch_config=true 时生效。
            cfg_mgr.reloadIfChanged();
            term.setEnabled(cfg_mgr.config().terminal_output);

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
            files.writeObs(decoder.getWeek(), tow, obs_list);

            // 卫星状态文件按历元写入两行时间头，便于后处理按历元分段解析。
            gnss::GPSTIME gps_time;
            gps_time.Week = decoder.getWeek();
            gps_time.SecOfWeek = tow;
            files.writeSatEpochHeader(gps_time);

            // 卫星状态输出：对本历元出现的（GPS/BDS、freqIdx=0）卫星，若能匹配到星历则计算状态并写入。
            if (cfg_mgr.config().enable_sat_state && cfg_mgr.config().file_output && cfg_mgr.config().file_sat_states) {
                std::map<uint32_t, gnss::ObsData> uniq_obs;
                for (const auto& obs : obs_list) {
                    if (obs.system != gnss::GnssSystem::GPS && obs.system != gnss::GnssSystem::BDS) continue;
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
            if (!cfg_mgr.config().enable_spp) continue;

            // 将当前缓存的全部星历转为线性列表（SPP 内部自行按 sys/prn 匹配）。
            std::vector<gnss::Ephemeris> eph_list;
            eph_list.reserve(eph_store.size());
            for (const auto& kv : eph_store) eph_list.push_back(kv.second);

            // 单历元 SPP 解算：输出 report（用于终端摘要与落盘）。
            gnss::SppEpochReport report;
            bool ok = spp.compute(tow, obs_list, eph_list, report);
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
        if (cfg_mgr.config().terminal_output && has_first_ok && last_ok_pos.size() == 3) {
            std::cout << "FIRST_OK_TOW=" << first_ok_tow << " LAST_OK_TOW=" << last_ok_tow << std::endl;
            std::cout << "LAST_OK_XYZ=" << last_ok_pos[0] << "," << last_ok_pos[1] << "," << last_ok_pos[2]
                      << " LAST_OK_CLK(m)=" << last_ok_clk_m << std::endl;
        }

        // 终端输出：输出目录提示（便于脚本或人工定位输出文件位置）。
        if (cfg_mgr.config().terminal_output && cfg_mgr.config().file_output) {
            std::cout << "OUTPUT_DIR=" << cfg_mgr.config().output_dir << std::endl;
        }

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 异常退出: " << e.what() << std::endl;
        return 2;
    }
}
