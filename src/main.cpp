/**
 * @file main.cpp
 * @brief GNSS 数据流驱动的实时 SPP 单点定位程序（模块化架构）
 * @author Jiaoyang Wu
 * @date 2026-03-30
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
/**
 * @brief 程序入口：驱动数据解码、预处理、SPP 解算与结果输出
 * @return 0: 正常退出；非 0: 异常退出
 */
namespace {

uint32_t MakeKey(gnss::GnssSystem sys, uint32_t prn) {
    return static_cast<uint32_t>(sys) * 1000U + prn;
}

double GpstToBdtSec(double t_gpst) {
    double out = t_gpst - gnss::SppConst::BDT_GPS_OFFSET;
    while (out >= gnss::SppConst::WEEK_SEC) out -= gnss::SppConst::WEEK_SEC;
    while (out < 0.0) out += gnss::SppConst::WEEK_SEC;
    return out;
}

struct RuntimeConfig {
    std::string input_path = "D:/gnss_positioning/Novatel OEM7 Data/01/NovatelOEM20211114-01.log";
    std::string output_dir = "D:/gnss_positioning/output";
    std::string config_path;
    bool watch_config = false;

    bool terminal_output = false;

    bool file_output = true;
    bool file_spp_solution = true;
    bool file_sat_states = true;
    bool file_ephemeris = true;
    bool file_observations = true;

    bool enable_spp = true;
    bool enable_sat_state = true;

    double elev_mask_deg = 10.0;
    double snr_mask_dbhz = 30.0;
    bool enable_iono_free = true;

    void enableFileOutput() { file_output = true; }
    void disableFileOutput() { file_output = false; }
    void enableTerminalOutput() { terminal_output = true; }
    void disableTerminalOutput() { terminal_output = false; }
    void enableSpp() { enable_spp = true; }
    void disableSpp() { enable_spp = false; }
    void enableSatStateOutput() { file_sat_states = true; }
    void disableSatStateOutput() { file_sat_states = false; }
};

class ConfigManager {
public:
    const RuntimeConfig& config() const { return cfg_; }
    RuntimeConfig& config() { return cfg_; }

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
        }

        if (!cfg_.config_path.empty()) {
            loadFromFile(cfg_.config_path);
            updateConfigTimestamp();
        }
    }

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
        }
        return true;
    }

    void updateConfigTimestamp() {
        std::error_code ec;
        last_ts_ = std::filesystem::last_write_time(cfg_.config_path, ec);
        has_ts_ = !ec;
    }

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

class TerminalOutput {
public:
    void setEnabled(bool enabled) { enabled_ = enabled; }

    void printStartup(const RuntimeConfig& cfg) {
        if (!enabled_) return;
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "GNSS SPP 定位程序" << std::endl;
        std::cout << "INPUT=" << cfg.input_path << std::endl;
        std::cout << "OUTPUT_DIR=" << cfg.output_dir << std::endl;
        if (!cfg.config_path.empty()) std::cout << "CONFIG=" << cfg.config_path << std::endl;
    }

    void printEpoch(const gnss::SppEpochReport& report, bool ok) {
        if (!enabled_) return;
        if (!report.attempted) return;
        std::cout << "TOW=" << std::fixed << std::setprecision(3) << report.tow_gpst
                  << " OK=" << (ok ? 1 : 0)
                  << " OBS(G/B)=" << report.obs_g << "/" << report.obs_b
                  << " MATCH(G/B)=" << report.match_g << "/" << report.match_b
                  << " CLK(m)=" << report.clk_gps_m
                  << " ISB(m)=" << report.isb_bds_m
                  << " PDOP=" << report.pdop
                  << std::endl;
    }

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

class FileManager {
public:
    explicit FileManager(const RuntimeConfig& cfg) : cfg_(cfg) {}

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
            spp_file_ << std::setprecision(3);
            spp_file_ << "TOW OBS_G OBS_B EPH_G EPH_B MATCH_G MATCH_B ATTEMPT OK X(m) Y(m) Z(m) CLK_GPS(m) ISB_BDS(m) PDOP VEL_X(m/s) VEL_Y(m/s) VEL_Z(m/s) CLK_DRIFT_MS(ms)" << std::endl;
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
            obs_file_ << "WEEK TOW SYS PRN FREQIDX P(m) L(cycle) D(Hz) SNR(dBHz) STATUS" << std::endl;
        }

        return true;
    }

    void writeSppReport(const gnss::SppEpochReport& report) {
        if (!cfg_.file_output || !cfg_.file_spp_solution) return;
        if (!spp_file_.is_open()) return;
        spp_file_ << report.tow_gpst << " "
                  << report.obs_g << " " << report.obs_b << " "
                  << report.eph_g << " " << report.eph_b << " "
                  << report.match_g << " " << report.match_b << " "
                  << (report.attempted ? 1 : 0) << " "
                  << (report.ok ? 1 : 0) << " "
                  << report.pos.x << " " << report.pos.y << " " << report.pos.z << " "
                  << report.clk_gps_m << " " << report.isb_bds_m << " " << report.pdop << " "
                  << report.vel.x << " " << report.vel.y << " "<< report.vel.z<<" "
                  << report.clk_drift_ms<<" "
                  << std::endl;
    }

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

    void writeObs(uint16_t week, double tow, const std::vector<gnss::ObsData>& obs_list) {
        if (!cfg_.file_output || !cfg_.file_observations) return;
        if (!obs_file_.is_open()) return;
        for (const auto& o : obs_list) {
            char sys_char = 'U';
            if (o.system == gnss::GnssSystem::GPS) sys_char = 'G';
            else if (o.system == gnss::GnssSystem::BDS) sys_char = 'C';
            obs_file_ << week << " " << tow << " " << sys_char << " " << o.prn << " " << o.freqIdx << " "
                      << o.pseudorange << " " << o.carrierPhase << " "
                      << static_cast<double>(o.doppler) << " " << static_cast<double>(o.snr) << " "
                      << o.status
                      << std::endl;
        }
    }

    const std::string& sppPath() const { return spp_path_; }
    const std::string& satPath() const { return sat_path_; }

private:
    const RuntimeConfig& cfg_;

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

class SppCore {
public:
    explicit SppCore(const RuntimeConfig& cfg) {
        solver_.setElevationMaskDeg(cfg.elev_mask_deg);
        solver_.setSnrMaskDbHz(cfg.snr_mask_dbhz);
        solver_.setEnableIonoFree(cfg.enable_iono_free);
        solver_.setVerbose(false);
    }

    bool compute(double tow_gpst,
                 const std::vector<gnss::ObsData>& obs_list,
                 const std::vector<gnss::Ephemeris>& eph_list,
                 gnss::SppEpochReport& report) {
        return solver_.computeWithReport(tow_gpst, obs_list, eph_list, report);
    }

    std::vector<double> lastPosition() const { return solver_.getPosition(); }

private:
    gnss::SPP solver_;
};

class SatStateCore {
public:
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

int main(int argc, char** argv) {
    try {
        ConfigManager cfg_mgr;
        cfg_mgr.loadFromArgs(argc, argv);

        TerminalOutput term;
        term.setEnabled(cfg_mgr.config().terminal_output);
        term.printStartup(cfg_mgr.config());

        std::ifstream in_file(cfg_mgr.config().input_path, std::ios::binary);
        if (!in_file.is_open()) {
            std::cerr << "[ERROR] 无法打开输入文件: " << cfg_mgr.config().input_path << std::endl;
            return 1;
        }

        FileManager files(cfg_mgr.config());
        if (!files.openAll()) {
            std::cerr << "[ERROR] 输出文件初始化失败: " << cfg_mgr.config().output_dir << std::endl;
            return 1;
        }

        gnss::NovatelDecoder decoder;
        SppCore spp(cfg_mgr.config());

        std::map<uint32_t, gnss::Ephemeris> eph_store;

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

        uint8_t byte = 0;
        while (in_file.read(reinterpret_cast<char*>(&byte), 1)) {
            total_bytes++;
            if (!decoder.inputData(byte)) continue;

            cfg_mgr.reloadIfChanged();
            term.setEnabled(cfg_mgr.config().terminal_output);

            uint16_t msg_id = decoder.getMsgId();

            if (msg_id == 7) {
                const auto& gps_eph = decoder.getGpsEphem();
                if (gps_eph.prn >= 1 && gps_eph.prn <= 32) {
                    eph_store[MakeKey(gnss::GnssSystem::GPS, gps_eph.prn)] = gnss::ConvertFromGpsEphem(gps_eph);
                    gps_eph_count++;
                    files.writeGpsEphem(gps_eph);
                }
                continue;
            }

            if (msg_id == 1696) {
                const auto& bds_eph = decoder.getBdsEphem();
                if (bds_eph.prn >= 1 && bds_eph.prn <= 63) {
                    eph_store[MakeKey(gnss::GnssSystem::BDS, bds_eph.prn)] = gnss::ConvertFromBdsEphem(bds_eph);
                    bds_eph_count++;
                    files.writeBdsEphem(bds_eph);
                }
                continue;
            }

            if (msg_id != 43) continue;

            range_frames++;
            double tow = decoder.getTow();
            const auto& obs_list = decoder.getObsData();
            files.writeObs(decoder.getWeek(), tow, obs_list);

            gnss::GPSTIME gps_time;
            gps_time.Week = decoder.getWeek();
            gps_time.SecOfWeek = tow;
            files.writeSatEpochHeader(gps_time);

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

            if (!cfg_mgr.config().enable_spp) continue;

            std::vector<gnss::Ephemeris> eph_list;
            eph_list.reserve(eph_store.size());
            for (const auto& kv : eph_store) eph_list.push_back(kv.second);

            gnss::SppEpochReport report;
            bool ok = spp.compute(tow, obs_list, eph_list, report);
            term.printEpoch(report, ok);

            if (report.attempted) solve_attempt++;
            if (ok) spp_ok++;
            else spp_fail++;
            if (!report.attempted && (report.eph_g + report.eph_b) == 0) skip_no_eph++;
            if (!report.attempted && (report.eph_g + report.eph_b) > 0) skip_low_match++;

            files.writeSppReport(report);

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

        if (cfg_mgr.config().terminal_output && has_first_ok && last_ok_pos.size() == 3) {
            std::cout << "FIRST_OK_TOW=" << first_ok_tow << " LAST_OK_TOW=" << last_ok_tow << std::endl;
            std::cout << "LAST_OK_XYZ=" << last_ok_pos[0] << "," << last_ok_pos[1] << "," << last_ok_pos[2]
                      << " LAST_OK_CLK(m)=" << last_ok_clk_m << std::endl;
        }

        if (cfg_mgr.config().terminal_output && cfg_mgr.config().file_output) {
            std::cout << "OUTPUT_DIR=" << cfg_mgr.config().output_dir << std::endl;
        }

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 异常退出: " << e.what() << std::endl;
        return 2;
    }
}
