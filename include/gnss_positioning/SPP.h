#ifndef SPP_H
#define SPP_H

#include <vector>
#include <Eigen/Dense>
#include "gnss_positioning/NovatelDecoder.h"
#include "gnss_positioning/NavMath.h"

namespace gnss {

/**
 * @brief 物理常数与系统常数（工程级统一定义）
 */
struct SppConst {
    static constexpr double C = 299792458.0;               ///< 光速 (m/s)
    static constexpr double OMEGA_E = 7.2921151467e-5;     ///< 地球自转角速度 (rad/s)
    static constexpr double MU_GPS = 3.986005e14;          ///< GPS 引力常数 (m^3/s^2)
    static constexpr double MU_BDS = 3.986004418e14;       ///< BDS 引力常数 (m^3/s^2)
    static constexpr double F_REL = -4.442807633e-10;      ///< 相对论改正系数 (s/m^0.5)
    static constexpr double WEEK_SEC = 604800.0;           ///< 周秒
    static constexpr double HALF_WEEK = 302400.0;          ///< 半周秒
    static constexpr double BDT_GPS_OFFSET = 14.0;         ///< GPST = BDT + 14s (当前有效，课程数据通常采用该常量)

    static constexpr double GPS_L1 = 1575.42e6;
    static constexpr double GPS_L2 = 1227.60e6;
    static constexpr double BDS_B1I = 1561.098e6;
    static constexpr double BDS_B3I = 1268.52e6;
};

/**
 * @brief 统一广播星历结构体（用于卫星位置与钟差计算）
 * @note toe/toc 为周内秒，系统时间基准可能不同：GPS 为 GPST，BDS 为 BDT。
 */
struct Ephemeris {
    GnssSystem system;
    uint32_t prn;

    double toe;               ///< 星历参考时刻 (s)
    double toc;               ///< 钟差参考时刻 (s)

    double af0, af1, af2;     ///< 卫星钟差参数 (s, s/s, s/s^2)
    double tgd;               ///< 群延迟 (s)，GPS: TGD，BDS: TGD1

    double rootA;             ///< sqrt(a) (m^0.5)
    double ecc;               ///< e
    double inc0;              ///< i0 (rad)
    double inc_dot;           ///< IDOT (rad/s)
    double omega0;            ///< Ω0 (rad)
    double omega_dot;         ///< Ωdot (rad/s)
    double argper;            ///< ω (rad)
    double m0;                ///< M0 (rad)
    double dN;                ///< Δn (rad/s)

    double cuc, cus, crc, crs, cic, cis;
};

/**
 * @brief SPP 计算输出
 */
struct SppSolution {
    XYZ pos;                  ///< 接收机 ECEF 坐标 (m)
    double clk_gps_m;         ///< GPS 接收机钟差 (m)，即 c*dt_r
    double isb_bds_m;         ///< BDS 相对 GPS 的系统间偏差 (m)
};

/**
 * @brief 单历元 SPP 运行报告（用于工程诊断与结果落盘）
 * @note 该结构体用于让 main.cpp 保持“应用层”职责：只做数据流驱动与文件输出，不承担 SPP 领域逻辑判断。
 */
struct SppEpochReport {
    double tow_gpst = 0.0;
    int obs_g = 0;
    int obs_b = 0;
    int eph_g = 0;
    int eph_b = 0;
    int match_g = 0;
    int match_b = 0;
    bool attempted = false;
    bool ok = false;
    XYZ pos = {0.0, 0.0, 0.0};
    double clk_gps_m = 0.0;
    double isb_bds_m = 0.0;
};

/**
 * @brief GPS 广播星历结构体转换为 SPP 统一星历结构体
 */
Ephemeris ConvertFromGpsEphem(const GpsEphem& gps_eph);

/**
 * @brief BDS 广播星历结构体转换为 SPP 统一星历结构体
 */
Ephemeris ConvertFromBdsEphem(const BdsEphem& bds_eph);

/**
 * @brief 标准单点定位（SPP）模块
 * 按教材“五步法”组织：
 * 1) 观测预处理
 * 2) 信号发射时刻
 * 3) 广播星历：卫星位置 + 卫星钟差
 * 4) 构建伪距方程
 * 5) 最小二乘迭代
 */
class SPP {
public:
    SPP();

    bool compute(double t_rx_gpst, const std::vector<ObsData>& obsList, const std::vector<Ephemeris>& ephList);
    /**
     * @brief 执行单历元定位并输出报告（用于落盘与工程诊断）
     * @note 与 compute() 相同的解算逻辑，但会额外输出观测/星历/匹配/是否尝试等关键信息，避免 main.cpp 承担定位领域逻辑。
     */
    bool computeWithReport(double t_rx_gpst, const std::vector<ObsData>& obsList, const std::vector<Ephemeris>& ephList, SppEpochReport& report);

    std::vector<double> getPosition() const;
    double getReceiverClockBias() const;
    double getBdsIsbMeters() const { return sol_.isb_bds_m; }

    void setVerbose(bool enable) { verbose_ = enable; }
    void setElevationMaskDeg(double deg) { elev_mask_deg_ = deg; }
    void setSnrMaskDbHz(double snr) { snr_mask_dbhz_ = snr; }
    void setEnableIonoFree(bool enable) { enable_iono_free_ = enable; }

private:
    struct ProcObs {
        GnssSystem system;
        uint32_t prn;
        double P;             ///< 参与解算的伪距 (m)（可为无电离层组合）
        double snr;
        bool iono_free;
    };

    struct SatCalc {
        GnssSystem system;
        uint32_t prn;
        XYZ pos_ecef;
        double clk_s;         ///< 卫星钟差 (s)
        double elev_rad;
        XYZ los;              ///< 单位视线向量
        double rho;           ///< 几何距离 (m)
        double tropo_m;        ///< 对流层延迟 (m)
        double P_corr;         ///< 观测伪距 + c*dt_s (m)
        bool valid;
    };

    static double normalizeTimeSec(double dt);
    static uint32_t makeKey(GnssSystem sys, uint32_t prn);

    Ephemeris normalizeEphemerisToGpSt(const Ephemeris& eph) const;

    std::vector<ProcObs> preprocessObservations(const std::vector<ObsData>& obsList) const;
    double computeTransmitTime(double t_rx_gpst, double pseudorange_m) const;
    bool computeSatelliteClock(const Ephemeris& eph_gpst, double t_tx_gpst, double& clk_s) const;
    bool computeSatellitePosition(const Ephemeris& eph_gpst, double t_tx_gpst, XYZ& sat_pos_ecef, double& Ek_out) const;
    void applyEarthRotationCorrection(XYZ& sat_pos_ecef, double transit_time_s) const;
    bool buildObservationEquation(double t_rx_gpst,
                                 const std::vector<ProcObs>& procObs,
                                 const std::vector<Ephemeris>& ephList,
                                 const XYZ& rx_pos,
                                 double clk_gps_m,
                                 double isb_bds_m,
                                 std::vector<SatCalc>& satCalcs) const;
    double computeTroposphereSaastamoinen(const BLH& rx_blh, double elev_rad) const;
    bool solveLeastSquares(const std::vector<SatCalc>& sats,
                           Eigen::VectorXd& dx,
                           Eigen::VectorXd& v,
                           Eigen::MatrixXd& H) const;

private:
    SppSolution sol_;
    int max_iter_;
    double conv_m_;
    double elev_mask_deg_;
    double snr_mask_dbhz_;
    bool enable_iono_free_;
    bool verbose_;
};

} // namespace gnss

#endif // SPP_H
