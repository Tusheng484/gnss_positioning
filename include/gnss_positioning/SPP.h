/**
 * @file SPP.h
 * @brief 标准单点定位（SPP）核心接口与数据结构声明
 * @author Jiaoyang Wu
 * @date 2026-03-25
 */

#ifndef SPP_H
#define SPP_H

#include <vector>
#include <map>
#include <string>
#include <Eigen/Dense>
#include "gnss_positioning/NovatelDecoder.h"
#include "gnss_positioning/NavMath.h"

namespace gnss {

/**
 * @brief SPP 计算所需的物理常数与系统常数
 *
 * 统一常量定义用于保证各模块计算口径一致，避免出现单位或数值版本不一致导致的系统性偏差。
 */
struct SppConst {
    static constexpr double C = 299792458.0;               ///< 光速 (m/s)
    static constexpr double OMEGA_E = 7.2921151467e-5;     ///< 地球自转角速度 (rad/s)
    static constexpr double MU_GPS = 3.986005e14;          ///< GPS 地球引力常数 (m^3/s^2)
    static constexpr double MU_BDS = 3.986004418e14;       ///< BDS 地球引力常数 (m^3/s^2)
    static constexpr double F_REL = -4.442807633e-10;      ///< 相对论改正系数 F (s/m^0.5)
    static constexpr double WEEK_SEC = 604800.0;           ///< 一周秒数 (s)
    static constexpr double HALF_WEEK = 302400.0;          ///< 半周秒数 (s)
    static constexpr double BDT_GPS_OFFSET = 14.0;         ///< 时间系统常量：GPST = BDT + 14s（用于将 BDT 统一到 GPST）

    static constexpr double GPS_L1 = 1575.42e6;            ///< GPS L1 载波频率 (Hz)
    static constexpr double GPS_L2 = 1227.60e6;            ///< GPS L2 载波频率 (Hz)
    static constexpr double BDS_B1I = 1561.098e6;          ///< BDS B1I 载波频率 (Hz)
    static constexpr double BDS_B3I = 1268.52e6;           ///< BDS B3I 载波频率 (Hz)
};

/**
 * @brief 统一广播星历结构体（用于卫星位置与钟差计算）
 * @note toe/toc 为周内秒，系统时间基准可能不同：GPS 为 GPST，BDS 为 BDT。
 */
struct Ephemeris {
    GnssSystem system;        ///< 卫星所属系统
    uint32_t prn;             ///< 卫星 PRN 号

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

    double cuc, cus, crc, crs, cic, cis; ///< 广播星历二阶谐波改正项
};

/**
 * @brief SPP 计算输出
 */
struct SppSolution {
    XYZ pos;                  ///< 接收机 ECEF 坐标 (m)
    double clk_gps_m;         ///< GPS 接收机钟差 (m)，即 c*dt_r
    double isb_bds_m;         ///< BDS 相对 GPS 的系统间偏差 (m)

    XYZ vel = {0.0, 0.0, 0.0};///< 接收机 ECEF 速度 (m/s)
    double clk_drift_ms = 0.0;///< 接收机钟差变化率 c*dt_dot_r (m/s)

    double pdop = 999.0;      ///< 位置精度因子（PDOP）
    double hdop = 999.0;      ///< 水平精度因子（HDOP）
    double vdop = 999.0;      ///< 高程精度因子（VDOP）
    double gdop = 999.0;      ///< 几何精度因子（GDOP）
    double tdop = 999.0;      ///< 时间精度因子（TDOP）
    double sigma0 = 0.0;      ///< 单位权中误差 (m)
    int nsat_used = 0;        ///< 参与解算的卫星数
    int nsat_gps = 0;         ///< 参与解算的 GPS 卫星数
    int nsat_bds = 0;         ///< 参与解算的 BDS 卫星数
};

/**
 * @brief 单历元 SPP 运行报告（用于工程诊断与结果落盘）
 * @note 该结构体用于让 main.cpp 保持“应用层”职责：只做数据流驱动与文件输出，不承担 SPP 领域逻辑判断。
 */
struct SppEpochReport {
    double tow_gpst = 0.0;    ///< 历元接收时刻（GPST 周内秒）
    int obs_g = 0;            ///< GPS 观测卫星数量（按主频点统计）
    int obs_b = 0;            ///< BDS 观测卫星数量（按主频点统计）
    int eph_g = 0;            ///< GPS 星历条目数量
    int eph_b = 0;            ///< BDS 星历条目数量
    int match_g = 0;          ///< GPS 观测与星历匹配数量
    int match_b = 0;          ///< BDS 观测与星历匹配数量
    bool attempted = false;   ///< 是否满足基本条件并实际进入解算流程
    bool ok = false;          ///< 是否解算成功
    XYZ pos = {0.0, 0.0, 0.0};///< 解算得到的接收机 ECEF 坐标 (m)
    double clk_gps_m = 0.0;   ///< 解算得到的 GPS 接收机钟差 (m)
    double isb_bds_m = 0.0;   ///< 解算得到的 BDS 相对 GPS 的系统间偏差 (m)

    XYZ vel = {0.0, 0.0, 0.0}; ///< 测速结果（ECEF，m/s）
    double clk_drift_ms = 0.0; ///< 接收机钟漂 c*dt_dot_r (m/s)
    double pdop = 999.0;       ///< 位置精度因子 (PDOP)
    double hdop = 999.0;       ///< 水平精度因子 (HDOP)
    double vdop = 999.0;       ///< 高程精度因子 (VDOP)
    double gdop = 999.0;       ///< 几何精度因子 (GDOP)
    double tdop = 999.0;       ///< 时间精度因子 (TDOP)
    double sigma0 = 0.0;       ///< 单位权中误差 (m)
    int nsat_used = 0;         ///< 参与解算卫星数
    int gs_used = 0;           ///< 参与解算的 GPS 卫星数
    int bs_used = 0;           ///< 参与解算的 BDS 卫星数
    std::string sat_used;      ///< 参与解算的卫星编号列表（如：G05 C12），多颗卫星用空格分隔
    int rejected = 0;          ///< 粗差剔除卫星数
    bool vel_ok = false;       ///< 测速是否成功
};

/**
 * @brief GPS 广播星历结构体转换为 SPP 统一星历结构体
 * @param gps_eph 输入的 GPS 广播星历
 * @return 统一格式的星历结构体
 */
Ephemeris ConvertFromGpsEphem(const GpsEphem& gps_eph);

/**
 * @brief BDS 广播星历结构体转换为 SPP 统一星历结构体
 * @param bds_eph 输入的 BDS 广播星历
 * @return 统一格式的星历结构体
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
    /**
     * @brief 构造 SPP 解算器并初始化默认参数
     * @return 无
     */
    SPP();

    /**
     * @brief 执行单历元 SPP 解算
     * @param t_rx_gpst 接收时刻（GPST 周内秒）
     * @param obsList 观测数据列表（同一历元）
     * @param ephList 广播星历列表（可包含多系统/多卫星）
     * @return true: 解算成功；false: 解算失败或数据不足
     */
    bool compute(double t_rx_gpst, const std::vector<ObsData>& obsList, const std::vector<Ephemeris>& ephList);
    /**
     * @brief 执行单历元定位并输出报告（用于落盘与工程诊断）
     * @note 与 compute() 相同的解算逻辑，但会额外输出观测/星历/匹配/是否尝试等关键信息，避免 main.cpp 承担定位领域逻辑。
     * @param t_rx_gpst 接收时刻（GPST 周内秒）
     * @param obsList 观测数据列表（同一历元）
     * @param ephList 广播星历列表（可包含多系统/多卫星）
     * @param report 输出的历元报告
     * @return true: 解算成功；false: 解算失败或数据不足
     */
    bool computeWithReport(double t_rx_gpst, const std::vector<ObsData>& obsList, const std::vector<Ephemeris>& ephList, SppEpochReport& report);

    /**
     * @brief 获取当前解算的接收机坐标（ECEF）
     * @return {X, Y, Z}，单位：米 (m)
     */
    std::vector<double> getPosition() const;
    /**
     * @brief 获取当前解算的接收机钟差（GPS，距离量纲）
     * @return 接收机钟差 c*dt_r，单位：米 (m)
     */
    double getReceiverClockBias() const;
    /**
     * @brief 获取当前解算的系统间偏差（BDS 相对 GPS）
     * @return 系统间偏差，单位：米 (m)
     */
    double getBdsIsbMeters() const { return sol_.isb_bds_m; }

    /**
     * @brief 设置运行时输出开关
     * @param enable true: 输出调试信息；false: 静默运行
     * @return 无
     */
    void setVerbose(bool enable) { verbose_ = enable; }
    /**
     * @brief 设置高度角掩码
     * @param deg 掩码角度，单位：度 (deg)
     * @return 无
     */
    void setElevationMaskDeg(double deg) { elev_mask_deg_ = deg; }
    /**
     * @brief 设置 SNR 掩码
     * @param snr 最小 SNR，单位：dB-Hz
     * @return 无
     */
    void setSnrMaskDbHz(double snr) { snr_mask_dbhz_ = snr; }
    /**
     * @brief 设置是否启用无电离层组合伪距
     * @param enable true: 使用双频无电离层组合（可用时）；false: 使用单频伪距
     * @return 无
     */
    void setEnableIonoFree(bool enable) { enable_iono_free_ = enable; }
    void setEnableTroposphere(bool enable) { enable_tropo_ = enable; }
    void setEnableKlobucharIono(bool enable) { enable_klobuchar_ = enable; }
    void setEnableTgdCorrection(bool enable) { enable_tgd_ = enable; }
    void setGpsKlobucharParams(const double alpha[4], const double beta[4]);
    void setBdsKlobucharParams(const double alpha[4], const double beta[4]);

private:
    /**
     * @brief 解算前的观测预处理结果（单星单观测）
     */
    struct ProcObs {
        GnssSystem system;    ///< 观测所属系统
        uint32_t prn;         ///< 卫星 PRN 号
        double P;             ///< 参与解算的伪距 (m)（可为无电离层组合）
        double snr;           ///< 观测信噪比（用于质量控制）
        bool iono_free;       ///< 是否为无电离层组合观测

        double doppler = 0.0;    ///< 多普勒观测值 (Hz)，正值表示卫星接近
        double wavelength = 0.0; ///< 对应载波波长 (m)
        bool has_doppler = false;///< 是否存在有效多普勒观测

        double gf_m = 0.0;        ///< GF 组合：P1 - P2 (m)
        double dgf_m = 0.0;       ///< 历元间 GF 差分：GF(t) - GF(t-1) (m)
        bool gf_outlier = false;  ///< GF 粗差标记
    };

    /**
     * @brief 单颗卫星的中间计算结果（用于构建观测方程与质量控制）
     */
    struct SatCalc {
        GnssSystem system;    ///< 卫星所属系统
        uint32_t prn;         ///< 卫星 PRN 号
        XYZ pos_ecef;         ///< 卫星 ECEF 坐标 (m)
        double clk_s;         ///< 卫星钟差 (s)
        double elev_rad;      ///< 高度角 (rad)
        XYZ los;              ///< 单位视线向量
        double rho;           ///< 几何距离 (m)
        double tropo_m;        ///< 对流层延迟 (m)
        double P_corr;         ///< 观测伪距 + c*dt_s (m)
        bool valid;           ///< 中间量是否有效

        XYZ vel_ecef = {0.0, 0.0, 0.0}; ///< 卫星 ECEF 速度 (m/s)
        double clk_drift_s = 0.0;       ///< 卫星钟速 (s/s)
        double doppler_ms = 0.0;        ///< 多普勒换算的距离变化率 (m/s)
        double az_rad = 0.0;            ///< 方位角 (rad)
        bool has_doppler = false;       ///< 是否有有效多普勒观测
    };

    /**
     * @brief 将时间差归一化到半周范围内
     * @param dt 输入时间差，单位：秒 (s)
     * @return 归一化后的时间差，单位：秒 (s)
     */
    static double normalizeTimeSec(double dt);
    /**
     * @brief 生成系统与 PRN 的哈希键（用于容器索引）
     * @param sys 卫星系统
     * @param prn 卫星 PRN 号
     * @return 组合后的键值
     */
    static uint32_t makeKey(GnssSystem sys, uint32_t prn);

    /**
     * @brief 将星历时间系统统一到 GPST
     * @param eph 输入星历
     * @return 时间系统已统一后的星历
     */
    Ephemeris normalizeEphemerisToGpSt(const Ephemeris& eph) const;

    /**
     * @brief 观测预处理与筛选（单星最佳观测选取与质量控制）
     * @param obsList 输入观测列表（同一历元）
     * @return 预处理后的观测列表
     */
    std::vector<ProcObs> preprocessObservations(const std::vector<ObsData>& obsList) const;

    /**
     * @brief 观测值一致性检验（双频伪距基本有效性检查）
     * @param P1 频点 1 伪距 (m)
     * @param P2 频点 2 伪距 (m)
     * @return true: 观测有效；false: 观测无效
     */
    bool CheckOBSConsist(double P1, double P2) const;

    /**
     * @brief 基于历元间差分的质量控制（GF组合探测与多普勒一致性检验）
     * @param sys 卫星系统
     * @param prn 卫星 PRN
     * @param P_m 当前历元伪距 (m)
     * @param doppler_ms 当前历元多普勒测速值 (m/s)
     * @param has_doppler 当前历元多普勒是否有效
     * @param gf_m 当前历元 GF 值 (m)
     * @param dgf_m 输出历元间差分 dGF (m)
     * @return true: 检测为粗差；false: 未检测为粗差
     */
    bool QualityControlEpoch(GnssSystem sys, uint32_t prn, double P_m, double doppler_ms, bool has_doppler, double gf_m, double& dgf_m) const;

    /**
     * @brief 由接收时刻与伪距计算信号发射时刻近似值
     * @param t_rx_gpst 接收时刻（GPST 周内秒）
     * @param pseudorange_m 伪距观测值，单位：米 (m)
     * @return 发射时刻（GPST 周内秒）
     */
    double computeTransmitTime(double t_rx_gpst, double pseudorange_m) const;
    /**
     * @brief 迭代计算信号传播时间并输出卫星状态（含地球自转改正）
     * @param t_rx_gpst 接收时刻（GPST 周内秒）
     * @param pseudorange_m 伪距观测值，单位：米 (m)
     * @param eph 广播星历（时间系统保持原始口径：GPS 为 GPST，BDS 为 BDT）
     * @param rx_pos_approx 接收机坐标近似值（ECEF）
     * @param proc 预处理观测（包含多普勒等信息）
     * @param satCalc 输出的卫星中间量
     * @return true: 计算成功；false: 数值异常或星历不可用
     */
    bool computeSatState(double t_rx_gpst,
                         double pseudorange_m,
                         const Ephemeris& eph,
                         const XYZ& rx_pos_approx,
                         const ProcObs& proc,
                         SatCalc& satCalc) const;
    /**
     * @brief 构建最小二乘解算所需的观测方程中间量
     * @param t_rx_gpst 接收时刻（GPST 周内秒）
     * @param procObs 预处理后的观测列表
     * @param ephList 星历列表
     * @param rx_pos 接收机坐标初值/当前值（ECEF）
     * @param clk_gps_m 接收机钟差（GPS，距离量纲）
     * @param isb_bds_m 系统间偏差（BDS 相对 GPS，距离量纲）
     * @param satCalcs 输出的卫星中间量列表
     * @return true: 成功构建且至少存在一颗有效卫星；false: 构建失败或无可用卫星
     */
    bool buildObservationEquation(double t_rx_gpst,
                                 const std::vector<ProcObs>& procObs,
                                 const std::vector<Ephemeris>& ephList,
                                 const XYZ& rx_pos,
                                 double clk_gps_m,
                                 double isb_bds_m,
                                 std::vector<SatCalc>& satCalcs) const;
    /**
     * @brief 最小二乘法求解状态增量
     * @param sats 有效卫星中间量列表
     * @param dx 输出的状态增量向量
     * @param v 输出的后验残差向量（未加权）
     * @param H 输出的设计矩阵（未加权）
     * @return true: 求解成功；false: 求解失败或结果非有限数
     */
    bool solveLeastSquares(const std::vector<SatCalc>& sats,
                           Eigen::VectorXd& dx,
                           Eigen::VectorXd& v,
                           Eigen::MatrixXd& H) const;

    /**
     * @brief 基于多普勒观测进行单点测速
     * @param sats 有效卫星中间量列表
     * @param rx_pos 接收机坐标（ECEF）
     * @param vel_dx 输出速度解向量 [Vx, Vy, Vz, c*dt_dot_r]^T
     * @return true: 求解成功；false: 可用卫星不足（要求参与解算的卫星数 >= 4 颗）或数值异常
     */
    bool solveVelocity(const std::vector<SatCalc>& sats, const XYZ& rx_pos, Eigen::VectorXd& vel_dx) const;

    /**
     * @brief 计算各类精度因子（DOPs）
     * 依据协因数阵 Q_{xx} = (H^T H)^{-1} 计算 GDOP, PDOP, HDOP, VDOP, TDOP。
     * 包含空间直角坐标系 (XYZ) 到站心地平坐标系 (ENU) 的协方差转换矩阵运算。
     * @param H 定位解算收敛后的设计矩阵（未加权）
     * @param rx_pos 接收机 ECEF 坐标（用于 XYZ 到 ENU 转换）
     * @return 无，结果存入 sol_
     */
    void computeDOPs(const Eigen::MatrixXd& H, const XYZ& rx_pos);



private:
    static constexpr double kDefaultElevMaskDeg = 10.0;
    static constexpr double kDefaultSnrMaskDbHz = 30.0;
    static constexpr double kGfDgfThresholdM = 2.0;
    static constexpr double kGfMaxGapSec = 10.0;
    static constexpr double kRaimPseudorangeSigmaM = 8.0; ///< 伪距残差阈值设为 8 m
    static constexpr double kRaimPhaseSigmaM = 0.5;       ///< 相位残差阈值设为 0.5 m
    static constexpr double kRaimSigmaFactor = 1.0;
    static constexpr int kRaimMaxIter = 3;
    static constexpr double kSigmaZenithM = 0.3; ///< 天顶方向伪距噪声 (m)

    SppSolution sol_;
    int last_rejected_ = 0;
    bool last_vel_ok_ = false;
    mutable double current_tow_gpst_ = 0.0;
    struct PrevObsData {
        double tow_gpst = 0.0;
        double P_m = 0.0;
        double gf_m = 0.0;
    };
    mutable std::map<uint32_t, PrevObsData> prev_obs_;
    int max_iter_;
    double conv_m_;
    double elev_mask_deg_;
    double snr_mask_dbhz_;
    bool enable_iono_free_;
    bool enable_tropo_ = true;
    bool enable_klobuchar_ = false;
    bool enable_tgd_ = true;
    bool has_gps_iono_ = false;
    bool has_bds_iono_ = false;
    double gps_alpha_[4] = {0.0, 0.0, 0.0, 0.0};
    double gps_beta_[4] = {0.0, 0.0, 0.0, 0.0};
    double bds_alpha_[4] = {0.0, 0.0, 0.0, 0.0};
    double bds_beta_[4] = {0.0, 0.0, 0.0, 0.0};
    std::string last_sat_used_;
    bool verbose_;
};

} // namespace gnss

#endif // SPP_H
