/**
 * @file SatPosCalc.h
 * @brief 广播星历卫星位置、速度与钟差/钟速计算模块接口
 * @author Jiaoyang Wu
 * @date 2026-03-27
 *
 * 本模块提供基于广播星历的卫星状态计算能力，输出卫星在 ECEF 坐标系下的位置、速度以及卫星钟差与钟速。
 */

#ifndef SAT_POS_CALC_H
#define SAT_POS_CALC_H

#include <cstdint>

namespace gnss {

/**
 * @brief 三维向量（用于 ECEF 位置与速度表达）
 */
struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/**
 * @brief GPS 广播星历（WGS84）
 *
 * 字段命名与含义与广播星历常用记号一致，可由工程内统一星历结构体直接映射得到。
 */
struct GpsEph {
    uint32_t prn = 0;

    double toc = 0.0;      ///< 钟差参考时刻（周内秒，GPST），单位：秒 (s)
    double toe = 0.0;      ///< 星历参考时刻（周内秒，GPST），单位：秒 (s)

    double af0 = 0.0;      ///< 卫星钟差参数 af0，单位：秒 (s)
    double af1 = 0.0;      ///< 卫星钟差参数 af1，单位：s/s
    double af2 = 0.0;      ///< 卫星钟差参数 af2，单位：s/s^2
    double tgd = 0.0;      ///< 群延迟（TGD），单位：秒 (s)

    double sqrtA = 0.0;    ///< 轨道长半轴平方根 sqrt(A)，单位：m^0.5
    double e = 0.0;        ///< 轨道离心率
    double i0 = 0.0;       ///< 参考时刻轨道倾角 i0，单位：rad
    double iDot = 0.0;     ///< 轨道倾角变化率 IDOT，单位：rad/s

    double Omega0 = 0.0;   ///< 参考时刻升交点赤经 Ω0，单位：rad
    double OmegaDot = 0.0; ///< 升交点赤经变化率 Ωdot，单位：rad/s
    double omega = 0.0;    ///< 近地点角 ω，单位：rad

    double M0 = 0.0;       ///< 参考时刻平近点角 M0，单位：rad
    double deltaN = 0.0;   ///< 平均运动改正 Δn，单位：rad/s

    double cuc = 0.0;
    double cus = 0.0;
    double crc = 0.0;
    double crs = 0.0;
    double cic = 0.0;
    double cis = 0.0;
};

/**
 * @brief BDS 广播星历（CGCS2000）
 *
 * toe/toc 采用 BDT 周内秒；若上层统一使用 GPST，应在调用前完成时间系统转换。
 */
struct BdsEph {
    uint32_t prn = 0;

    double toc = 0.0;      ///< 钟差参考时刻（周内秒，BDT），单位：秒 (s)
    double toe = 0.0;      ///< 星历参考时刻（周内秒，BDT），单位：秒 (s)

    double af0 = 0.0;      ///< 卫星钟差参数 af0，单位：秒 (s)
    double af1 = 0.0;      ///< 卫星钟差参数 af1，单位：s/s
    double af2 = 0.0;      ///< 卫星钟差参数 af2，单位：s/s^2
    double tgd = 0.0;      ///< 群延迟（TGD/TGD1），单位：秒 (s)

    double sqrtA = 0.0;    ///< 轨道长半轴平方根 sqrt(A)，单位：m^0.5
    double e = 0.0;        ///< 轨道离心率
    double i0 = 0.0;       ///< 参考时刻轨道倾角 i0，单位：rad
    double iDot = 0.0;     ///< 轨道倾角变化率 IDOT，单位：rad/s

    double Omega0 = 0.0;   ///< 参考时刻升交点赤经 Ω0，单位：rad
    double OmegaDot = 0.0; ///< 升交点赤经变化率 Ωdot，单位：rad/s
    double omega = 0.0;    ///< 近地点角 ω，单位：rad

    double M0 = 0.0;       ///< 参考时刻平近点角 M0，单位：rad
    double deltaN = 0.0;   ///< 平均运动改正 Δn，单位：rad/s

    double cuc = 0.0;
    double cus = 0.0;
    double crc = 0.0;
    double crs = 0.0;
    double cic = 0.0;
    double cis = 0.0;
};

/**
 * @brief 卫星计算结果（ECEF 状态 + 钟差/钟速）
 */
struct SatState {
    Vec3 pos_ecef;          ///< 卫星 ECEF 位置 (m)
    Vec3 vel_ecef;          ///< 卫星 ECEF 速度 (m/s)
    double clk_s = 0.0;     ///< 卫星钟差 (s)
    double clk_drift = 0.0; ///< 卫星钟速 (s/s)
    bool valid = false;     ///< 计算结果有效性标志（数值有效性）
    bool eph_expired = false; ///< 星历是否超时（GPS: 7200s；BDS: 3600s）
};

/**
 * @brief 基于接收机表面时刻和伪距观测，迭代计算信号发射时刻的 GPS 卫星状态（含 Sagnac 改正）
 * @param eph GPS 广播星历
 * @param t_rx_gpst 信号接收时刻（GPST，单位：秒）
 * @param pseudorange_m 伪距观测值（单位：米）
 * @param rx_pos_approx 接收机近似坐标 (ECEF，单位：米)
 * @param state 输出：发射时刻的卫星状态（位置、速度、钟差、钟速）
 * @return true: 计算成功；false: 计算失败或数值异常
 */
bool ComputeGpsSatStateIterative(const GpsEph& eph, double t_rx_gpst, double pseudorange_m, const Vec3& rx_pos_approx, SatState& state);

/**
 * @brief 基于接收机表面时刻和伪距观测，迭代计算信号发射时刻的 BDS 卫星状态（含 Sagnac 改正）
 * @param eph BDS 广播星历
 * @param t_rx_bdt 信号接收时刻（BDT，单位：秒）
 * @param pseudorange_m 伪距观测值（单位：米）
 * @param rx_pos_approx 接收机近似坐标 (ECEF，单位：米)
 * @param state 输出：发射时刻的卫星状态（位置、速度、钟差、钟速）
 * @return true: 计算成功；false: 计算失败或数值异常
 */
bool ComputeBdsSatStateIterative(const BdsEph& eph, double t_rx_bdt, double pseudorange_m, const Vec3& rx_pos_approx, SatState& state);

/**
 * @brief 计算 GPS 卫星位置、速度与钟差/钟速
 * @param eph GPS 广播星历（WGS84）
 * @param t_gpst 计算历元（周内秒），单位：秒 (s)
 * @param transit_time_s 信号传播时间（用于 Sagnac 效应的地球自转改正），单位：秒 (s)
 * @param state 输出卫星状态
 * @return true: 计算完成且数值有效；false: 计算失败或数值异常
 */
bool ComputeGpsSatState(const GpsEph& eph, double t_gpst, double transit_time_s, SatState& state);

/**
 * @brief 计算 BDS 卫星位置、速度与钟差/钟速
 * @param eph BDS 广播星历（CGCS2000）
 * @param t_bdt 计算历元（周内秒，BDT），单位：秒 (s)
 * @param transit_time_s 信号传播时间（用于 Sagnac 效应的地球自转改正），单位：秒 (s)
 * @param state 输出卫星状态
 * @return true: 计算完成且数值有效；false: 计算失败或数值异常
 */
bool ComputeBdsSatState(const BdsEph& eph, double t_bdt, double transit_time_s, SatState& state);

} // namespace gnss

#endif // SAT_POS_CALC_H

