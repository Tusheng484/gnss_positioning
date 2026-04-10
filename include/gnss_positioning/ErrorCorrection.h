/**
 * @file ErrorCorrection.h
 * @brief SPP 误差改正模块：对流层（Hopfield）与电离层（Klobuchar）以及双频消电离层组合
 * @author Jiaoyang Wu
 * @date 2026-03-30
 *
 * 单位约定：
 * - 角度：弧度 (rad)
 * - 距离：米 (m)
 * - 时间：秒 (s)
 */
 
#ifndef ERROR_CORRECTION_H
#define ERROR_CORRECTION_H

#include <cstddef>

namespace gnss {

/**
 * @brief 物理常量与频点常量
 */
struct ErrorConst {
    static constexpr double SPEED_OF_LIGHT = 299792458.0; ///< 光速 (m/s)
    static constexpr double PI = 3.14159265358979323846;

    static constexpr double GPS_L1 = 1575.42e6;  ///< GPS L1 载波频率 (Hz)
    static constexpr double GPS_L2 = 1227.60e6;  ///< GPS L2 载波频率 (Hz)
    static constexpr double BDS_B1I = 1561.098e6; ///< BDS B1I 载波频率 (Hz)
    static constexpr double BDS_B3I = 1268.52e6; ///< BDS B3I 载波频率 (Hz)
};

/**
 * @brief Hopfield 对流层延迟模型
 * @param H 测站大地高，单位：米 (m)
 * @param E 卫星高度角，单位：弧度 (rad)
 * @param T0 参考面温度，单位：开尔文 (K)，默认 288.16K
 * @param p0 参考面气压，单位：毫巴 (mbar)，默认 1013.25mbar
 * @param RH0 参考面相对湿度，单位：百分比 (%)，默认 50%
 * @return 对流层延迟改正量，单位：米 (m)
 */
double HopfieldTropDelay(double H, double E, double T0 = 288.16, double p0 = 1013.25, double RH0 = 50.0);

/**
 * @brief Saastamoinen 对流层延迟模型（标准气象参数默认值）
 * @param lat 测站纬度，单位：弧度 (rad)
 * @param H 测站大地高，单位：米 (m)
 * @param E 卫星高度角，单位：弧度 (rad)
 * @param T0 参考面温度，单位：开尔文 (K)，默认 288.15K
 * @param p0 参考面气压，单位：毫巴 (mbar)，默认 1013.25mbar
 * @param RH0 参考面相对湿度，单位：百分比 (%)，默认 50%
 * @return 对流层斜路径延迟改正量，单位：米 (m)
 */
double SaastamoinenTropDelay(double lat, double H, double E, double T0 = 288.15, double p0 = 1013.25, double RH0 = 50.0);

/**
 * @brief GPS Klobuchar 电离层延迟模型（L1 等效）
 * @param tow GPS 周内秒，单位：秒 (s)
 * @param pos 测站位置 [lat, lon, h]，lat/lon 为弧度，h 为米
 * @param az 卫星方位角，单位：弧度 (rad)
 * @param el 卫星高度角，单位：弧度 (rad)
 * @param alpha Klobuchar 参数 α0-α3
 * @param beta Klobuchar 参数 β0-β3
 * @return L1 频率上的电离层延迟改正量，单位：米 (m)
 */
double GPSKlobucharIonoDelay(double tow,
                             const double pos[3],
                             double az,
                             double el,
                             const double alpha[4],
                             const double beta[4]);

/**
 * @brief BDS Klobuchar 电离层延迟模型（B1I 等效）
 * @param tow BDT 周内秒，单位：秒 (s)
 * @param pos 测站位置 [lat, lon, h]，lat/lon 为弧度，h 为米
 * @param az 卫星方位角，单位：弧度 (rad)
 * @param el 卫星高度角，单位：弧度 (rad)
 * @param alpha Klobuchar 参数 α0-α3（来自 BDS 导航电文）
 * @param beta Klobuchar 参数 β0-β3（来自 BDS 导航电文）
 * @return B1I 频率上的电离层延迟改正量，单位：米 (m)
 */
double BDSKlobucharIonoDelay(double tow,
                             const double pos[3],
                             double az,
                             double el,
                             const double alpha[4],
                             const double beta[4]);

/**
 * @brief 双频消电离层组合（伪距/相位通用形式）
 * @param P1 频率 1 观测值，单位：米 (m)
 * @param P2 频率 2 观测值，单位：米 (m)
 * @param f1 频率 1，单位：赫兹 (Hz)
 * @param f2 频率 2，单位：赫兹 (Hz)
 * @return 消电离层组合观测值，单位：米 (m)
 */
double IonFreeCombo(double P1, double P2, double f1, double f2);

/**
 * @brief 电离层改正频率转换（按 (f1/f2)^2 缩放）
 * @param ionoL1 f1 频率上的电离层延迟，单位：米 (m)
 * @param f1 原始频率，单位：赫兹 (Hz)
 * @param f2 目标频率，单位：赫兹 (Hz)
 * @return 目标频率上的电离层延迟，单位：米 (m)
 */
double IonoL1ToFreq(double ionoL1, double f1, double f2);

} // namespace gnss

#endif // ERROR_CORRECTION_H

