/**
 * @file NavMath.cpp
 * @brief GNSS 基础数学计算库实现
 * @author Jiaoyang Wu
 * @date 2026-03-15
 *
 * 本文件实现时间系统转换与坐标系统转换的基础算法，作为定位解算模块的通用数学支撑。
 */

#define _USE_MATH_DEFINES
#include <cmath>
#include <gnss_positioning/NavMath.h>

#ifndef M_PI
// 兼容性处理：部分编译环境未提供 M_PI 宏定义。
#define M_PI 3.14159265358979323846
#endif

namespace gnss {

/**
 * @brief WGS84 椭球体常数
 *
 * 统一在本模块内部定义，避免不同实现文件重复声明导致的数值口径不一致。
 */
const double WGS84_A = 6378137.0;              ///< 长半轴，单位：米 (m)
const double WGS84_E2 = 0.00669437999014;      ///< 第一偏心率平方

/**
 * @brief 通用时间转简化儒略日 (CommonTime -> MJD)
 * @param commonTime 输入通用时间
 * @param mjdTime 输出简化儒略日
 * @return 0: 成功；1: 输入参数不合法
 */
int CommonTimeToMjdTime(const COMMONTIME& commonTime, MJDTIME& mjdTime) {
    // 输入合法性校验：避免越界日期导致的儒略日计算异常。
    if (commonTime.Month < 1 || commonTime.Month > 12 || 
        commonTime.Day < 1 || commonTime.Day > 31 ||
        commonTime.Hour > 23 || commonTime.Minute > 59 || 
        commonTime.Second < 0 || commonTime.Second >= 60.0) {
        return 1;
    }

    int y = commonTime.Year;
    int m = commonTime.Month;
    if (m <= 2) {
        y -= 1;
        m += 12;
    }

    // 计算日内小数部分（小时制）。
    double UT = commonTime.Hour + commonTime.Minute / 60.0 + commonTime.Second / 3600.0;
    
    // 简化儒略日起始历元：1858-11-17 00:00:00。
    mjdTime.Days = int(365.25 * y) + int(30.6001 * (m + 1)) + commonTime.Day - 679019;
    mjdTime.FracDay = UT / 24.0;

    return 0;
}

/**
 * @brief 简化儒略日转通用时间 (MJD -> CommonTime)
 * @param mjdTime 输入简化儒略日
 * @param commonTime 输出通用时间
 * @return 0: 成功；1: 失败
 */
int MjdTimeToCommonTime(const MJDTIME& mjdTime, COMMONTIME& commonTime) {
    if (mjdTime.Days < -100000) { 
        return 1;
    }

    // 先恢复为儒略日（JD），再按历法公式分解为年月日与日内时间。
    double JD = mjdTime.Days + mjdTime.FracDay + 2400000.5;
    
    int a = int(JD + 0.5);
    int b = a + 1537;
    int c = int((b - 122.1) / 365.25);
    int d = int(365.25 * c);
    int e = int((b - d) / 30.6001);
    
    double D_full = b - d - int(30.6001 * e) + (JD + 0.5 - int(JD + 0.5));
    
    commonTime.Month = e - 1 - 12 * int(e / 14);
    commonTime.Year = c - 4715 - int((7 + commonTime.Month) / 10);
    commonTime.Day = int(D_full);
    
    double frac_day = D_full - commonTime.Day;
    double hours = frac_day * 24.0;
    commonTime.Hour = int(hours);
    
    double mins = (hours - commonTime.Hour) * 60.0;
    commonTime.Minute = int(mins + 1e-12); // 浮点保护：避免 0.999999... 被截断导致分钟误差。
    commonTime.Second = (mins - commonTime.Minute) * 60.0;

    // 秒进位处理：避免 59.999... 无法进位导致的时间不连续。
    if (commonTime.Second >= 59.999999) {
        commonTime.Second = 0.0;
        commonTime.Minute++;
        if (commonTime.Minute >= 60) {
            commonTime.Minute = 0;
            commonTime.Hour++;
        }
    }
    if (commonTime.Second < 0) commonTime.Second = 0.0;

    return 0;
}

/**
 * @brief 简化儒略日转 GPS 时间 (MJD -> GPST)
 * @param mjdTime 输入简化儒略日
 * @param gpsTime 输出 GPS 时间
 * @return 0: 成功；1: 时间早于 GPS 起始历元
 */
int MjdTimeToGpsTime(const MJDTIME& mjdTime, GPSTIME& gpsTime) {
    if (mjdTime.Days < 44244) {
        return 1; // 早于 GPS 启动时间
    }

    double total_mjd = mjdTime.Days + mjdTime.FracDay;
    gpsTime.Week = int((total_mjd - 44244) / 7);
    gpsTime.SecOfWeek = (total_mjd - 44244 - gpsTime.Week * 7) * 86400.0;

    return 0;
}

/**
 * @brief GPS 时间转简化儒略日 (GPST -> MJD)
 * @param gpsTime 输入 GPS 时间
 * @param mjdTime 输出简化儒略日
 * @return 0: 成功；1: 输入周内秒越界
 */
int GpsTimeToMjdTime(const GPSTIME& gpsTime, MJDTIME& mjdTime) {
    if (gpsTime.SecOfWeek < 0 || gpsTime.SecOfWeek >= 604800.0) {
        return 1;
    }

    double total_mjd = 44244 + gpsTime.Week * 7 + gpsTime.SecOfWeek / 86400.0;
    mjdTime.Days = int(total_mjd);
    mjdTime.FracDay = total_mjd - mjdTime.Days;

    return 0;
}

/**
 * @brief 通用时间转 GPS 时间 (CommonTime -> GPST)
 * @param commonTime 输入通用时间
 * @param gpsTime 输出 GPS 时间
 * @return 0: 成功；1: 失败
 */
int CommonTimeToGpsTime(const COMMONTIME& commonTime, GPSTIME& gpsTime) {
    // 通过 MJD 作为中间时间轴，降低不同历法与时间表达形式之间的耦合。
    MJDTIME mjd;
    int status = CommonTimeToMjdTime(commonTime, mjd);
    if (status != 0) return status;
    
    return MjdTimeToGpsTime(mjd, gpsTime);
}

/**
 * @brief GPS 时间转通用时间 (GPST -> CommonTime)
 * @param gpsTime 输入 GPS 时间
 * @param commonTime 输出通用时间
 * @return 0: 成功；1: 失败
 */
int GpsTimeToCommonTime(const GPSTIME& gpsTime, COMMONTIME& commonTime) {
    MJDTIME mjd;
    int status = GpsTimeToMjdTime(gpsTime, mjd);
    if (status != 0) return status;
    
    return MjdTimeToCommonTime(mjd, commonTime);
}

/**
 * @brief 大地坐标转地心空间直角坐标 (BLH -> XYZ)
 * @param blh 输入大地坐标（longitude/latitude 为弧度，height 为米）
 * @param xyz 输出地心空间直角坐标（ECEF，单位：米）
 * @return 0: 成功；1: 纬度输入越界
 */
int BLHToXYZ(const BLH& blh, XYZ& xyz) {
    if (blh.latitude < -1.5707963267948966 || blh.latitude > 1.5707963267948966) {
        return 1;
    }

    double sinB = std::sin(blh.latitude);
    double cosB = std::cos(blh.latitude);
    double sinL = std::sin(blh.longitude);
    double cosL = std::cos(blh.longitude);
    
    // 卯酉圈曲率半径 N 反映椭球曲率对局部法线方向的影响，是 BLH->ECEF 的关键量。
    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sinB * sinB);
    
    xyz.x = (N + blh.height) * cosB * cosL;
    xyz.y = (N + blh.height) * cosB * sinL;
    xyz.z = (N * (1.0 - WGS84_E2) + blh.height) * sinB;

    return 0;
}

/**
 * @brief 地心空间直角坐标转大地坐标 (XYZ -> BLH)
 * @param xyz 输入地心空间直角坐标（ECEF，单位：米）
 * @param blh 输出大地坐标（longitude/latitude 为弧度，height 为米）
 * @return 0: 成功；1: 失败
 */
int XYZToBLH(const XYZ& xyz, BLH& blh) {
    double r = std::sqrt(xyz.x * xyz.x + xyz.y * xyz.y);
    
    // 特殊处理：极区附近 r≈0 时经度不稳定，直接给定经度并按极点近似计算高度。
    if (r < 1e-6) {
        blh.longitude = 0.0;
        blh.latitude = (xyz.z > 0) ? M_PI/2.0 : -M_PI/2.0;
        blh.height = std::fabs(xyz.z) - WGS84_A * std::sqrt(1.0 - WGS84_E2);
        return 0;
    }

    blh.longitude = std::atan2(xyz.y, xyz.x);
    
    // 纬度迭代：以椭球改正后的初值起步，迭代收敛以保证不同高度与纬度范围内的稳定性。
    double sinB, N;
    double lat = std::atan2(xyz.z, r * (1.0 - WGS84_E2));
    double lat_old;

    for (int i = 0; i < 10; ++i) {
        lat_old = lat;
        sinB = std::sin(lat);
        N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sinB * sinB);
        lat = std::atan2(xyz.z + WGS84_E2 * N * sinB, r);
        
        if (std::fabs(lat - lat_old) < 1e-12) break;
    }

    blh.latitude = lat;
    sinB = std::sin(lat);
    N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sinB * sinB);
    blh.height = r / std::cos(lat) - N;

    return 0;
}

/**
 * @brief 地心空间直角坐标转站心坐标 (XYZ -> ENU)
 * @param origin_xyz 测站的 ECEF 坐标
 * @param target_xyz 目标点的 ECEF 坐标
 * @param E 东向分量
 * @param N 北向分量
 * @param U 天向分量
 * @return 0: 成功；1: 失败
 */
int XyzToEnu(const XYZ& origin_xyz, const XYZ& target_xyz, double& E, double& N, double& U) {
    BLH origin_blh;
    if (XYZToBLH(origin_xyz, origin_blh) != 0) {
        return 1;
    }

    double sinL = std::sin(origin_blh.longitude);
    double cosL = std::cos(origin_blh.longitude);
    double sinB = std::sin(origin_blh.latitude);
    double cosB = std::cos(origin_blh.latitude);

    double dx = target_xyz.x - origin_xyz.x;
    double dy = target_xyz.y - origin_xyz.y;
    double dz = target_xyz.z - origin_xyz.z;

    E = -sinL * dx + cosL * dy;
    N = -sinB * cosL * dx - sinB * sinL * dy + cosB * dz;
    U = cosB * cosL * dx + cosB * sinL * dy + sinB * dz;

    return 0;
}

} // namespace gnss
