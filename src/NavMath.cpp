/**
 * @file NavMath.cpp
 * @brief GNSS 基础数学计算库实现
 * 
 * 本模块实现了导航定位中的核心算法，包括：
 * 1. 时间系统转换：支持 CommonTime (UTC/本地时), MJD (简化儒略日), GPST (GPS时间) 互转。
 * 2. 坐标系统转换：支持 WGS84 参考系下的 Geodetic (大地坐标 BLH) 与 ECEF (地心空间直角坐标 XYZ) 互转。
 * 
 * 算法说明：
 * - 坐标转换采用 WGS84 椭球参数 (a=6378137.0, f=1/298.257223563)。
 * - XYZ 到 BLH 的转换采用迭代法，确保在高程较大或极地地区仍有极高精度。
 */

#define _USE_MATH_DEFINES
#include <cmath>
#include <gnss_positioning/NavMath.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace gnss {

/**
 * @brief WGS84 椭球体常数
 * 定义参考：GPS ICD-IS-200
 */
const double WGS84_A = 6378137.0;              ///< 长半轴 (Semi-major axis), 单位: m
const double WGS84_E2 = 0.00669437999014;      ///< 第一偏心率平方 (First eccentricity squared)

/**
 * @brief 通用时间转简化儒略日 (CommonTime -> MJD)
 * 算法逻辑：基于儒略历/公历转换公式，将年月日时分秒映射到连续的天数轴。
 */
int CommonTimeToMjdTime(const COMMONTIME& commonTime, MJDTIME& mjdTime) {
    // 数据合法性校验 (Sanity Check)
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

    // 计算世界时 (UT) 的小数部分
    double UT = commonTime.Hour + commonTime.Minute / 60.0 + commonTime.Second / 3600.0;
    
    // MJD 计算公式 (起始于 1858-11-17)
    mjdTime.Days = int(365.25 * y) + int(30.6001 * (m + 1)) + commonTime.Day - 679019;
    mjdTime.FracDay = UT / 24.0;

    return 0;
}

/**
 * @brief 简化儒略日转通用时间 (MJD -> CommonTime)
 * 采用 Fliegel-Van Flandern 算法及其变体。
 */
int MjdTimeToCommonTime(const MJDTIME& mjdTime, COMMONTIME& commonTime) {
    if (mjdTime.Days < -100000) { 
        return 1;
    }

    // 转换为儒略日 (Julian Date)
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
    commonTime.Minute = int(mins + 1e-12); // 处理浮点精度，防止 0.999999 被截断
    commonTime.Second = (mins - commonTime.Minute) * 60.0;

    // 处理秒进位：解决 59.999... 秒无法自动进位到分钟的问题
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
 * GPS 时间零历元：1980-01-06 00:00:00 UTC (对应 MJD 44244.0)
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
 * 流程：CommonTime -> MJD -> GPST
 */
int CommonTimeToGpsTime(const COMMONTIME& commonTime, GPSTIME& gpsTime) {
    MJDTIME mjd;
    int status = CommonTimeToMjdTime(commonTime, mjd);
    if (status != 0) return status;
    
    return MjdTimeToGpsTime(mjd, gpsTime);
}

/**
 * @brief GPS 时间转通用时间 (GPST -> CommonTime)
 * 流程：GPST -> MJD -> CommonTime
 */
int GpsTimeToCommonTime(const GPSTIME& gpsTime, COMMONTIME& commonTime) {
    MJDTIME mjd;
    int status = GpsTimeToMjdTime(gpsTime, mjd);
    if (status != 0) return status;
    
    return MjdTimeToCommonTime(mjd, commonTime);
}

/**
 * @brief 大地坐标转地心空间直角坐标 (BLH -> XYZ)
 * 公式：
 * X = (N + H) * cos(B) * cos(L)
 * Y = (N + H) * cos(B) * sin(L)
 * Z = (N * (1 - e^2) + H) * sin(B)
 * 其中 N 为卯酉圈曲率半径。
 */
int BLHToXYZ(const BLH& blh, XYZ& xyz) {
    if (blh.latitude < -1.5707963267948966 || blh.latitude > 1.5707963267948966) {
        return 1;
    }

    double sinB = std::sin(blh.latitude);
    double cosB = std::cos(blh.latitude);
    double sinL = std::sin(blh.longitude);
    double cosL = std::cos(blh.longitude);
    
    // 计算卯酉圈曲率半径 N
    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sinB * sinB);
    
    xyz.x = (N + blh.height) * cosB * cosL;
    xyz.y = (N + blh.height) * cosB * sinL;
    xyz.z = (N * (1.0 - WGS84_E2) + blh.height) * sinB;

    return 0;
}

/**
 * @brief 地心空间直角坐标转大地坐标 (XYZ -> BLH)
 * 算法：采用纬度迭代法，具有良好的数值稳定性。
 */
int XYZToBLH(const XYZ& xyz, BLH& blh) {
    double r = std::sqrt(xyz.x * xyz.x + xyz.y * xyz.y);
    
    // 特殊处理：坐标位于两极附近
    if (r < 1e-6) {
        blh.longitude = 0.0;
        blh.latitude = (xyz.z > 0) ? M_PI/2.0 : -M_PI/2.0;
        blh.height = std::fabs(xyz.z) - WGS84_A * std::sqrt(1.0 - WGS84_E2);
        return 0;
    }

    blh.longitude = std::atan2(xyz.y, xyz.x);
    
    // 纬度迭代计算 (迭代初值)
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
 * 算法：首先计算测站的大地坐标，然后构建旋转矩阵将目标向量转换到站心坐标系。
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