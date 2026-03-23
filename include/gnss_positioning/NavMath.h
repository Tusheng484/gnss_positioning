/**
 * @file NavMath.h
 * @brief GNSS 基础数学库头文件
 * @author Unknown
 * @date Unknown
 *
 * 包含时间系统转换（UTC/GPST/MJD）与坐标系统转换（WGS84）相关的数据结构与接口声明。
 */

#ifndef NAVMATH_H
#define NAVMATH_H

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>

namespace gnss {

/**
 * @brief 通用时间结构体
 * 对应民用习惯的年月日时分秒表示，通常用于表示 UTC 或本地时间。
 */
struct COMMONTIME {
    unsigned short Year;   ///< 年 (例如 2024)
    unsigned short Month;  ///< 月 (1-12)
    unsigned short Day;    ///< 日 (1-31)
    unsigned short Hour;   ///< 时 (0-23)
    unsigned short Minute; ///< 分 (0-59)
    double Second;         ///< 秒 (0.000-59.999...)，采用 double 保证毫秒级精度
};

/**
 * @brief 简化儒略日结构体（MJD）
 * 将儒略日（JD）减去 2400000.5 天，起始历元为 1858年11月17日 00:00:00。
 */
struct MJDTIME {
    int Days;       ///< MJD 整数部分 (天数)
    double FracDay; ///< MJD 小数部分 (不足一天的部分，范围 [0, 1))
    
    MJDTIME() : Days(0), FracDay(0.0) {}
};

/**
 * @brief GPS 时间结构体（GPST）
 * 卫星导航系统内部时间参考，由 GPS 周和周内秒组成。
 * 起始历元为 1980年1月6日 00:00:00（UTC）。
 */
struct GPSTIME {
    unsigned short Week; ///< GPS 周 (从 0 开始计数)
    double SecOfWeek;    ///< GPS 周内秒 (范围 [0, 604800.0))
    
    GPSTIME() : Week(0), SecOfWeek(0.0) {}
};

/**
 * @brief 地心空间直角坐标结构体（ECEF XYZ）
 * 参考系：WGS84
 * 单位：米 (m)
 */
struct XYZ {
    double x; ///< X轴坐标 (指向本初子午线与赤道交点)
    double y; ///< Y轴坐标 (指向东经90度与赤道交点)
    double z; ///< Z轴坐标 (指向地球北极)
};

/**
 * @brief 大地坐标结构体（BLH）
 * 参考系：WGS84 椭球体
 */
struct BLH {
    double longitude; ///< 经度 L，单位：弧度 (rad)，正值为东经
    double latitude;  ///< 纬度 B，单位：弧度 (rad)，正值为北纬
    double height;    ///< 大地高 H，单位：米 (m)
};

/* --- 时间转换接口 --- */

/**
 * @brief 通用时间转简化儒略日 (COMMONTIME -> MJD)
 * @param commonTime 输入通用时间
 * @param mjdTime 输出简化儒略日
 * @return 0: 成功；1: 输入参数不合法（例如月份越界）
 */
int CommonTimeToMjdTime(const COMMONTIME& commonTime, MJDTIME& mjdTime);

/**
 * @brief 简化儒略日转通用时间 (MJD -> COMMONTIME)
 * @param mjdTime 输入简化儒略日
 * @param commonTime 输出通用时间
 * @return 0: 成功；1: 失败
 */
int MjdTimeToCommonTime(const MJDTIME& mjdTime, COMMONTIME& commonTime);

/**
 * @brief 简化儒略日转 GPS 时间 (MJD -> GPST)
 * @param mjdTime 输入简化儒略日
 * @param gpsTime 输出 GPS 时间
 * @return 0: 成功；1: 时间早于 GPS 起始历元
 */
int MjdTimeToGpsTime(const MJDTIME& mjdTime, GPSTIME& gpsTime);

/**
 * @brief GPS 时间转简化儒略日 (GPST -> MJD)
 * @param gpsTime 输入 GPS 时间
 * @param mjdTime 输出简化儒略日
 * @return 0: 成功；1: 输入周内秒越界
 */
int GpsTimeToMjdTime(const GPSTIME& gpsTime, MJDTIME& mjdTime);

/**
 * @brief 通用时间转 GPS 时间 (COMMONTIME -> GPST)
 * @param commonTime 输入通用时间
 * @param gpsTime 输出 GPS 时间
 * @return 0: 成功；1: 失败
 */
int CommonTimeToGpsTime(const COMMONTIME& commonTime, GPSTIME& gpsTime);

/**
 * @brief GPS 时间转通用时间 (GPST -> COMMONTIME)
 * @param gpsTime 输入 GPS 时间
 * @param commonTime 输出通用时间
 * @return 0: 成功；1: 失败
 */
int GpsTimeToCommonTime(const GPSTIME& gpsTime, COMMONTIME& commonTime);

/* --- 坐标转换接口 --- */

/**
 * @brief WGS84 大地坐标转地心空间直角坐标 (BLH -> XYZ)
 * 采用标准椭球投影公式计算。
 * @param blh 输入大地坐标（longitude/latitude 为弧度，height 为米）
 * @param xyz 输出空间直角坐标（ECEF，单位：米）
 * @return 0: 成功；1: 纬度输入越界
 */
int BLHToXYZ(const BLH& blh, XYZ& xyz);

/**
 * @brief WGS84 地心空间直角坐标转大地坐标 (XYZ -> BLH)
 * 采用迭代法计算纬度，收敛阈值为 1e-12。
 * @param xyz 输入空间直角坐标（ECEF，单位：米）
 * @param blh 输出大地坐标（longitude/latitude 为弧度，height 为米）
 * @return 0: 成功；1: 坐标位于地心附近导致无法计算
 */
int XYZToBLH(const XYZ& xyz, BLH& blh);

/**
 * @brief 地心空间直角坐标转站心坐标 (XYZ -> ENU)
 * @param origin_xyz 测站的 ECEF 坐标
 * @param target_xyz 目标的 ECEF 坐标
 * @param E 东向分量
 * @param N 北向分量
 * @param U 天向分量
 * @return 0: 成功；1: 失败
 */
int XyzToEnu(const XYZ& origin_xyz, const XYZ& target_xyz, double& E, double& N, double& U);

} // namespace gnss

#endif // NAVMATH_H
