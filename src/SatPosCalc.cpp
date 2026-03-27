/**
 * @file SatPosCalc.cpp
 * @brief 广播星历卫星位置、速度与钟差/钟速计算模块实现
 * @author Jiaoyang Wu
 * @date 2026-03-27
 */

#include "gnss_positioning/SatPosCalc.h"

#include <algorithm>
#include <cmath>

namespace gnss {
namespace {

// 周内秒边界：用于跨周归一化与周跳修正。
constexpr double kHalfWeekSec = 302400.0;
constexpr double kWeekSec = 604800.0;

// 时间系统偏移：GPST = BDT + 14s（工程常用固定偏移，适用于课程数据与常见实验数据）。
constexpr double kBdtGpsOffsetSec = 14.0;

// GPS（WGS84）轨道动力学常量。
constexpr double kMuGps = 3.986005e14;
constexpr double kOmegaEGps = 7.2921151467e-5;

// BDS（CGCS2000）轨道动力学常量。
constexpr double kMuBds = 3.986004418e14;
constexpr double kOmegaEBds = 7.2921150e-5;

// 相对论改正：Δtr = F * e * sqrt(A) * sin(E)。
constexpr double kRelF = -4.442807633e-10;

// 星历有效期：用于输出 eph_expired 标志（不强制剔除，由上层策略决定）。
constexpr double kGpsEphValidSec = 7200.0;
constexpr double kBdsEphValidSec = 3600.0;

// 数值稳定性参数：Kepler 迭代次数、收敛阈值与分母阈值。
constexpr int kKeplerMaxIter = 30;
constexpr double kKeplerTol = 1e-13;
constexpr double kDenomEps = 1e-12;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// BDS GEO 特殊旋转：Rx(+5°)。
constexpr double kBdsGeoTiltRad = 5.0 * kDegToRad;

inline bool IsFinite(double x) { return std::isfinite(x); }

double NormalizeSecInWeek(double t_sec) {
    double out = t_sec;
    while (out >= kWeekSec) out -= kWeekSec;
    while (out < 0.0) out += kWeekSec;
    return out;
}

double NormalizeWeekCrossover(double dt_sec) {
    // 周跳修正：将时间差限制到 [-302400, +302400]，避免跨周导致 tk/dt 畸变。
    if (dt_sec > kHalfWeekSec) dt_sec -= kWeekSec;
    if (dt_sec < -kHalfWeekSec) dt_sec += kWeekSec;
    return dt_sec;
}

double ConvertBdtToGpStSec(double t_bdt_sec) {
    // 时间系统统一：模块内部统一采用 GPST，BDS 输入按 BDT 解释并转换到 GPST。
    return NormalizeSecInWeek(t_bdt_sec + kBdtGpsOffsetSec);
}

bool IsBdsGeoPrn(uint32_t prn) {
    return (prn >= 1 && prn <= 5) || (prn >= 59 && prn <= 62);
}

bool SolveKepler(double Mk, double ecc, double& Ek) {
    if (!(ecc >= 0.0 && ecc < 1.0) || !IsFinite(Mk)) return false;

    // Kepler 方程：E - e*sin(E) = M，采用牛顿迭代求解偏近点角 E。
    double M = std::remainder(Mk, 2.0 * kPi);
    Ek = M;

    for (int i = 0; i < kKeplerMaxIter; ++i) {
        double sE = std::sin(Ek);
        double cE = std::cos(Ek);
        double f = Ek - ecc * sE - M;
        double fp = 1.0 - ecc * cE;
        if (std::abs(fp) < kDenomEps) return false;
        double dE = -f / fp;
        Ek += dE;
        if (std::abs(dE) < kKeplerTol) return true;
    }

    return false;
}

struct OrbitalPlane {
    double x = 0.0;
    double y = 0.0;
    double x_dot = 0.0;
    double y_dot = 0.0;
    double i = 0.0;
    double i_dot = 0.0;
    double Ek = 0.0;
    double E_dot = 0.0;
};

bool ComputeOrbitalPlaneAnalytical(double mu,
                                  double t,
                                  double toe,
                                  double sqrtA,
                                  double ecc,
                                  double inc0,
                                  double inc_dot,
                                  double argper,
                                  double m0,
                                  double dN,
                                  double cuc,
                                  double cus,
                                  double crc,
                                  double crs,
                                  double cic,
                                  double cis,
                                  OrbitalPlane& out,
                                  double& tk_out) {
    if (!(mu > 0.0) || !(sqrtA > 0.0) || !(ecc >= 0.0 && ecc < 1.0)) return false;

    // tk：观测历元到参考历元的时间差（周跳修正后）。
    double tk = NormalizeWeekCrossover(t - toe);
    tk_out = tk;

    // 平均角速度：n = n0 + Δn，其中 n0 = sqrt(μ) / A^(3/2)。
    double sqrtA3 = sqrtA * sqrtA * sqrtA;
    double n0 = std::sqrt(mu) / sqrtA3;
    double n = n0 + dN;
    if (!IsFinite(n)) return false;

    double Mk = m0 + n * tk;

    double Ek = 0.0;
    if (!SolveKepler(Mk, ecc, Ek)) return false;

    double sinE = std::sin(Ek);
    double cosE = std::cos(Ek);
    double denom = 1.0 - ecc * cosE;
    if (std::abs(denom) < kDenomEps) return false;

    // Ė = n / (1 - e*cosE)。
    double E_dot = n / denom;
    if (!IsFinite(E_dot)) return false;

    // 真近点角 v 使用 atan2 保证象限正确；v̇ 由链式法则得到。
    double sqrt_1_e2 = std::sqrt(std::max(0.0, 1.0 - ecc * ecc));
    double vk = std::atan2(sqrt_1_e2 * sinE, cosE - ecc);
    double v_dot = sqrt_1_e2 * E_dot / denom;

    double phi = vk + argper;
    double phi_dot = v_dot;

    double sin2p = std::sin(2.0 * phi);
    double cos2p = std::cos(2.0 * phi);

    // 二阶谐波摄动改正项及其时间导数：用于补偿轨道参数的周期性误差。
    double du = cus * sin2p + cuc * cos2p;
    double dr = crs * sin2p + crc * cos2p;
    double di = cis * sin2p + cic * cos2p;

    double du_dot = 2.0 * (cus * cos2p - cuc * sin2p) * phi_dot;
    double dr_dot = 2.0 * (crs * cos2p - crc * sin2p) * phi_dot;
    double di_dot = 2.0 * (cis * cos2p - cic * sin2p) * phi_dot;

    double u = phi + du;
    double u_dot = phi_dot + du_dot;

    double A = sqrtA * sqrtA;
    double r = A * denom + dr;
    double r_dot = A * ecc * sinE * E_dot + dr_dot;

    double i = inc0 + inc_dot * tk + di;
    double i_dot = inc_dot + di_dot;

    double su = std::sin(u);
    double cu = std::cos(u);

    out.x = r * cu;
    out.y = r * su;
    out.x_dot = r_dot * cu - r * su * u_dot;
    out.y_dot = r_dot * su + r * cu * u_dot;
    out.i = i;
    out.i_dot = i_dot;
    out.Ek = Ek;
    out.E_dot = E_dot;

    return IsFinite(out.x) && IsFinite(out.y) && IsFinite(out.x_dot) && IsFinite(out.y_dot) &&
           IsFinite(out.i) && IsFinite(out.i_dot) && IsFinite(out.Ek) && IsFinite(out.E_dot);
}

void ComputeEcefFromOrbitalPlane(const OrbitalPlane& orb,
                                 double Omega,
                                 double Omega_dot,
                                 Vec3& pos_ecef,
                                 Vec3& vel_ecef) {
    // 轨道平面坐标 -> ECEF：按照广播星历标准模型旋转到地固系，并使用链式法则解析求导得到速度。
    double cO = std::cos(Omega);
    double sO = std::sin(Omega);
    double ci = std::cos(orb.i);
    double si = std::sin(orb.i);

    pos_ecef.x = orb.x * cO - orb.y * ci * sO;
    pos_ecef.y = orb.x * sO + orb.y * ci * cO;
    pos_ecef.z = orb.y * si;

    vel_ecef.x = orb.x_dot * cO - orb.x * sO * Omega_dot - orb.y_dot * ci * sO +
                 orb.y * si * orb.i_dot * sO - orb.y * ci * cO * Omega_dot;
    vel_ecef.y = orb.x_dot * sO + orb.x * cO * Omega_dot + orb.y_dot * ci * cO -
                 orb.y * si * orb.i_dot * cO - orb.y * ci * sO * Omega_dot;
    vel_ecef.z = orb.y_dot * si + orb.y * ci * orb.i_dot;
}

Vec3 Rx(double angle_rad, const Vec3& v) {
    double c = std::cos(angle_rad);
    double s = std::sin(angle_rad);
    return Vec3{v.x, c * v.y - s * v.z, s * v.y + c * v.z};
}

Vec3 Rz(double angle_rad, const Vec3& v) {
    double c = std::cos(angle_rad);
    double s = std::sin(angle_rad);
    return Vec3{c * v.x - s * v.y, s * v.x + c * v.y, v.z};
}

Vec3 dRz_dt(double angle_rad, double omega_rad_s, const Vec3& v) {
    double c = std::cos(angle_rad);
    double s = std::sin(angle_rad);
    return Vec3{
        omega_rad_s * (-s * v.x - c * v.y),
        omega_rad_s * (c * v.x - s * v.y),
        0.0};
}

bool ComputeClock(double t,
                  double toc,
                  double toe,
                  double mu,
                  double af0,
                  double af1,
                  double af2,
                  double tgd,
                  double sqrtA,
                  double ecc,
                  double dN,
                  double m0,
                  double& clk_s,
                  double& clk_drift) {
    // 卫星钟差与钟速（广播星历模型）：
    // clk = af0 + af1*dt + af2*dt^2 + Δtr - TGD
    // clk_dot = af1 + 2*af2*dt + Δṫr
    // TGD 用于单频等效改正；多频情况下应按频点进行设备延迟建模。
    double dt = NormalizeWeekCrossover(t - toc);
    if (std::abs(dt) > kHalfWeekSec) return false;

    double tk = NormalizeWeekCrossover(t - toe);
    if (std::abs(tk) > kHalfWeekSec) return false;

    double sqrtA3 = sqrtA * sqrtA * sqrtA;
    double n0 = std::sqrt(mu) / sqrtA3;
    double n = n0 + dN;
    if (!IsFinite(n)) return false;

    double Mk = m0 + n * tk;
    double Ek = 0.0;
    if (!SolveKepler(Mk, ecc, Ek)) return false;

    double cosE = std::cos(Ek);
    double denom = 1.0 - ecc * cosE;
    if (std::abs(denom) < kDenomEps) return false;
    double E_dot = n / denom;

    double relativity = kRelF * ecc * sqrtA * std::sin(Ek);
    double relativity_dot = kRelF * ecc * sqrtA * std::cos(Ek) * E_dot;

    clk_s = af0 + af1 * dt + af2 * dt * dt + relativity - tgd;
    clk_drift = af1 + 2.0 * af2 * dt + relativity_dot;

    return IsFinite(clk_s) && IsFinite(clk_drift);
}

} // namespace

bool ComputeGpsSatState(const GpsEph& eph, double t_gpst, double transit_time_s, SatState& state) {
    state = SatState{};

    (void)transit_time_s;

    double tk = NormalizeWeekCrossover(t_gpst - eph.toe);
    state.eph_expired = (std::abs(tk) > kGpsEphValidSec);

    if (!ComputeClock(t_gpst, eph.toc, eph.toe, kMuGps, eph.af0, eph.af1, eph.af2, eph.tgd, eph.sqrtA, eph.e, eph.deltaN,
                      eph.M0, state.clk_s, state.clk_drift)) {
        return false;
    }

    OrbitalPlane orb;
    double tk_orb = 0.0;
    if (!ComputeOrbitalPlaneAnalytical(kMuGps, t_gpst, eph.toe, eph.sqrtA, eph.e, eph.i0, eph.iDot, eph.omega, eph.M0,
                                       eph.deltaN, eph.cuc, eph.cus, eph.crc, eph.crs, eph.cic, eph.cis, orb, tk_orb)) {
        return false;
    }

    // Ω = Ω0 + (Ωdot - ωe)*tk - ωe*toe。
    double Omega = eph.Omega0 + (eph.OmegaDot - kOmegaEGps) * tk_orb - kOmegaEGps * eph.toe;
    double Omega_dot = eph.OmegaDot - kOmegaEGps;
    ComputeEcefFromOrbitalPlane(orb, Omega, Omega_dot, state.pos_ecef, state.vel_ecef);

    state.valid = IsFinite(state.pos_ecef.x) && IsFinite(state.pos_ecef.y) && IsFinite(state.pos_ecef.z) &&
                  IsFinite(state.vel_ecef.x) && IsFinite(state.vel_ecef.y) && IsFinite(state.vel_ecef.z) &&
                  IsFinite(state.clk_s) && IsFinite(state.clk_drift);
    return true;
}

bool ComputeBdsSatState(const BdsEph& eph, double t_bdt, double transit_time_s, SatState& state) {
    state = SatState{};

    (void)transit_time_s;

    // BDS 时间系统统一：入口参数按 BDT 解释，在此转换为 GPST 参与后续所有计算。
    double t_gpst = ConvertBdtToGpStSec(t_bdt);
    double toe_gpst = ConvertBdtToGpStSec(eph.toe);
    double toc_gpst = ConvertBdtToGpStSec(eph.toc);

    double tk = NormalizeWeekCrossover(t_gpst - toe_gpst);
    state.eph_expired = (std::abs(tk) > kBdsEphValidSec);

    if (!ComputeClock(t_gpst, toc_gpst, toe_gpst, kMuBds, eph.af0, eph.af1, eph.af2, eph.tgd, eph.sqrtA, eph.e, eph.deltaN,
                      eph.M0, state.clk_s, state.clk_drift)) {
        return false;
    }

    OrbitalPlane orb;
    double tk_orb = 0.0;
    if (!ComputeOrbitalPlaneAnalytical(kMuBds, t_gpst, toe_gpst, eph.sqrtA, eph.e, eph.i0, eph.iDot, eph.omega, eph.M0,
                                       eph.deltaN, eph.cuc, eph.cus, eph.crc, eph.crs, eph.cic, eph.cis, orb, tk_orb)) {
        return false;
    }

    bool is_geo = IsBdsGeoPrn(eph.prn);
    if (!is_geo) {
        // BDS MEO/IGSO：与 GPS 类似的地固系建模，但 μ、ωe 使用 BDS 常量。
        double Omega = eph.Omega0 + (eph.OmegaDot - kOmegaEBds) * tk_orb - kOmegaEBds * toe_gpst;
        double Omega_dot = eph.OmegaDot - kOmegaEBds;
        ComputeEcefFromOrbitalPlane(orb, Omega, Omega_dot, state.pos_ecef, state.vel_ecef);
    } else {
        // BDS GEO：采用 GEO 专用建模。注意：不能使用与 MEO/IGSO 相同的 Ω 公式。
        // 1) 先在 GEO 自身参考系下构建（不包含 -ωe*toe 项）
        // 2) 施加 Rx(-5°) 倾斜
        // 3) 再以 Rz(ωe*tk) 旋转至地固系（BDCS/ECEF）
        Vec3 pos_geo;
        Vec3 vel_geo;

        // GEO 分支使用 Ω = Ω0 + Ωdot*tk - ωe*toe，地球自转项不并入 Ωdot。
        double Omega = eph.Omega0 + eph.OmegaDot * tk_orb - kOmegaEBds * toe_gpst;
        double Omega_dot = eph.OmegaDot;
        ComputeEcefFromOrbitalPlane(orb, Omega, Omega_dot, pos_geo, vel_geo);

        Vec3 pos_tilt = Rx(kBdsGeoTiltRad, pos_geo);
        Vec3 vel_tilt = Rx(kBdsGeoTiltRad, vel_geo);

        // GEO 专用旋转矩阵：Rz(ωe*tk) Rx(-5°)。
        // 注意：BDS ICD 常用写法的 Rz(ωe*tk) 对应坐标旋转矩阵 [cos sin; -sin cos]，等价于本实现中的 Rz(-θ)。
        double theta = kOmegaEBds * tk_orb;
        state.pos_ecef = Rz(-theta, pos_tilt);
        state.vel_ecef = Rz(-theta, vel_tilt);

        Vec3 rot_term = dRz_dt(-theta, -kOmegaEBds, pos_tilt);
        state.vel_ecef.x += rot_term.x;
        state.vel_ecef.y += rot_term.y;
        state.vel_ecef.z += rot_term.z;

    }

    state.valid = IsFinite(state.pos_ecef.x) && IsFinite(state.pos_ecef.y) && IsFinite(state.pos_ecef.z) &&
                  IsFinite(state.vel_ecef.x) && IsFinite(state.vel_ecef.y) && IsFinite(state.vel_ecef.z) &&
                  IsFinite(state.clk_s) && IsFinite(state.clk_drift);
    return true;
}

} // namespace gnss
