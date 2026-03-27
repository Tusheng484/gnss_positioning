/**
 * @file SatPosCalc.cpp
 * @brief 广播星历卫星位置、速度与钟差/钟速计算模块实现
 * @author Jiaoyang Wu
 * @date 2026-03-27
 */

#include "gnss_positioning/SatPosCalc.h"

#include <cmath>
#include <limits>

namespace gnss {
namespace {

// 周界常量：用于 tk/dt 的跨周归一化。
constexpr double kHalfWeekSec = 302400.0;
constexpr double kWeekSec = 604800.0;

// 引力常数（不同星座不得混用）。
constexpr double kMuGps = 3.986005e14;
constexpr double kMuBds = 3.986004418e14;

// 地球自转角速度：为保证与现有工程口径一致，这里采用 SPP 模块使用的 ωe 数值。
constexpr double kOmegaE = 7.2921151467e-5;

// 相对论改正常数：Δtr = F * e * sqrt(A) * sin(E)。
constexpr double kRelF = -4.442807633e-10;

// 星历有效期阈值：用于输出 eph_expired 标志。
constexpr double kGpsEphValidSec = 7200.0;
constexpr double kBdsEphValidSec = 3600.0;

// 速度计算采用中心差分：用有限差分避免引入新的解析推导口径，从而保持重构前后数值一致性。
constexpr double kVelDiffDtSec = 0.5;

inline bool IsFinite(double x) {
    return std::isfinite(x);
}

double NormalizeWeekCrossover(double dt_sec) {
    // 周跳修正：将时间差限制到 [-302400, +302400]，避免跨周导致 tk/dt 畸变。
    if (dt_sec > kHalfWeekSec) dt_sec -= kWeekSec;
    if (dt_sec < -kHalfWeekSec) dt_sec += kWeekSec;
    return dt_sec;
}

void SolveKeplerE(double Mk, double ecc, double& Ek) {
    // 开普勒方程迭代：E - e*sin(E) = M。
    // 迭代次数与收敛阈值保持与原 SPP 实现一致，以确保结果可复现。
    Ek = Mk;
    for (int i = 0; i < 10; ++i) {
        double f = Ek - ecc * std::sin(Ek) - Mk;
        double fp = 1.0 - ecc * std::cos(Ek);
        double dE = -f / fp;
        Ek += dE;
        if (std::abs(dE) < 1e-12) break;
    }
}

void ApplyEarthRotationCorrection(Vec3& sat_pos_ecef, double transit_time_s) {
    // 地球自转改正（Sagnac）：传播时间内地固系绕 Z 轴旋转，按小角度近似直接旋转卫星坐标。
    double angle = kOmegaE * transit_time_s;
    double x_new = sat_pos_ecef.x * std::cos(angle) + sat_pos_ecef.y * std::sin(angle);
    double y_new = -sat_pos_ecef.x * std::sin(angle) + sat_pos_ecef.y * std::cos(angle);
    sat_pos_ecef.x = x_new;
    sat_pos_ecef.y = y_new;
}

bool ComputeSatelliteClockCommon(double mu,
                                double t_tx,
                                double toc,
                                double toe,
                                double af0,
                                double af1,
                                double af2,
                                double tgd,
                                double rootA,
                                double ecc,
                                double dN,
                                double m0,
                                double& clk_s,
                                double& clk_drift,
                                double& Ek_out) {
    // 卫星钟差模型（广播星历）：
    // clk = af0 + af1*dt + af2*dt^2 + Δtr - TGD，其中 Δtr 为相对论效应改正。
    double dt = NormalizeWeekCrossover(t_tx - toc);
    if (std::abs(dt) > kHalfWeekSec) return false;

    double tk = NormalizeWeekCrossover(t_tx - toe);
    if (std::abs(tk) > kHalfWeekSec) return false;

    double n0 = std::sqrt(mu) / std::pow(rootA, 3);
    double n = n0 + dN;
    double Mk = m0 + n * tk;

    double Ek = Mk;
    SolveKeplerE(Mk, ecc, Ek);
    Ek_out = Ek;

    double relativity = kRelF * ecc * rootA * std::sin(Ek);
    clk_s = af0 + af1 * dt + af2 * dt * dt + relativity - tgd;
    if (!IsFinite(clk_s)) return false;

    double denom = 1.0 - ecc * std::cos(Ek);
    if (std::abs(denom) > 0.0) {
        // 钟速包含相对论效应改正的一阶导数项：Δṫr = F * e * sqrt(A) * cos(E) * Ė。
        double E_dot = n / denom;
        double relativity_dot = kRelF * ecc * rootA * std::cos(Ek) * E_dot;
        clk_drift = af1 + 2.0 * af2 * dt + relativity_dot;
        if (!IsFinite(clk_drift)) clk_drift = 0.0;
    } else {
        clk_drift = 0.0;
    }

    return true;
}

bool ComputeSatellitePositionCommon(double mu,
                                   double t_tx,
                                   double toe,
                                   double rootA,
                                   double ecc,
                                   double inc0,
                                   double inc_dot,
                                   double omega0,
                                   double omega_dot,
                                   double argper,
                                   double m0,
                                   double dN,
                                   double cuc,
                                   double cus,
                                   double crc,
                                   double crs,
                                   double cic,
                                   double cis,
                                   Vec3& sat_pos_ecef,
                                   double& Ek_out) {
    // 广播星历卫星位置计算（按教材/ICD 五步法）：
    // 轨道平面坐标 -> 升交点赤经旋转 -> ECEF。
    double tk = NormalizeWeekCrossover(t_tx - toe);
    if (std::abs(tk) > kHalfWeekSec) return false;

    double n0 = std::sqrt(mu) / std::pow(rootA, 3);
    double n = n0 + dN;
    double Mk = m0 + n * tk;

    double Ek = Mk;
    SolveKeplerE(Mk, ecc, Ek);
    Ek_out = Ek;

    double sinE = std::sin(Ek);
    double cosE = std::cos(Ek);

    // 真近点角使用 atan2 保证象限正确。
    double vk = std::atan2(std::sqrt(1.0 - ecc * ecc) * sinE, cosE - ecc);
    double uk = vk + argper;

    // 二阶谐波摄动改正项：用于补偿轨道参数的周期性误差。
    double sin2u = std::sin(2.0 * uk);
    double cos2u = std::cos(2.0 * uk);
    double du = cus * sin2u + cuc * cos2u;
    double dr = crs * sin2u + crc * cos2u;
    double di = cis * sin2u + cic * cos2u;

    double a = rootA * rootA;
    double r = a * (1.0 - ecc * cosE) + dr;
    double i = inc0 + inc_dot * tk + di;
    uk += du;

    double x_orb = r * std::cos(uk);
    double y_orb = r * std::sin(uk);

    // 升交点赤经：包含地球自转项（-ωe*toe）与 Ωdot 修正。
    double Omega = omega0 + (omega_dot - kOmegaE) * tk - kOmegaE * toe;

    sat_pos_ecef.x = x_orb * std::cos(Omega) - y_orb * std::cos(i) * std::sin(Omega);
    sat_pos_ecef.y = x_orb * std::sin(Omega) + y_orb * std::cos(i) * std::cos(Omega);
    sat_pos_ecef.z = y_orb * std::sin(i);

    if (!IsFinite(sat_pos_ecef.x) || !IsFinite(sat_pos_ecef.y) || !IsFinite(sat_pos_ecef.z)) return false;
    return true;
}

bool ComputeVelocityByCentralDiff(double mu,
                                 double t_tx,
                                 double toe,
                                 double rootA,
                                 double ecc,
                                 double inc0,
                                 double inc_dot,
                                 double omega0,
                                 double omega_dot,
                                 double argper,
                                 double m0,
                                 double dN,
                                 double cuc,
                                 double cus,
                                 double crc,
                                 double crs,
                                 double cic,
                                 double cis,
                                 double transit_time_s,
                                 Vec3& vel_ecef) {
    // 速度采用中心差分近似：v(t) ≈ (p(t+Δt) - p(t-Δt)) / (2Δt)。
    // 该策略用于避免引入解析速度推导后带来的实现口径差异。
    Vec3 p_plus;
    Vec3 p_minus;
    double Ek_dummy = 0.0;

    if (!ComputeSatellitePositionCommon(mu, t_tx + kVelDiffDtSec, toe, rootA, ecc, inc0, inc_dot, omega0, omega_dot,
                                        argper, m0, dN, cuc, cus, crc, crs, cic, cis, p_plus, Ek_dummy)) {
        return false;
    }
    ApplyEarthRotationCorrection(p_plus, transit_time_s);

    if (!ComputeSatellitePositionCommon(mu, t_tx - kVelDiffDtSec, toe, rootA, ecc, inc0, inc_dot, omega0, omega_dot,
                                        argper, m0, dN, cuc, cus, crc, crs, cic, cis, p_minus, Ek_dummy)) {
        return false;
    }
    ApplyEarthRotationCorrection(p_minus, transit_time_s);

    vel_ecef.x = (p_plus.x - p_minus.x) / (2.0 * kVelDiffDtSec);
    vel_ecef.y = (p_plus.y - p_minus.y) / (2.0 * kVelDiffDtSec);
    vel_ecef.z = (p_plus.z - p_minus.z) / (2.0 * kVelDiffDtSec);

    return IsFinite(vel_ecef.x) && IsFinite(vel_ecef.y) && IsFinite(vel_ecef.z);
}

} // namespace

bool ComputeGpsSatState(const GpsEph& eph, double t_gpst, double transit_time_s, SatState& state) {
    state = SatState{};

    // 星历有效性：仅用于输出标志位，调用方可选择是否剔除。
    double tk = NormalizeWeekCrossover(t_gpst - eph.toe);
    state.eph_expired = (std::abs(tk) > kGpsEphValidSec);

    double Ek_clk = 0.0;
    if (!ComputeSatelliteClockCommon(kMuGps, t_gpst, eph.toc, eph.toe, eph.af0, eph.af1, eph.af2, eph.tgd, eph.sqrtA,
                                     eph.e, eph.deltaN, eph.M0, state.clk_s, state.clk_drift, Ek_clk)) {
        return false;
    }

    double Ek_pos = 0.0;
    if (!ComputeSatellitePositionCommon(kMuGps, t_gpst, eph.toe, eph.sqrtA, eph.e, eph.i0, eph.iDot, eph.Omega0,
                                        eph.OmegaDot, eph.omega, eph.M0, eph.deltaN, eph.cuc, eph.cus, eph.crc, eph.crs,
                                        eph.cic, eph.cis, state.pos_ecef, Ek_pos)) {
        return false;
    }
    ApplyEarthRotationCorrection(state.pos_ecef, transit_time_s);

    if (!ComputeVelocityByCentralDiff(kMuGps, t_gpst, eph.toe, eph.sqrtA, eph.e, eph.i0, eph.iDot, eph.Omega0, eph.OmegaDot,
                                      eph.omega, eph.M0, eph.deltaN, eph.cuc, eph.cus, eph.crc, eph.crs, eph.cic, eph.cis,
                                      transit_time_s, state.vel_ecef)) {
        state.vel_ecef = Vec3{};
    }

    state.valid = IsFinite(state.pos_ecef.x) && IsFinite(state.pos_ecef.y) && IsFinite(state.pos_ecef.z) &&
                  IsFinite(state.clk_s);
    return true;
}

bool ComputeBdsSatState(const BdsEph& eph, double t_bdt, double transit_time_s, SatState& state) {
    state = SatState{};

    // 星历有效性：仅用于输出标志位，调用方可选择是否剔除。
    double tk = NormalizeWeekCrossover(t_bdt - eph.toe);
    state.eph_expired = (std::abs(tk) > kBdsEphValidSec);

    double Ek_clk = 0.0;
    if (!ComputeSatelliteClockCommon(kMuBds, t_bdt, eph.toc, eph.toe, eph.af0, eph.af1, eph.af2, eph.tgd, eph.sqrtA, eph.e,
                                     eph.deltaN, eph.M0, state.clk_s, state.clk_drift, Ek_clk)) {
        return false;
    }

    double Ek_pos = 0.0;
    if (!ComputeSatellitePositionCommon(kMuBds, t_bdt, eph.toe, eph.sqrtA, eph.e, eph.i0, eph.iDot, eph.Omega0, eph.OmegaDot,
                                        eph.omega, eph.M0, eph.deltaN, eph.cuc, eph.cus, eph.crc, eph.crs, eph.cic, eph.cis,
                                        state.pos_ecef, Ek_pos)) {
        return false;
    }
    ApplyEarthRotationCorrection(state.pos_ecef, transit_time_s);

    if (!ComputeVelocityByCentralDiff(kMuBds, t_bdt, eph.toe, eph.sqrtA, eph.e, eph.i0, eph.iDot, eph.Omega0, eph.OmegaDot,
                                      eph.omega, eph.M0, eph.deltaN, eph.cuc, eph.cus, eph.crc, eph.crs, eph.cic, eph.cis,
                                      transit_time_s, state.vel_ecef)) {
        state.vel_ecef = Vec3{};
    }

    state.valid = IsFinite(state.pos_ecef.x) && IsFinite(state.pos_ecef.y) && IsFinite(state.pos_ecef.z) &&
                  IsFinite(state.clk_s);
    return true;
}

} // namespace gnss
