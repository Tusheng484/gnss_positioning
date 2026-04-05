/**
 * @file ErrorCorrection.cpp
 * @brief SPP 误差改正模块实现：Hopfield 对流层 + Klobuchar 电离层 + 双频组合
 * @author Jiaoyang Wu
 * @date 2026-03-30
 */

#include "gnss_positioning/ErrorCorrection.h"

#include <cmath>
#include <limits>

namespace gnss {
namespace {

double Clamp(double x, double lo, double hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

double NormalizeSecOfDay(double t_sec) {
    double out = t_sec;
    while (out >= 86400.0) out -= 86400.0;
    while (out < 0.0) out += 86400.0;
    return out;
}

bool AllZero4(const double a[4]) {
    return a[0] == 0.0 && a[1] == 0.0 && a[2] == 0.0 && a[3] == 0.0;
}

double KlobucharDelayMeters(double tow,
                            const double pos[3],
                            double az,
                            double el,
                            const double alpha[4],
                            const double beta[4]) {
    if (!pos || !alpha || !beta) return 0.0;
    if (!(el > 0.0)) return 0.0;
    if (AllZero4(alpha) && AllZero4(beta)) return 0.0;

    double phi_u_sc = pos[0] / ErrorConst::PI;
    double lam_u_sc = pos[1] / ErrorConst::PI;
    double el_sc = el / ErrorConst::PI;

    double psi = 0.0137 / (el_sc + 0.11) - 0.022;

    double phi_i = phi_u_sc + psi * std::cos(az);
    phi_i = Clamp(phi_i, -0.416, 0.416);

    double lam_i = lam_u_sc + psi * std::sin(az) / std::cos(phi_i * ErrorConst::PI);

    double phi_m = phi_i + 0.064 * std::cos((lam_i - 1.617) * ErrorConst::PI);

    double t = 43200.0 * lam_i + tow;
    t = NormalizeSecOfDay(t);

    double F = 1.0 + 16.0 * std::pow(0.53 - el_sc, 3);

    double PER = beta[0] + beta[1] * phi_m + beta[2] * phi_m * phi_m + beta[3] * phi_m * phi_m * phi_m;
    if (PER < 72000.0) PER = 72000.0;

    double AMP = alpha[0] + alpha[1] * phi_m + alpha[2] * phi_m * phi_m + alpha[3] * phi_m * phi_m * phi_m;
    if (AMP < 0.0) AMP = 0.0;

    double x = 2.0 * ErrorConst::PI * (t - 50400.0) / PER;

    double iono_sec = 0.0;
    if (std::abs(x) < 1.57) {
        double x2 = x * x;
        double x4 = x2 * x2;
        iono_sec = F * (5e-9 + AMP * (1.0 - x2 / 2.0 + x4 / 24.0));
    } else {
        iono_sec = F * 5e-9;
    }

    if (!std::isfinite(iono_sec) || iono_sec < 0.0) return 0.0;
    return iono_sec * ErrorConst::SPEED_OF_LIGHT;
}

} // namespace

double HopfieldTropDelay(double H, double E, double T0, double p0, double RH0) {
    if (!std::isfinite(H) || !std::isfinite(E) || !std::isfinite(T0) || !std::isfinite(p0) || !std::isfinite(RH0)) {
        return 0.0;
    }

    double el = E;
    double el_min = 5.0 * ErrorConst::PI / 180.0;
    if (el < el_min) el = el_min;

    double Hw = 11000.0;
    double Hd = 40136.0 + 148.72 * (T0 - 273.16);

    if (H < -1000.0 || H > 10000.0) H = 0.0;
    if (H >= Hd) return 0.0;

    double T = T0 - 0.0065 * H;
    if (!(T > 0.0)) return 0.0;

    double p = p0 * std::pow(1.0 - 0.0000226 * H, 5.225);

    double RH = Clamp(RH0 / 100.0, 0.0, 1.0);
    double e = RH * std::exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);

    double Kd = 155.2e-7 * (p / T) * (Hd - H);
    double Kw = 155.2e-7 * (4810.0 * e / (T * T)) * (Hw - H);
    if (H >= Hw) Kw = 0.0;

    double el_deg = el * 180.0 / ErrorConst::PI;
    double sin_d = std::sin(std::sqrt(el_deg * el_deg + 6.25) * ErrorConst::PI / 180.0);
    double sin_w = std::sin(std::sqrt(el_deg * el_deg + 2.25) * ErrorConst::PI / 180.0);
    if (sin_d < 0.1) sin_d = 0.1;
    if (sin_w < 0.1) sin_w = 0.1;
    double md = 1.0 / sin_d;
    double mw = 1.0 / sin_w;

    double trop = Kd * md + Kw * mw;
    if (!std::isfinite(trop) || trop < 0.0 || trop > 50.0) {
        return 2.3 / std::sin(el);
    }
    return trop;
}

double GPSKlobucharIonoDelay(double tow,
                             const double pos[3],
                             double az,
                             double el,
                             const double alpha[4],
                             const double beta[4]) {
    return KlobucharDelayMeters(tow, pos, az, el, alpha, beta);
}

double BDSKlobucharIonoDelay(double tow,
                             const double pos[3],
                             double az,
                             double el,
                             const double alpha[4],
                             const double beta[4]) {
    return KlobucharDelayMeters(tow, pos, az, el, alpha, beta);
}

double IonFreeCombo(double P1, double P2, double f1, double f2) {
    if (!std::isfinite(P1) || !std::isfinite(P2) || !(f1 > 0.0) || !(f2 > 0.0)) return P1;
    double f1_2 = f1 * f1;
    double f2_2 = f2 * f2;
    double denom = f1_2 - f2_2;
    if (std::abs(denom) < std::numeric_limits<double>::epsilon()) return P1;
    return (f1_2 * P1 - f2_2 * P2) / denom;
}

double IonoL1ToFreq(double ionoL1, double f1, double f2) {
    if (!std::isfinite(ionoL1) || !(f1 > 0.0) || !(f2 > 0.0)) return 0.0;
    double ratio = f1 / f2;
    return ionoL1 * ratio * ratio;
}

} // namespace gnss
