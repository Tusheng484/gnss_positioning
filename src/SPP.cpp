#include "gnss_positioning/SPP.h"
#include<set>
#include <map>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace gnss {

static double deg2rad(double deg) { return deg * M_PI / 180.0; }

Ephemeris ConvertFromGpsEphem(const GpsEphem& gps_eph) {
    // 工程约定：在 main.cpp 层面缓存的是解码器原始结构体，这里提供统一转换函数，避免在 main.cpp 中掺杂定位领域字段映射细节
    Ephemeris eph;
    eph.system = GnssSystem::GPS;
    eph.prn = gps_eph.prn;
    eph.toe = gps_eph.toe;
    eph.toc = gps_eph.toc;
    eph.af0 = gps_eph.af0;
    eph.af1 = gps_eph.af1;
    eph.af2 = gps_eph.af2;
    eph.tgd = gps_eph.tgd;
    eph.rootA = std::sqrt(gps_eph.majaxis);
    eph.ecc = gps_eph.ecc;
    eph.inc0 = gps_eph.inc;
    eph.inc_dot = gps_eph.inc_dot;
    eph.omega0 = gps_eph.omegazero;
    eph.omega_dot = gps_eph.omegadot;
    eph.argper = gps_eph.argper;
    eph.m0 = gps_eph.ma;
    eph.dN = gps_eph.dN;
    eph.cuc = gps_eph.cuc;
    eph.cus = gps_eph.cus;
    eph.crc = gps_eph.crc;
    eph.crs = gps_eph.crs;
    eph.cic = gps_eph.cic;
    eph.cis = gps_eph.cis;
    return eph;
}

Ephemeris ConvertFromBdsEphem(const BdsEphem& bds_eph) {
    // 注意：BDS 星历的 rootA 已为 sqrt(a)，tgd 取 TGD1（B1I）
    Ephemeris eph;
    eph.system = GnssSystem::BDS;
    eph.prn = bds_eph.prn;
    eph.toe = bds_eph.toe;
    eph.toc = bds_eph.toc;
    eph.af0 = bds_eph.af0;
    eph.af1 = bds_eph.af1;
    eph.af2 = bds_eph.af2;
    eph.tgd = bds_eph.tgd1;
    eph.rootA = bds_eph.rootA;
    eph.ecc = bds_eph.ecc;
    eph.inc0 = bds_eph.i0;
    eph.inc_dot = bds_eph.idot;
    eph.omega0 = bds_eph.omegazero;
    eph.omega_dot = bds_eph.omegadot;
    eph.argper = bds_eph.omega;
    eph.m0 = bds_eph.m0;
    eph.dN = bds_eph.dN;
    eph.cuc = bds_eph.cuc;
    eph.cus = bds_eph.cus;
    eph.crc = bds_eph.crc;
    eph.crs = bds_eph.crs;
    eph.cic = bds_eph.cic;
    eph.cis = bds_eph.cis;
    return eph;
}

SPP::SPP() : max_iter_(10), conv_m_(1e-4), elev_mask_deg_(10.0), snr_mask_dbhz_(30.0), enable_iono_free_(true), verbose_(false) {
    // 初始值策略（工程经验）：
    // - 初始坐标给一个地表附近的合理值，避免从地心 (0,0,0) 出发导致几何矩阵病态
    // - 初始钟差/系统间偏差置零，由最小二乘迭代收敛
    sol_.pos.x = -2267749.0;
    sol_.pos.y = 5009355.0;
    sol_.pos.z = 3221234.0;
    sol_.clk_gps_m = 0.0;
    sol_.isb_bds_m = 0.0;
}

std::vector<double> SPP::getPosition() const {
    return {sol_.pos.x, sol_.pos.y, sol_.pos.z};
}

double SPP::getReceiverClockBias() const {
    return sol_.clk_gps_m;
}

double SPP::normalizeTimeSec(double dt) {
    // 周跳修正：将时间差限制到 [-302400, +302400]，避免跨周导致的 tk/dt 畸变
    if (dt > SppConst::HALF_WEEK) dt -= SppConst::WEEK_SEC;
    if (dt < -SppConst::HALF_WEEK) dt += SppConst::WEEK_SEC;
    return dt;
}

uint32_t SPP::makeKey(GnssSystem sys, uint32_t prn) {
    return static_cast<uint32_t>(sys) * 1000U + prn;
}

Ephemeris SPP::normalizeEphemerisToGpSt(const Ephemeris& eph) const {
    Ephemeris out = eph;
    if (out.system == GnssSystem::BDS) {
        // 时间系统统一（核心约束）：
        // - 解算内部统一使用 GPST
        // - BDS 广播星历 toe/toc 若为 BDT，则转换到 GPST：t_gps = t_bds + 14s
        // - 禁止在“发射时刻 t_tx”阶段做系统时间偏移；系统间差异在观测方程中由 ISB 建模
        out.toe += SppConst::BDT_GPS_OFFSET;
        out.toc += SppConst::BDT_GPS_OFFSET;
        while (out.toe >= SppConst::WEEK_SEC) out.toe -= SppConst::WEEK_SEC;
        while (out.toe < 0.0) out.toe += SppConst::WEEK_SEC;
        while (out.toc >= SppConst::WEEK_SEC) out.toc -= SppConst::WEEK_SEC;
        while (out.toc < 0.0) out.toc += SppConst::WEEK_SEC;
    }
    return out;
}

std::vector<SPP::ProcObs> SPP::preprocessObservations(const std::vector<ObsData>& obsList) const {
    // 五步法-第1步：观测预处理
    // - 选取单星最佳观测（同一频点取最高 SNR）
    // - SNR 掩码剔除低质量观测
    // - 支持双频无电离层组合（优先），以抑制电离层误差导致的高度方向漂移
    struct FreqObs {
        bool has0 = false;
        bool has1 = false;
        double P0 = 0.0;
        double P1 = 0.0;
        double snr0 = 0.0;
        double snr1 = 0.0;
        GnssSystem sys = GnssSystem::UNKNOWN;
        uint32_t prn = 0;
    };

    std::map<uint32_t, FreqObs> m;
    for (const auto& obs : obsList) {
        if (obs.pseudorange <= 0.0) continue;
        if (obs.snr < static_cast<float>(snr_mask_dbhz_)) continue;
        if (obs.freqIdx != 0 && obs.freqIdx != 1) continue;

        uint32_t k = makeKey(obs.system, obs.prn);
        auto& fo = m[k];
        fo.sys = obs.system;
        fo.prn = obs.prn;

        if (obs.freqIdx == 0) {
            if (!fo.has0 || obs.snr > fo.snr0) {
                fo.has0 = true;
                fo.P0 = obs.pseudorange;
                fo.snr0 = obs.snr;
            }
        } else {
            if (!fo.has1 || obs.snr > fo.snr1) {
                fo.has1 = true;
                fo.P1 = obs.pseudorange;
                fo.snr1 = obs.snr;
            }
        }
    }

    std::vector<ProcObs> out;
    out.reserve(m.size());

    for (const auto& kv : m) {
        const auto& fo = kv.second;
        if (!fo.has0) continue;

        ProcObs po;
        po.system = fo.sys;
        po.prn = fo.prn;
        po.snr = std::max(fo.snr0, fo.snr1);
        po.iono_free = false;
        po.P = fo.P0;

        if (enable_iono_free_ && fo.has1) {
            // 无电离层组合伪距（Iono-Free）：
            // P_IF = (f1^2*P1 - f2^2*P2) / (f1^2 - f2^2)
            double f1 = 0.0;
            double f2 = 0.0;
            if (fo.sys == GnssSystem::GPS) {
                f1 = SppConst::GPS_L1;
                f2 = SppConst::GPS_L2;
            } else if (fo.sys == GnssSystem::BDS) {
                f1 = SppConst::BDS_B1I;
                f2 = SppConst::BDS_B3I;
            }

            if (f1 > 0.0 && f2 > 0.0) {
                double f1_2 = f1 * f1;
                double f2_2 = f2 * f2;
                double denom = (f1_2 - f2_2);
                if (std::abs(denom) > 1e-6) {
                    po.P = (f1_2 * fo.P0 - f2_2 * fo.P1) / denom;
                    po.iono_free = true;
                }
            }
        }

        out.push_back(po);
    }

    return out;
}

double SPP::computeTransmitTime(double t_rx_gpst, double pseudorange_m) const {
    // 五步法-第2步：计算信号发射时刻
    // t_tx = t_rx - ρ/c
    // 这里的 ρ 用观测伪距近似（工程常用的初值），随后会用卫星钟差对 t_tx 做一次迭代修正
    return t_rx_gpst - pseudorange_m / SppConst::C;
}

bool SPP::computeSatelliteClock(const Ephemeris& eph_gpst, double t_tx_gpst, double& clk_s) const {
    // 五步法-第3步（部分）：广播星历卫星钟差计算（独立函数）
    // dt = t_tx - toc
    // clk = af0 + af1*dt + af2*dt^2 + relativity - TGD
    // relativity = F * e * sqrtA * sin(Ek)
    double dt = normalizeTimeSec(t_tx_gpst - eph_gpst.toc);
    if (std::abs(dt) > SppConst::HALF_WEEK) return false;

    double tk = normalizeTimeSec(t_tx_gpst - eph_gpst.toe);
    if (std::abs(tk) > SppConst::HALF_WEEK) return false;

    double mu = (eph_gpst.system == GnssSystem::GPS) ? SppConst::MU_GPS : SppConst::MU_BDS;
    double n0 = std::sqrt(mu) / std::pow(eph_gpst.rootA, 3);
    double n = n0 + eph_gpst.dN;
    double Mk = eph_gpst.m0 + n * tk;

    double Ek = Mk;
    for (int i = 0; i < 10; ++i) {
        double f = Ek - eph_gpst.ecc * std::sin(Ek) - Mk;
        double fp = 1.0 - eph_gpst.ecc * std::cos(Ek);
        double dE = -f / fp;
        Ek += dE;
        if (std::abs(dE) < 1e-12) break;
    }

    double relativity = SppConst::F_REL * eph_gpst.ecc * eph_gpst.rootA * std::sin(Ek);
    clk_s = eph_gpst.af0 + eph_gpst.af1 * dt + eph_gpst.af2 * dt * dt + relativity - eph_gpst.tgd;

    if (!std::isfinite(clk_s)) return false;
    return true;
}

bool SPP::computeSatellitePosition(const Ephemeris& eph_gpst, double t_tx_gpst, XYZ& sat_pos_ecef, double& Ek_out) const {
    // 五步法-第3步（部分）：广播星历卫星位置计算（严格按教材顺序）
    // 1) tk = t_tx - toe（含周跳修正）
    // 2) Mk = M0 + (n0+Δn)*tk
    // 3) 解开普勒方程 Ek
    // 4) vk、uk
    // 5) du/dr/di
    // 6) r、i
    // 7) Ωk
    // 8) 轨道平面 -> ECEF
    double tk = normalizeTimeSec(t_tx_gpst - eph_gpst.toe);
    if (std::abs(tk) > SppConst::HALF_WEEK) return false;

    double mu = (eph_gpst.system == GnssSystem::GPS) ? SppConst::MU_GPS : SppConst::MU_BDS;
    double n0 = std::sqrt(mu) / std::pow(eph_gpst.rootA, 3);
    double n = n0 + eph_gpst.dN;
    double Mk = eph_gpst.m0 + n * tk;

    double Ek = Mk;
    for (int i = 0; i < 10; ++i) {
        double f = Ek - eph_gpst.ecc * std::sin(Ek) - Mk;
        double fp = 1.0 - eph_gpst.ecc * std::cos(Ek);
        double dE = -f / fp;
        Ek += dE;
        if (std::abs(dE) < 1e-12) break;
    }
    Ek_out = Ek;

    double sinE = std::sin(Ek);
    double cosE = std::cos(Ek);

    double vk = std::atan2(std::sqrt(1.0 - eph_gpst.ecc * eph_gpst.ecc) * sinE, cosE - eph_gpst.ecc);
    double uk = vk + eph_gpst.argper;

    double sin2u = std::sin(2.0 * uk);
    double cos2u = std::cos(2.0 * uk);
    double du = eph_gpst.cus * sin2u + eph_gpst.cuc * cos2u;
    double dr = eph_gpst.crs * sin2u + eph_gpst.crc * cos2u;
    double di = eph_gpst.cis * sin2u + eph_gpst.cic * cos2u;

    double a = eph_gpst.rootA * eph_gpst.rootA;
    double r = a * (1.0 - eph_gpst.ecc * cosE) + dr;
    double i = eph_gpst.inc0 + eph_gpst.inc_dot * tk + di;
    uk += du;

    double x_orb = r * std::cos(uk);
    double y_orb = r * std::sin(uk);

    double Omega = eph_gpst.omega0 + (eph_gpst.omega_dot - SppConst::OMEGA_E) * tk - SppConst::OMEGA_E * eph_gpst.toe;

    sat_pos_ecef.x = x_orb * std::cos(Omega) - y_orb * std::cos(i) * std::sin(Omega);
    sat_pos_ecef.y = x_orb * std::sin(Omega) + y_orb * std::cos(i) * std::cos(Omega);
    sat_pos_ecef.z = y_orb * std::sin(i);

    if (!std::isfinite(sat_pos_ecef.x) || !std::isfinite(sat_pos_ecef.y) || !std::isfinite(sat_pos_ecef.z)) return false;
    return true;
}

void SPP::applyEarthRotationCorrection(XYZ& sat_pos_ecef, double transit_time_s) const {
    // Sagnac 效应（地球自转改正）：
    // 将卫星在传播时间内的地球自转影响折算到 ECEF 坐标中（绕 Z 轴旋转）
    double angle = SppConst::OMEGA_E * transit_time_s;
    double x_new = sat_pos_ecef.x * std::cos(angle) + sat_pos_ecef.y * std::sin(angle);
    double y_new = -sat_pos_ecef.x * std::sin(angle) + sat_pos_ecef.y * std::cos(angle);
    sat_pos_ecef.x = x_new;
    sat_pos_ecef.y = y_new;
}

double SPP::computeTroposphereSaastamoinen(const BLH& rx_blh, double elev_rad) const {
    // 五步法-第4步：对流层改正 T（Saastamoinen 简化模型）
    // 工程假设：标准大气 + 相对湿度固定，映射函数近似 1/sin(elev)
    if (!(elev_rad > 0.0)) return 0.0;
    double h = rx_blh.height;
    if (!std::isfinite(h)) h = 0.0;
    if (h < -100.0) h = -100.0;
    if (h > 10000.0) h = 10000.0;

    double P0 = 1013.25;
    double T0 = 293.15;
    double P = P0 * std::pow(1.0 - 2.26e-5 * h, 5.225);
    double T = T0 - 6.5e-3 * h;
    double RH = 0.5;
    double Tc = T - 273.15;
    double e = RH * 6.11 * std::exp((17.15 * Tc) / (234.7 + Tc));

    double sinE = std::sin(std::max(elev_rad, deg2rad(5.0)));
    double zhd = 0.0022768 * P;
    double zwd = 0.002277 * (1255.0 / T + 0.05) * e;
    return (zhd + zwd) / sinE;
}

bool SPP::buildObservationEquation(double t_rx_gpst,
                                  const std::vector<ProcObs>& procObs,
                                  const std::vector<Ephemeris>& ephList,
                                  const XYZ& rx_pos,
                                  double clk_gps_m,
                                  double isb_bds_m,
                                  std::vector<SatCalc>& satCalcs) const {
    // 五步法-第3/4步：对每颗卫星计算 (t_tx, sat_pos, sat_clk) 并形成观测方程需要的中间量
    // 伪距方程（按教材形式）：
    // P = ρ + c(dt_r - dt_s) + T
    // 变形为：
    // P + c*dt_s = ρ + c*dt_r + T (+ ISB_BDS)
    std::map<uint32_t, Ephemeris> ephMap;
    ephMap.clear();
    for (const auto& eph : ephList) {
        Ephemeris eph_gpst = normalizeEphemerisToGpSt(eph);
        ephMap[makeKey(eph_gpst.system, eph_gpst.prn)] = eph_gpst;
    }

    BLH rx_blh;
    if (XYZToBLH(rx_pos, rx_blh) != 0) return false;

    double elev_mask_rad = deg2rad(elev_mask_deg_);

    satCalcs.clear();
    satCalcs.reserve(procObs.size());

    for (const auto& o : procObs) {
        SatCalc sc;
        sc.system = o.system;
        sc.prn = o.prn;
        sc.valid = false;

        auto it = ephMap.find(makeKey(o.system, o.prn));
        if (it == ephMap.end()) continue;
        const auto& eph = it->second;

        double t_tx = computeTransmitTime(t_rx_gpst, o.P);

        double clk_s = 0.0;
        if (!computeSatelliteClock(eph, t_tx, clk_s)) continue;
        // 发射时刻二次修正（工程级做法）：
        // t_tx = t_rx - P/c - dt_s
        t_tx = t_rx_gpst - (o.P / SppConst::C) - clk_s;
        if (!computeSatelliteClock(eph, t_tx, clk_s)) continue;

        XYZ sat_pos;
        double Ek_dummy = 0.0;
        if (!computeSatellitePosition(eph, t_tx, sat_pos, Ek_dummy)) continue;

        applyEarthRotationCorrection(sat_pos, o.P / SppConst::C);

        double dx = sat_pos.x - rx_pos.x;
        double dy = sat_pos.y - rx_pos.y;
        double dz = sat_pos.z - rx_pos.z;
        double rho = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (!(rho > 1.0)) continue;

        XYZ los;
        los.x = dx / rho;
        los.y = dy / rho;
        los.z = dz / rho;

        double E, N, U;
        if (XyzToEnu(rx_pos, sat_pos, E, N, U) != 0) continue;
        double elev = std::atan2(U, std::sqrt(E * E + N * N));
        if (!std::isfinite(elev) || elev < elev_mask_rad) continue;

        double tropo = computeTroposphereSaastamoinen(rx_blh, elev);

        double P_corr = o.P + SppConst::C * clk_s;

        double bias_m = clk_gps_m;
        if (o.system == GnssSystem::BDS) bias_m += isb_bds_m;

        double resid = P_corr - (rho + tropo + bias_m);

        sc.pos_ecef = sat_pos;
        sc.clk_s = clk_s;
        sc.elev_rad = elev;
        sc.los = los;
        sc.rho = rho;
        sc.tropo_m = tropo;
        sc.P_corr = P_corr;
        sc.valid = true;

        satCalcs.push_back(sc);

        if (verbose_) {
            std::cout << "  [SAT] " << (o.system == GnssSystem::GPS ? "G" : "C") << o.prn
                      << " elev=" << elev * 180.0 / M_PI
                      << " clk_s=" << clk_s
                      << " rho=" << rho
                      << " tropo=" << tropo
                      << " prefit=" << resid
                      << std::endl;
        }
    }

    return !satCalcs.empty();
}

bool SPP::solveLeastSquares(const std::vector<SatCalc>& sats,
                            Eigen::VectorXd& dx,
                            Eigen::VectorXd& v,
                            Eigen::MatrixXd& H) const {
    // 五步法-第5步：最小二乘迭代求解
    // 状态量：
    // - GPS+BDS：x = [X,Y,Z, c*dt_r, ISB]^T
    // - 仅 GPS： x = [X,Y,Z, c*dt_r]^T
    // 权阵：w = sin^2(elevation)
    int m = static_cast<int>(sats.size());
    bool hasBds = false;
    for (const auto& s : sats) {
        if (s.system == GnssSystem::BDS) { hasBds = true; break; }
    }
    int n = hasBds ? 5 : 4;

    H.resize(m, n);
    v.resize(m);

    for (int i = 0; i < m; ++i) {
        const auto& s = sats[i];

        double clk_term_m = sol_.clk_gps_m;
        double isb_term_m = sol_.isb_bds_m;
        double bias_m = clk_term_m + ((s.system == GnssSystem::BDS) ? isb_term_m : 0.0);

        double resid = s.P_corr - (s.rho + s.tropo_m + bias_m);
        v(i) = resid;

        double w = std::sin(std::max(s.elev_rad, deg2rad(5.0)));
        w = w * w;
        if (w < 0.01) w = 0.01;

        H(i, 0) = -s.los.x * w;
        H(i, 1) = -s.los.y * w;
        H(i, 2) = -s.los.z * w;
        H(i, 3) = 1.0 * w;
        if (n == 5) {
            H(i, 4) = (s.system == GnssSystem::BDS) ? 1.0 * w : 0.0;
        }
        v(i) *= w;
    }

    dx = H.colPivHouseholderQr().solve(v);
    return dx.allFinite();
}

bool SPP::compute(double t_rx_gpst, const std::vector<ObsData>& obsList, const std::vector<Ephemeris>& ephList) {
    // 五步法整体驱动函数：
    // 1) preprocessObservations
    // 2) computeTransmitTime
    // 3) computeSatellitePosition / computeSatelliteClock
    // 4) buildObservationEquation
    // 5) solveLeastSquares（迭代更新）
    std::vector<ProcObs> procObs = preprocessObservations(obsList);
    if (procObs.size() < 4) return false;

    XYZ rx_pos = sol_.pos;
    double clk_gps_m = sol_.clk_gps_m;
    double isb_bds_m = sol_.isb_bds_m;

    for (int iter = 0; iter < max_iter_; ++iter) {
        std::vector<SatCalc> sats;
        if (!buildObservationEquation(t_rx_gpst, procObs, ephList, rx_pos, clk_gps_m, isb_bds_m, sats)) {
            return false;
        }

        if (iter > 0) {
            // 异常值剔除（工程约束）：|残差| > 100m 的观测剔除
            std::vector<SatCalc> filtered;
            filtered.reserve(sats.size());
            for (const auto& s : sats) {
                double bias_m = clk_gps_m + ((s.system == GnssSystem::BDS) ? isb_bds_m : 0.0);
                double resid = s.P_corr - (s.rho + s.tropo_m + bias_m);
                if (std::abs(resid) <= 100.0) filtered.push_back(s);
            }
            sats.swap(filtered);
        }

        Eigen::VectorXd dx;
        Eigen::VectorXd v;
        Eigen::MatrixXd H;
        if (!solveLeastSquares(sats, dx, v, H)) {
            return false;
        }

        if (dx.size() == 4) {
            rx_pos.x += dx(0);
            rx_pos.y += dx(1);
            rx_pos.z += dx(2);
            clk_gps_m += dx(3);
        } else {
            rx_pos.x += dx(0);
            rx_pos.y += dx(1);
            rx_pos.z += dx(2);
            clk_gps_m += dx(3);
            isb_bds_m += dx(4);
        }

        double dpos = std::sqrt(dx(0) * dx(0) + dx(1) * dx(1) + dx(2) * dx(2));
        if (verbose_) {
            std::cout << "  [ITER] " << iter + 1
                      << " dpos=" << dpos
                      << " clk_gps_m=" << clk_gps_m
                      << " isb_bds_m=" << isb_bds_m
                      << " used=" << sats.size()
                      << std::endl;
        }

        if (dpos < conv_m_) {
            sol_.pos = rx_pos;
            sol_.clk_gps_m = clk_gps_m;
            sol_.isb_bds_m = isb_bds_m;
            return true;
        }
    }

    sol_.pos = rx_pos;
    sol_.clk_gps_m = clk_gps_m;
    sol_.isb_bds_m = isb_bds_m;
    return false;
}

bool SPP::computeWithReport(double t_rx_gpst, const std::vector<ObsData>& obsList, const std::vector<Ephemeris>& ephList, SppEpochReport& report) {
    // 工程化定位入口：对每个 RANGE 历元输出一行报告，用于落盘统计与问题回溯
    report = SppEpochReport{};
    report.tow_gpst = t_rx_gpst;

    auto makeKeyLocal = [](GnssSystem sys, uint32_t prn) -> uint32_t {
        return static_cast<uint32_t>(sys) * 1000U + prn;
    };

    std::map<uint32_t, Ephemeris> ephMap;
    int eph_g = 0;
    int eph_b = 0;
    for (const auto& eph : ephList) {
        uint32_t k = makeKeyLocal(eph.system, eph.prn);
        if (ephMap.emplace(k, eph).second) {
            if (eph.system == GnssSystem::GPS) eph_g++;
            else if (eph.system == GnssSystem::BDS) eph_b++;
        }
    }
    report.eph_g = eph_g;
    report.eph_b = eph_b;

    std::set<uint32_t> gpsObsPrn;
    std::set<uint32_t> bdsObsPrn;
    for (const auto& obs : obsList) {
        if (obs.pseudorange <= 0.0) continue;
        if (obs.freqIdx != 0) continue;
        if (obs.system == GnssSystem::GPS) gpsObsPrn.insert(obs.prn);
        else if (obs.system == GnssSystem::BDS) bdsObsPrn.insert(obs.prn);
    }
    report.obs_g = static_cast<int>(gpsObsPrn.size());
    report.obs_b = static_cast<int>(bdsObsPrn.size());

    int match_g = 0;
    int match_b = 0;
    for (auto prn : gpsObsPrn) {
        if (ephMap.count(makeKeyLocal(GnssSystem::GPS, prn))) match_g++;
    }
    for (auto prn : bdsObsPrn) {
        if (ephMap.count(makeKeyLocal(GnssSystem::BDS, prn))) match_b++;
    }
    report.match_g = match_g;
    report.match_b = match_b;

    int ephTotal = report.eph_g + report.eph_b;
    int matchTotal = report.match_g + report.match_b;

    if (ephTotal == 0) {
        report.attempted = false;
        report.ok = false;
        double nan = std::numeric_limits<double>::quiet_NaN();
        report.pos = {nan, nan, nan};
        report.clk_gps_m = nan;
        report.isb_bds_m = nan;
        return false;
    }

    int required = (report.match_b > 0) ? 5 : 4;
    if (matchTotal < required) {
        report.attempted = false;
        report.ok = false;
        double nan = std::numeric_limits<double>::quiet_NaN();
        report.pos = {nan, nan, nan};
        report.clk_gps_m = nan;
        report.isb_bds_m = nan;
        return false;
    }

    report.attempted = true;
    report.ok = compute(t_rx_gpst, obsList, ephList);

    if (report.ok) {
        report.pos = sol_.pos;
        report.clk_gps_m = sol_.clk_gps_m;
        report.isb_bds_m = sol_.isb_bds_m;
        return true;
    }

    double nan = std::numeric_limits<double>::quiet_NaN();
    report.pos = {nan, nan, nan};
    report.clk_gps_m = nan;
    report.isb_bds_m = nan;
    return false;
}

} // namespace gnss
