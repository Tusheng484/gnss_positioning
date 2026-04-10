/**
 * @file SPP.cpp
 * @brief 标准单点定位（SPP）实现：观测预处理、广播星历解算与最小二乘迭代
 * @author Jiaoyang Wu
 * @date 2026-03-25
 */

#include "gnss_positioning/SPP.h"
#include "gnss_positioning/SatPosCalc.h"
#include "gnss_positioning/ErrorCorrection.h"
#include <set>
#include <map>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <limits>

#ifndef M_PI
// 兼容性处理：部分编译环境未提供 M_PI 宏定义。
#define M_PI 3.14159265358979323846
#endif

namespace gnss {

// 角度转弧度。
static double deg2rad(double deg) { return deg * M_PI / 180.0; }

static double normalizeTowSec(double tow) {
    double out = tow;
    while (out >= SppConst::WEEK_SEC) out -= SppConst::WEEK_SEC;
    while (out < 0.0) out += SppConst::WEEK_SEC;
    return out;
}

static double gpstToBdtSec(double t_gpst) {
    return normalizeTowSec(t_gpst - SppConst::BDT_GPS_OFFSET);
}

// 将 SPP 统一星历结构体映射为卫星状态计算模块的输入格式。
// 该映射仅做字段搬运，不引入任何单位换算，以保持前后计算口径一致。
static GpsEph ToGpsEph(const Ephemeris& eph) {
    GpsEph out;
    out.prn = eph.prn;
    out.toc = eph.toc;
    out.toe = eph.toe;
    out.af0 = eph.af0;
    out.af1 = eph.af1;
    out.af2 = eph.af2;
    out.tgd = eph.tgd;
    out.sqrtA = eph.rootA;
    out.e = eph.ecc;
    out.i0 = eph.inc0;
    out.iDot = eph.inc_dot;
    out.Omega0 = eph.omega0;
    out.OmegaDot = eph.omega_dot;
    out.omega = eph.argper;
    out.M0 = eph.m0;
    out.deltaN = eph.dN;
    out.cuc = eph.cuc;
    out.cus = eph.cus;
    out.crc = eph.crc;
    out.crs = eph.crs;
    out.cic = eph.cic;
    out.cis = eph.cis;
    return out;
}

// 将 SPP 统一星历结构体映射为卫星状态计算模块的输入格式。
static BdsEph ToBdsEph(const Ephemeris& eph) {
    BdsEph out;
    out.prn = eph.prn;
    out.toc = eph.toc;
    out.toe = eph.toe;
    out.af0 = eph.af0;
    out.af1 = eph.af1;
    out.af2 = eph.af2;
    out.tgd = eph.tgd;
    out.sqrtA = eph.rootA;
    out.e = eph.ecc;
    out.i0 = eph.inc0;
    out.iDot = eph.inc_dot;
    out.Omega0 = eph.omega0;
    out.OmegaDot = eph.omega_dot;
    out.omega = eph.argper;
    out.M0 = eph.m0;
    out.deltaN = eph.dN;
    out.cuc = eph.cuc;
    out.cus = eph.cus;
    out.crc = eph.crc;
    out.crs = eph.crs;
    out.cic = eph.cic;
    out.cis = eph.cis;
    return out;
}

Ephemeris ConvertFromGpsEphem(const GpsEphem& gps_eph) {
    // 工程约束：解码器输出结构体作为“输入数据模型”在应用层缓存；字段映射在此集中处理，避免应用层掺入定位领域细节。
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
    // BDS 字段口径说明：
    // - rootA 为 sqrt(a)（与 GPS 的 majaxis 不同）
    // - tgd 取 TGD1（B1I 频点）
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
    sol_.pos.x = 0.0;
    sol_.pos.y = 0.0;
    sol_.pos.z = 0.0;
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
    // 周界归一化：将时间差限制到 [-302400, +302400]，避免跨周引起 tk/dt 量级突变并污染轨道与钟差计算。
    if (dt > SppConst::HALF_WEEK) dt -= SppConst::WEEK_SEC;
    if (dt < -SppConst::HALF_WEEK) dt += SppConst::WEEK_SEC;
    return dt;
}

uint32_t SPP::makeKey(GnssSystem sys, uint32_t prn) {
    return static_cast<uint32_t>(sys) * 1000U + prn;
}

Ephemeris SPP::normalizeEphemerisToGpSt(const Ephemeris& eph) const {
    Ephemeris out = eph;
    out.toe = normalizeTowSec(out.toe);
    out.toc = normalizeTowSec(out.toc);
    return out;
}

std::vector<SPP::ProcObs> SPP::preprocessObservations(const std::vector<ObsData>& obsList) const {
    // 五步法-第1步：观测预处理
    // - 单星观测聚合：同一频点保留最高 SNR 的伪距，降低多路径与跟踪噪声风险
    // - 质量控制：应用 SNR 掩码剔除低质量观测
    // - 双频策略：可用时优先采用无电离层组合，以抑制电离层一阶误差对解算（尤其高度分量）的主导影响
    struct FreqObs {
        bool has0 = false;
        bool has1 = false;
        double P0 = 0.0;
        double P1 = 0.0;
        double snr0 = 0.0;
        double snr1 = 0.0;
        double dop0 = 0.0;
        double dop1 = 0.0;
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
                fo.dop0 = static_cast<double>(obs.doppler);
            }
        } else {
            if (!fo.has1 || obs.snr > fo.snr1) {
                fo.has1 = true;
                fo.P1 = obs.pseudorange;
                fo.snr1 = obs.snr;
                fo.dop1 = static_cast<double>(obs.doppler);
            }
        }
    }

    std::vector<ProcObs> out;
    out.reserve(m.size());

    for (const auto& kv : m) {
        const auto& fo = kv.second;
        if (!fo.has0) continue;
        if (enable_iono_free_ && !fo.has1) continue;
        if (enable_iono_free_ && !CheckOBSConsist(fo.P0, fo.P1)) continue;

        ProcObs po;
        po.system = fo.sys;
        po.prn = fo.prn;
        po.snr = std::max(fo.snr0, fo.snr1);
        po.iono_free = false;
        po.P = fo.P0;
        po.doppler = fo.dop0;
        po.wavelength = 0.0;
        po.has_doppler = std::isfinite(po.doppler);
        po.gf_m = 0.0;
        po.dgf_m = 0.0;
        po.gf_outlier = false;

        if (fo.sys == GnssSystem::GPS) {
            po.wavelength = SppConst::C / ErrorConst::GPS_L1;
        } else if (fo.sys == GnssSystem::BDS) {
            po.wavelength = SppConst::C / ErrorConst::BDS_B1I;
        }

        if (enable_iono_free_ && fo.has1) {
            const double gf_m = fo.P0 - fo.P1;
            double dgf_m = 0.0;
            if (DetectOutlier(fo.sys, fo.prn, gf_m, dgf_m)) {
                po.gf_m = gf_m;
                po.dgf_m = dgf_m;
                po.gf_outlier = true;
                out.push_back(po);
                continue;
            }
            po.gf_m = gf_m;
            po.dgf_m = dgf_m;
            po.gf_outlier = false;

            double f1 = 0.0;
            double f2 = 0.0;
            if (fo.sys == GnssSystem::GPS) {
                f1 = ErrorConst::GPS_L1;
                f2 = ErrorConst::GPS_L2;
            } else if (fo.sys == GnssSystem::BDS) {
                f1 = ErrorConst::BDS_B1I;
                f2 = ErrorConst::BDS_B3I;
            }

            if (f1 > 0.0 && f2 > 0.0) {
                double denom = (f1 * f1 - f2 * f2);
                if (std::abs(denom) > 1e-6) {
                    po.P = IonFreeCombo(fo.P0, fo.P1, f1, f2);
                    po.iono_free = true;
                }
            }
        }

        out.push_back(po);
    }

    return out;
}

bool SPP::CheckOBSConsist(double P1, double P2) const {
    if (!std::isfinite(P1) || !std::isfinite(P2)) return false;
    if (!(P1 > 0.0) || !(P2 > 0.0)) return false;
    return true;
}

bool SPP::DetectOutlier(GnssSystem sys, uint32_t prn, double gf_m, double& dgf_m) const {
    dgf_m = 0.0;
    if (!std::isfinite(gf_m)) return true;

    const uint32_t key = makeKey(sys, prn);
    auto it = prev_gf_.find(key);
    if (it == prev_gf_.end()) {
        prev_gf_[key] = PrevGf{current_tow_gpst_, gf_m};
        return false;
    }

    const double dt = normalizeTimeSec(current_tow_gpst_ - it->second.tow_gpst);
    if (std::abs(dt) > 10.0) {
        it->second = PrevGf{current_tow_gpst_, gf_m};
        return false;
    }

    dgf_m = gf_m - it->second.gf_m;
    if (!std::isfinite(dgf_m)) return true;

    const double kDgfThresholdM = 2.0;
    if (std::abs(dgf_m) > kDgfThresholdM) {
        return true;
    }

    it->second = PrevGf{current_tow_gpst_, gf_m};
    return false;
}

double SPP::computeTransmitTime(double t_rx_gpst, double pseudorange_m) const {
    // 五步法-第2步：计算信号发射时刻
    // t_tx = t_rx - ρ/c
    // 以观测伪距近似几何距离作为初值；随后通过卫星钟差对发射时刻进行二次修正以提高一致性。
    return t_rx_gpst - pseudorange_m / SppConst::C;
}

bool SPP::computeSatState(double t_rx_gpst,
                          double pseudorange_m,
                          const Ephemeris& eph,
                          const XYZ& rx_pos_approx,
                          const ProcObs& proc,
                          SatCalc& satCalc) const {
    if (!(pseudorange_m > 1.0)) return false;

    double dt = pseudorange_m / SppConst::C;
    SatState st{};
    SatState st_corr{};

    for (int k = 0; k < 3; ++k) {
        double t_tx = t_rx_gpst - dt;
        double t_tx_corr = t_tx;

        if (eph.system == GnssSystem::GPS) {
            const GpsEph gps_eph = ToGpsEph(eph);
            if (!ComputeGpsSatState(gps_eph, normalizeTowSec(t_tx), 0.0, st)) return false;
            t_tx_corr = t_tx - st.clk_s;
            if (!ComputeGpsSatState(gps_eph, normalizeTowSec(t_tx_corr), 0.0, st_corr)) return false;
        } else if (eph.system == GnssSystem::BDS) {
            const BdsEph bds_eph = ToBdsEph(eph);
            if (!ComputeBdsSatState(bds_eph, gpstToBdtSec(t_tx), 0.0, st)) return false;
            t_tx_corr = t_tx - st.clk_s;
            if (!ComputeBdsSatState(bds_eph, gpstToBdtSec(t_tx_corr), 0.0, st_corr)) return false;
        } else {
            return false;
        }

        const XYZ sat_pos{st_corr.pos_ecef.x, st_corr.pos_ecef.y, st_corr.pos_ecef.z};
        const double dx = sat_pos.x - rx_pos_approx.x;
        const double dy = sat_pos.y - rx_pos_approx.y;
        const double dz = sat_pos.z - rx_pos_approx.z;
        const double rho = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (!(rho > 1.0)) return false;

        const double dt_new = rho / SppConst::C;
        if (std::abs(dt_new - dt) <= 1e-10) {
            dt = dt_new;
            break;
        }
        dt = dt_new;
    }

    const double theta = SppConst::OMEGA_E * dt;
    const double cth = std::cos(theta);
    const double sth = std::sin(theta);

    const double x = st_corr.pos_ecef.x;
    const double y = st_corr.pos_ecef.y;
    const double z = st_corr.pos_ecef.z;
    satCalc.pos_ecef.x = x * cth + y * sth;
    satCalc.pos_ecef.y = -x * sth + y * cth;
    satCalc.pos_ecef.z = z;

    const double vx = st_corr.vel_ecef.x;
    const double vy = st_corr.vel_ecef.y;
    const double vz = st_corr.vel_ecef.z;
    satCalc.vel_ecef.x = vx * cth + vy * sth;
    satCalc.vel_ecef.y = -vx * sth + vy * cth;
    satCalc.vel_ecef.z = vz;

    satCalc.clk_s = st_corr.clk_s;
    if (!proc.iono_free) satCalc.clk_s -= eph.tgd;
    satCalc.clk_drift_s = st_corr.clk_drift;

    satCalc.has_doppler = proc.has_doppler && (proc.wavelength > 0.0) && std::isfinite(proc.doppler);
    if (satCalc.has_doppler) {
        satCalc.doppler_ms = -proc.doppler * proc.wavelength;
    } else {
        satCalc.doppler_ms = 0.0;
    }

    return true;
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

    BLH rx_blh{};
    const double rx_norm = std::sqrt(rx_pos.x * rx_pos.x + rx_pos.y * rx_pos.y + rx_pos.z * rx_pos.z);
    const bool rx_geo_ok = (rx_norm > 1.0) && (XYZToBLH(rx_pos, rx_blh) == 0);

    double elev_mask_rad = deg2rad(elev_mask_deg_);

    satCalcs.clear();
    satCalcs.reserve(procObs.size());

    for (const auto& o : procObs) {
        if (o.gf_outlier) continue;
        SatCalc sc;
        sc.system = o.system;
        sc.prn = o.prn;
        sc.valid = false;

        auto it = ephMap.find(makeKey(o.system, o.prn));
        if (it == ephMap.end()) continue;
        const auto& eph = it->second;

        if (!computeSatState(t_rx_gpst, o.P, eph, rx_pos, o, sc)) continue;
        const XYZ sat_pos = sc.pos_ecef;

        double dx = sat_pos.x - rx_pos.x;
        double dy = sat_pos.y - rx_pos.y;
        double dz = sat_pos.z - rx_pos.z;
        double rho0 = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (!(rho0 > 1.0)) continue;

        XYZ los;
        los.x = dx / rho0;
        los.y = dy / rho0;
        los.z = dz / rho0;

        const double rho = rho0;

        double elev = M_PI / 2.0;
        double az = 0.0;
        if (rx_geo_ok) {
            double E = 0.0;
            double N = 0.0;
            double U = 0.0;
            if (XyzToEnu(rx_pos, sat_pos, E, N, U) != 0) continue;
            elev = std::atan2(U, std::sqrt(E * E + N * N));
            az = std::atan2(E, N);
            if (az < 0.0) az += 2.0 * M_PI;
            if (!std::isfinite(elev) || elev < elev_mask_rad) continue;
        }

        double tropo = 0.0;
        if (rx_geo_ok) tropo = HopfieldTropDelay(rx_blh.height, elev);

        double P_corr = o.P + SppConst::C * sc.clk_s;

        double bias_m = clk_gps_m;
        if (o.system == GnssSystem::BDS) bias_m += isb_bds_m;

        double resid = P_corr - (rho + tropo + bias_m);

        sc.pos_ecef = sat_pos;
        sc.elev_rad = elev;
        sc.az_rad = az;
        sc.los = los;
        sc.rho = rho;
        sc.tropo_m = tropo;
        sc.P_corr = P_corr;
        sc.valid = true;

        satCalcs.push_back(sc);

        if (verbose_) {
            std::cout << "  [SAT] " << (o.system == GnssSystem::GPS ? "G" : "C") << o.prn
                      << " elev=" << elev * 180.0 / M_PI
                      << " clk_s=" << sc.clk_s
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
    const int m = static_cast<int>(sats.size());
    bool hasGps = false;
    bool hasBds = false;
    for (const auto& s : sats) {
        if (s.system == GnssSystem::GPS) hasGps = true;
        else if (s.system == GnssSystem::BDS) hasBds = true;
    }
    const int n = (hasGps && hasBds) ? 5 : 4;

    H.resize(m, n);
    Eigen::VectorXd y(m);
    Eigen::VectorXd w(m);

    for (int i = 0; i < m; ++i) {
        const auto& s = sats[i];
        const double bias_m = sol_.clk_gps_m + ((s.system == GnssSystem::BDS) ? sol_.isb_bds_m : 0.0);

        y(i) = s.P_corr - (s.rho + s.tropo_m + bias_m);
        H(i, 0) = -s.los.x;
        H(i, 1) = -s.los.y;
        H(i, 2) = -s.los.z;
        H(i, 3) = 1.0;
        if (n == 5) H(i, 4) = (s.system == GnssSystem::BDS) ? 1.0 : 0.0;

        double wi = std::sin(std::max(s.elev_rad, deg2rad(5.0)));
        wi = wi * wi;
        if (wi < 0.01) wi = 0.01;
        w(i) = wi;
    }

    Eigen::MatrixXd A = H;
    Eigen::VectorXd b = y;
    for (int i = 0; i < m; ++i) {
        const double sw = std::sqrt(w(i));
        A.row(i) *= sw;
        b(i) *= sw;
    }

    dx = A.colPivHouseholderQr().solve(b);
    if (!dx.allFinite()) return false;

    v = y - H * dx;
    return v.allFinite();
}

bool SPP::solveVelocity(const std::vector<SatCalc>& sats, const XYZ& rx_pos, Eigen::VectorXd& vel_dx) const {
    std::vector<const SatCalc*> used;
    used.reserve(sats.size());
    for (const auto& s : sats) {
        if (!s.valid) continue;
        if (!s.has_doppler) continue;
        if (!(s.rho > 1.0)) continue;
        used.push_back(&s);
    }
    if (static_cast<int>(used.size()) < 4) return false;

    const int m = static_cast<int>(used.size());
    Eigen::MatrixXd H(m, 4);
    Eigen::VectorXd y(m);
    Eigen::VectorXd w(m);

    for (int i = 0; i < m; ++i) {
        const auto& s = *used[i];
        H(i, 0) = -s.los.x;
        H(i, 1) = -s.los.y;
        H(i, 2) = -s.los.z;
        H(i, 3) = 1.0;

        const double dx = s.pos_ecef.x - rx_pos.x;
        const double dy = s.pos_ecef.y - rx_pos.y;
        const double dz = s.pos_ecef.z - rx_pos.z;
        const double rho = s.rho;
        const double proj_vs = (dx * s.vel_ecef.x + dy * s.vel_ecef.y + dz * s.vel_ecef.z) / rho;
        y(i) = s.doppler_ms - proj_vs + SppConst::C * s.clk_drift_s;

        double wi = std::sin(std::max(s.elev_rad, deg2rad(5.0)));
        wi = wi * wi;
        if (wi < 0.01) wi = 0.01;
        w(i) = wi;
    }

    Eigen::MatrixXd A = H;
    Eigen::VectorXd b = y;
    for (int i = 0; i < m; ++i) {
        const double sw = std::sqrt(w(i));
        A.row(i) *= sw;
        b(i) *= sw;
    }

    vel_dx = A.colPivHouseholderQr().solve(b);
    return vel_dx.size() == 4 && vel_dx.allFinite();
}

double SPP::computePDOP(const Eigen::MatrixXd& H) const {
    if (H.rows() < H.cols()) return 999.0;
    const Eigen::MatrixXd N = H.transpose() * H;
    Eigen::LDLT<Eigen::MatrixXd> ldlt(N);
    if (ldlt.info() != Eigen::Success) return 999.0;
    const Eigen::MatrixXd Q = ldlt.solve(Eigen::MatrixXd::Identity(N.rows(), N.cols()));
    if (!Q.allFinite()) return 999.0;
    if (!(Q(0, 0) > 0.0 && Q(1, 1) > 0.0 && Q(2, 2) > 0.0)) return 999.0;
    return std::sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));
}

int SPP::detectOutliers(std::vector<SatCalc>& sats, const Eigen::VectorXd& v, double sigma0) const {
    if (!(sigma0 > 0.0) || !std::isfinite(sigma0)) return 0;
    const int m = static_cast<int>(sats.size());
    if (v.size() != m) return 0;

    int rejected = 0;
    for (int i = 0; i < m; ++i) {
        const double w = std::abs(v(i)) / sigma0;
        if (w > 3.0) {
            if (sats[i].valid) {
                sats[i].valid = false;
                rejected++;
            }
        }
    }
    return rejected;
}

bool SPP::compute(double t_rx_gpst, const std::vector<ObsData>& obsList, const std::vector<Ephemeris>& ephList) {
    // 五步法整体驱动函数：
    // 1) preprocessObservations
    // 2) computeTransmitTime
    // 3) computeSatellitePosition / computeSatelliteClock
    // 4) buildObservationEquation
    // 5) solveLeastSquares（迭代更新）
    last_rejected_ = 0;
    last_vel_ok_ = false;
    sol_.pdop = 999.0;
    sol_.sigma0 = 0.0;
    sol_.nsat_used = 0;
    sol_.nsat_gps = 0;
    sol_.nsat_bds = 0;
    sol_.vel = {0.0, 0.0, 0.0};
    sol_.clk_drift_ms = 0.0;

    current_tow_gpst_ = t_rx_gpst;
    std::vector<ProcObs> procObs = preprocessObservations(obsList);
    int usable_obs = 0;
    for (const auto& o : procObs) {
        if (!o.gf_outlier) usable_obs++;
    }
    if (usable_obs < 4) return false;

    XYZ rx_pos = sol_.pos;
    double clk_gps_m = sol_.clk_gps_m;
    double isb_bds_m = sol_.isb_bds_m;
    if (std::sqrt(rx_pos.x * rx_pos.x + rx_pos.y * rx_pos.y + rx_pos.z * rx_pos.z) <= 1.0) {
        rx_pos = {0.0, 0.0, 0.0};
        clk_gps_m = 0.0;
        isb_bds_m = 0.0;
    }

    std::set<uint32_t> rejected_keys;
    Eigen::VectorXd v_last;
    Eigen::MatrixXd H_last;
    std::vector<SatCalc> sats_last;
    double sigma0_last = 0.0;

    for (int out_round = 0; out_round < 2; ++out_round) {
        bool converged = false;
        for (int iter = 0; iter < max_iter_; ++iter) {
            sol_.pos = rx_pos;
            sol_.clk_gps_m = clk_gps_m;
            sol_.isb_bds_m = isb_bds_m;

            const double t_rx_true = t_rx_gpst - (clk_gps_m / SppConst::C);
            std::vector<SatCalc> sats;
            if (!buildObservationEquation(t_rx_true, procObs, ephList, rx_pos, clk_gps_m, isb_bds_m, sats)) return false;

            std::vector<SatCalc> filtered;
            filtered.reserve(sats.size());
            int nGps = 0;
            int nBds = 0;
            for (const auto& s : sats) {
                if (!s.valid) continue;
                if (rejected_keys.count(makeKey(s.system, s.prn))) continue;
                filtered.push_back(s);
                if (s.system == GnssSystem::GPS) nGps++;
                else if (s.system == GnssSystem::BDS) nBds++;
            }
            sats.swap(filtered);

            const bool dual = (nGps > 0 && nBds > 0);
            const int required = dual ? 5 : 4;
            if (static_cast<int>(sats.size()) < required) return false;

            if (verbose_) {
                double elev_min = std::numeric_limits<double>::infinity();
                double elev_max = -std::numeric_limits<double>::infinity();
                double trop_min = std::numeric_limits<double>::infinity();
                double trop_max = -std::numeric_limits<double>::infinity();
                for (const auto& s : sats) {
                    elev_min = std::min(elev_min, s.elev_rad);
                    elev_max = std::max(elev_max, s.elev_rad);
                    trop_min = std::min(trop_min, s.tropo_m);
                    trop_max = std::max(trop_max, s.tropo_m);
                }
                std::cout << "  [OBS] n=" << sats.size()
                          << " nG=" << nGps
                          << " nC=" << nBds
                          << " elev_deg=[" << elev_min * 180.0 / M_PI << "," << elev_max * 180.0 / M_PI << "]"
                          << " trop_m=[" << trop_min << "," << trop_max << "]"
                          << std::endl;
            }

            Eigen::VectorXd dx;
            Eigen::VectorXd v;
            Eigen::MatrixXd H;
            if (!solveLeastSquares(sats, dx, v, H)) return false;

            if (verbose_) {
                std::cout << "    dx=(" << dx.transpose() << ")"
                          << " vmax=" << v.cwiseAbs().maxCoeff()
                          << std::endl;
            }

            if (dx.size() == 4) {
                rx_pos.x += dx(0);
                rx_pos.y += dx(1);
                rx_pos.z += dx(2);
                clk_gps_m += dx(3);
            } else if (dx.size() == 5) {
                rx_pos.x += dx(0);
                rx_pos.y += dx(1);
                rx_pos.z += dx(2);
                clk_gps_m += dx(3);
                isb_bds_m += dx(4);
            } else {
                return false;
            }

            const double dpos2 = dx(0) * dx(0) + dx(1) * dx(1) + dx(2) * dx(2);
            if (verbose_) {
                std::cout << "  [ITER] " << iter + 1
                          << " dpos=" << std::sqrt(dpos2)
                          << " clk_gps_m=" << clk_gps_m
                          << " isb_bds_m=" << isb_bds_m
                          << " used=" << sats.size()
                          << std::endl;
            }

            v_last = v;
            H_last = H;
            sats_last = sats;

            Eigen::VectorXd w(sats.size());
            for (int i = 0; i < static_cast<int>(sats.size()); ++i) {
                double wi = std::sin(std::max(sats[i].elev_rad, deg2rad(5.0)));
                wi = wi * wi;
                if (wi < 0.01) wi = 0.01;
                w(i) = wi;
            }
            const int dof = static_cast<int>(sats.size()) - static_cast<int>(H.cols());
            sigma0_last = 0.0;
            if (dof > 0) sigma0_last = std::sqrt((v.array().square() * w.array()).sum() / static_cast<double>(dof));

            if (dpos2 < 1e-6) {
                converged = true;
                break;
            }
        }

        if (!converged) return false;

        std::vector<SatCalc> sats_candidate = sats_last;
        const int nrej = detectOutliers(sats_candidate, v_last, sigma0_last);
        if (nrej <= 0) break;

        int nGps = 0;
        int nBds = 0;
        int nValid = 0;
        for (const auto& s : sats_candidate) {
            if (!s.valid) continue;
            nValid++;
            if (s.system == GnssSystem::GPS) nGps++;
            else if (s.system == GnssSystem::BDS) nBds++;
        }
        const bool dual = (nGps > 0 && nBds > 0);
        const int required = dual ? 5 : 4;
        if (nValid < required) break;

        for (const auto& s : sats_candidate) {
            if (!s.valid) rejected_keys.insert(makeKey(s.system, s.prn));
        }
        last_rejected_ += nrej;
    }

    sol_.pos = rx_pos;
    sol_.clk_gps_m = clk_gps_m;
    sol_.isb_bds_m = isb_bds_m;
    sol_.sigma0 = sigma0_last;
    sol_.pdop = computePDOP(H_last);

    int nGpsUsed = 0;
    int nBdsUsed = 0;
    for (const auto& s : sats_last) {
        if (s.system == GnssSystem::GPS) nGpsUsed++;
        else if (s.system == GnssSystem::BDS) nBdsUsed++;
    }
    sol_.nsat_used = static_cast<int>(sats_last.size());
    sol_.nsat_gps = nGpsUsed;
    sol_.nsat_bds = nBdsUsed;

    Eigen::VectorXd vel_dx;
    if (solveVelocity(sats_last, sol_.pos, vel_dx)) {
        sol_.vel = {vel_dx(0), vel_dx(1), vel_dx(2)};
        sol_.clk_drift_ms = vel_dx(3);
        last_vel_ok_ = true;
    }

    return true;
}

bool SPP::computeWithReport(double t_rx_gpst, const std::vector<ObsData>& obsList, const std::vector<Ephemeris>& ephList, SppEpochReport& report) {
    // 工程化入口：对每个 RANGE 历元输出一行报告，用于结果落盘、统计与问题回溯。
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
        report.vel = sol_.vel;
        report.clk_drift_ms = sol_.clk_drift_ms;
        report.pdop = sol_.pdop;
        report.sigma0 = sol_.sigma0;
        report.nsat_used = sol_.nsat_used;
        report.rejected = last_rejected_;
        report.vel_ok = last_vel_ok_;
        return true;
    }

    double nan = std::numeric_limits<double>::quiet_NaN();
    report.pos = {nan, nan, nan};
    report.clk_gps_m = nan;
    report.isb_bds_m = nan;
    return false;
}

} // namespace gnss
