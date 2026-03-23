/**
 * @file main.cpp
 * @brief GNSS 数据流驱动与 SPP 在线定位程序
 * @author Jiaoyang Wu
 * @date 2026-03-23
 *
 * 工程职责划分：
 * - main.cpp：负责二进制数据流读取、解码器驱动、结果落盘与宏观统计输出。
 * - SPP 模块：负责观测预处理、星历匹配、时间系统统一、卫星位置/钟差计算、最小二乘解算等定位领域逻辑。
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <fstream>
#include <map>
#include <filesystem>
#include <gnss_positioning/NavMath.h>
#include <gnss_positioning/NovatelDecoder.h>
#include <gnss_positioning/SPP.h>

/**
 * @brief 程序入口：驱动数据解码与单点定位解算，并将历元结果输出到文件
 * @return 0: 正常退出；非 0: 异常退出
 */
int main() {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "GNSS SPP 定位程序" << std::endl;

    gnss::NovatelDecoder decoder;
    gnss::SPP sppSolver;
    
    // 星历缓存：按系统+PRN 存储最新星历，用于后续每个历元的在线定位
    std::map<uint32_t, gnss::Ephemeris> ephStore;
    auto makeKey = [](gnss::GnssSystem sys, uint32_t prn) -> uint32_t {
        return static_cast<uint32_t>(sys) * 1000U + prn;
    };

    std::string inputPath = "D:/gnss_positioning/Novatel OEM7 Data/01/NovatelOEM20211114-01.log";
    std::ifstream inFile(inputPath, std::ios::binary);
    if (!inFile.is_open()) {
        std::cerr << "[ERROR] 无法打开输入文件: " << inputPath << std::endl;
        return 1;
    }

    // 输出目录与定位结果文件
    std::filesystem::create_directories("D:/gnss_positioning/output");
    std::string outPath = "D:/gnss_positioning/output/spp_solution.txt";
    std::ofstream outFile(outPath);
    if (!outFile.is_open()) {
        std::cerr << "[ERROR] 无法创建输出文件: " << outPath << std::endl;
        return 1;
    }
    outFile.setf(std::ios::fixed, std::ios::floatfield);
    outFile << std::setprecision(3);
    outFile << "TOW OBS_G OBS_B EPH_G EPH_B MATCH_G MATCH_B ATTEMPT OK X(m) Y(m) Z(m) CLK_GPS(m) ISB_BDS(m)" << std::endl;

    uint32_t totalBytes = 0;
    uint32_t rangeFrameCount = 0;
    uint32_t gpsEphCount = 0;
    uint32_t bdsEphCount = 0;
    uint32_t sppOkCount = 0;
    uint32_t sppFailCount = 0;
    uint32_t skipNoEphCount = 0;
    uint32_t skipLowMatchCount = 0;
    uint32_t solveAttemptCount = 0;
    bool hasFirstOk = false;
    double firstOkTow = 0.0;
    double lastOkTow = 0.0;
    std::vector<double> lastOkPos;
    double lastOkClk = 0.0;

    uint8_t byte;
    while (inFile.read(reinterpret_cast<char*>(&byte), 1)) {
        totalBytes++;
        if (decoder.inputData(byte)) {
            uint16_t msgId = decoder.getMsgId();

            if (msgId == 7) { // GPSEPHEM
                const auto& gps_eph = decoder.getGpsEphem();
                if (gps_eph.prn >= 1 && gps_eph.prn <= 32) {
                    ephStore[makeKey(gnss::GnssSystem::GPS, gps_eph.prn)] = gnss::ConvertFromGpsEphem(gps_eph);
                    gpsEphCount++;
                }
            } else if (msgId == 1696) { // BDSEPHEMERIS
                const auto& bds_eph = decoder.getBdsEphem();
                if (bds_eph.prn >= 1 && bds_eph.prn <= 63) {
                    ephStore[makeKey(gnss::GnssSystem::BDS, bds_eph.prn)] = gnss::ConvertFromBdsEphem(bds_eph);
                    bdsEphCount++;
                }
            } else if (msgId == 43) { // RANGE
                rangeFrameCount++;
                double tow = decoder.getTow();
                const auto& obsList = decoder.getObsData();
                
                // 执行定位解算
                std::vector<gnss::Ephemeris> currentEphList;
                for (const auto& pair : ephStore) {
                    currentEphList.push_back(pair.second);
                }

                gnss::SppEpochReport report;
                bool ok = sppSolver.computeWithReport(tow, obsList, currentEphList, report);

                if (report.attempted) solveAttemptCount++;
                if (ok) sppOkCount++;
                else sppFailCount++;
                if (!report.attempted && (report.eph_g + report.eph_b) == 0) skipNoEphCount++;
                if (!report.attempted && (report.eph_g + report.eph_b) > 0) skipLowMatchCount++;

                outFile << report.tow_gpst << " "
                        << report.obs_g << " " << report.obs_b << " "
                        << report.eph_g << " " << report.eph_b << " "
                        << report.match_g << " " << report.match_b << " "
                        << (report.attempted ? 1 : 0) << " "
                        << (report.ok ? 1 : 0) << " "
                        << report.pos.x << " " << report.pos.y << " " << report.pos.z << " "
                        << report.clk_gps_m << " " << report.isb_bds_m
                        << std::endl;

                if (ok) {
                    if (!hasFirstOk) {
                        hasFirstOk = true;
                        firstOkTow = report.tow_gpst;
                    }
                    lastOkTow = report.tow_gpst;
                    lastOkPos = sppSolver.getPosition();
                    lastOkClk = report.clk_gps_m;
                }
            }
        }
    }

    std::cout << "SUMMARY bytes=" << totalBytes
              << " RANGE=" << rangeFrameCount
              << " GPSEPHEM=" << gpsEphCount
              << " BDSEPHEM=" << bdsEphCount
              << " SOLVE_ATTEMPT=" << solveAttemptCount
              << " SPP_OK=" << sppOkCount
              << " SPP_FAIL=" << sppFailCount
              << " SKIP_NO_EPH=" << skipNoEphCount
              << " SKIP_LOW_MATCH=" << skipLowMatchCount
              << std::endl;

    if (hasFirstOk && lastOkPos.size() == 3) {
        std::cout << "FIRST_OK_TOW=" << firstOkTow << " LAST_OK_TOW=" << lastOkTow << std::endl;
        std::cout << "LAST_OK_XYZ=" << lastOkPos[0] << "," << lastOkPos[1] << "," << lastOkPos[2]
                  << " LAST_OK_CLK(m)=" << lastOkClk << std::endl;
    }
    std::cout << "OUTPUT=" << outPath << std::endl;

    return 0;
}
