/**
 * @file Config.cpp
 * @brief SPP 程序集中式参数配置模块实现：YAML 读取、字段解析、范围校验与依赖关系校验
 * @author Jiaoyang Wu
 * @date 2026-04-20
 *
 * 单位约定：
 * - 角度：度 (deg)
 * - 距离：米 (m)
 * - 时间：秒 (s)
 */

#include "gnss_positioning/Config.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace gnss {

namespace {

/**
 * @brief 去除字符串首尾空白字符
 * @param text 输入输出字符串，允许为空
 * @return 无
 */
void Trim(std::string& text) {
    auto is_space = [](unsigned char ch) { return std::isspace(ch) != 0; };
    while (!text.empty() && is_space(static_cast<unsigned char>(text.front()))) text.erase(text.begin());
    while (!text.empty() && is_space(static_cast<unsigned char>(text.back()))) text.pop_back();
}

/**
 * @brief 去除行尾注释
 * @param line 原始配置行
 * @return 删除 `#` 注释后的内容
 */
std::string StripComment(const std::string& line) {
    const std::size_t pos = line.find('#');
    return (pos == std::string::npos) ? line : line.substr(0, pos);
}

/**
 * @brief 统计行首空格缩进数
 * @param line 输入行文本
 * @return 行首空格数量，取值范围 `[0, line.size()]`
 */
int CountIndent(const std::string& line) {
    int indent = 0;
    while (indent < static_cast<int>(line.size()) && line[static_cast<std::size_t>(indent)] == ' ') ++indent;
    return indent;
}

/**
 * @brief 去除成对包裹的单引号或双引号
 * @param value 输入值字符串
 * @return 去引号后的字符串；若无成对引号则返回原值
 */
std::string Unquote(std::string value) {
    Trim(value);
    if (value.size() >= 2U) {
        const char first = value.front();
        const char last = value.back();
        if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
            return value.substr(1, value.size() - 2U);
        }
    }
    return value;
}

/**
 * @brief 将字符串转换为小写
 * @param value 输入字符串
 * @return 小写版本字符串
 */
std::string ToLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

/**
 * @brief 解析布尔值
 * @param text 输入字符串，支持 `true/false/on/off/1/0`
 * @return 解析后的布尔值
 * @exception std::invalid_argument 当值不属于支持范围时抛出
 */
bool ParseBool(const std::string& text) {
    const std::string value = ToLower(Unquote(text));
    if (value == "true" || value == "1" || value == "on") return true;
    if (value == "false" || value == "0" || value == "off") return false;
    throw std::invalid_argument("Config parse failed: invalid bool value '" + text + "'.");
}

/**
 * @brief 解析双精度浮点数
 * @param text 输入字符串，要求可被 `std::stod` 正确解析
 * @return 解析后的双精度值
 * @exception std::invalid_argument 当输入为空或包含非法字符时抛出
 */
double ParseDouble(const std::string& text) {
    const std::string value = Unquote(text);
    std::size_t consumed = 0;
    const double out = std::stod(value, &consumed);
    if (consumed != value.size()) {
        throw std::invalid_argument("Config parse failed: invalid numeric value '" + text + "'.");
    }
    return out;
}

/**
 * @brief 解析长度为 4 的浮点数组
 * @param text 输入字符串，格式示例：`[1, 2, 3, 4]`
 * @return 四元素数组
 * @exception std::invalid_argument 当格式错误、元素不足或元素非法时抛出
 */
std::array<double, 4> ParseArray4(const std::string& text) {
    std::string value = Unquote(text);
    Trim(value);
    if (value.size() < 2U || value.front() != '[' || value.back() != ']') {
        throw std::invalid_argument("Config parse failed: expected 4-element YAML array, got '" + text + "'.");
    }

    value = value.substr(1, value.size() - 2U);
    for (char& ch : value) {
        if (ch == ',') ch = ' ';
    }

    std::array<double, 4> out{0.0, 0.0, 0.0, 0.0};
    std::istringstream iss(value);
    for (double& item : out) {
        if (!(iss >> item)) {
            throw std::invalid_argument("Config parse failed: invalid 4-element array '" + text + "'.");
        }
    }
    std::string tail;
    if (iss >> tail) {
        throw std::invalid_argument("Config parse failed: too many elements in array '" + text + "'.");
    }
    return out;
}

/**
 * @brief 判断数组中是否存在非零值
 * @param values 四元素数组
 * @return true 表示至少一个元素绝对值大于 `1e-12`
 */
bool HasNonZero(const std::array<double, 4>& values) {
    for (double value : values) {
        if (std::abs(value) > 1e-12) return true;
    }
    return false;
}

/**
 * @brief 判断数组元素是否均为有限数
 * @param values 四元素数组
 * @return true 表示全部元素均为有限数
 */
bool AllFinite(const std::array<double, 4>& values) {
    for (double value : values) {
        if (!std::isfinite(value)) return false;
    }
    return true;
}

} // namespace

const char* ToString(SppFrequencyMode mode) {
    switch (mode) {
    case SppFrequencyMode::SINGLE_FREQ_KLOBUCHAR:
        return "SINGLE_FREQ_KLOBUCHAR";
    case SppFrequencyMode::DUAL_FREQ_IF:
        return "DUAL_FREQ_IF";
    default:
        return "UNKNOWN";
    }
}

Config Config::LoadFromYaml(const std::string& file_path) {
    if (file_path.empty()) {
        throw std::invalid_argument("Config validation failed: config file path must not be empty.");
    }

    std::ifstream input(file_path);
    if (!input.is_open()) {
        throw std::invalid_argument("Config load failed: unable to open config file '" + file_path + "'.");
    }

    Config cfg;
    cfg.source_path_ = file_path;

    std::vector<std::pair<int, std::string>> section_stack;
    std::string raw_line;
    int line_no = 0;
    while (std::getline(input, raw_line)) {
        ++line_no;
        std::string line = StripComment(raw_line);
        Trim(line);
        if (line.empty()) continue;

        const int indent = CountIndent(raw_line);
        const std::size_t colon_pos = line.find(':');
        if (colon_pos == std::string::npos) {
            throw std::invalid_argument("Config parse failed at line " + std::to_string(line_no) + ": missing ':'.");
        }

        std::string key = line.substr(0, colon_pos);
        std::string value = line.substr(colon_pos + 1);
        Trim(key);
        Trim(value);
        if (key.empty()) {
            throw std::invalid_argument("Config parse failed at line " + std::to_string(line_no) + ": empty key.");
        }

        while (!section_stack.empty() && section_stack.back().first >= indent) {
            section_stack.pop_back();
        }

        if (value.empty()) {
            section_stack.emplace_back(indent, key);
            continue;
        }

        std::string full_key;
        for (const auto& section : section_stack) {
            if (!full_key.empty()) full_key += '.';
            full_key += section.second;
        }
        if (!full_key.empty()) full_key += '.';
        full_key += key;

        if (full_key == "io.input_path") cfg.input_path_ = Unquote(value);
        else if (full_key == "io.output_dir") cfg.output_dir_ = Unquote(value);
        else if (full_key == "output.terminal_output") cfg.terminal_output_ = ParseBool(value);
        else if (full_key == "output.file_output") cfg.file_output_ = ParseBool(value);
        else if (full_key == "output.file_spp_solution") cfg.file_spp_solution_ = ParseBool(value);
        else if (full_key == "output.file_sat_states") cfg.file_sat_states_ = ParseBool(value);
        else if (full_key == "output.file_ephemeris") cfg.file_ephemeris_ = ParseBool(value);
        else if (full_key == "output.file_observations") cfg.file_observations_ = ParseBool(value);
        else if (full_key == "runtime.enable_spp") cfg.enable_spp_ = ParseBool(value);
        else if (full_key == "runtime.enable_sat_state") cfg.enable_sat_state_ = ParseBool(value);
        else if (full_key == "spp.frequency_mode") {
            const std::string mode = ToLower(Unquote(value));
            if (mode == "single_freq_klobuchar" || mode == "single_freq" || mode == "single") {
                cfg.spp_options_.frequency_mode = SppFrequencyMode::SINGLE_FREQ_KLOBUCHAR;
            } else if (mode == "dual_freq_if" || mode == "dual" || mode == "if") {
                cfg.spp_options_.frequency_mode = SppFrequencyMode::DUAL_FREQ_IF;
            } else {
                throw std::invalid_argument("Config parse failed: unsupported spp.frequency_mode '" + value + "'.");
            }
        } else if (full_key == "spp.enable_tropo") cfg.spp_options_.enable_tropo = ParseBool(value);
        else if (full_key == "spp.enable_tgd") cfg.spp_options_.enable_tgd = ParseBool(value);
        else if (full_key == "spp.enable_klobuchar_model") cfg.spp_options_.enable_klobuchar_model = ParseBool(value);
        else if (full_key == "spp.elev_mask_deg") cfg.spp_options_.elev_mask_deg = ParseDouble(value);
        else if (full_key == "spp.snr_mask_dbhz") cfg.spp_options_.snr_mask_dbhz = ParseDouble(value);
        else if (full_key == "spp.gps_klobuchar.enabled") cfg.spp_options_.gps_klobuchar.enabled = ParseBool(value);
        else if (full_key == "spp.gps_klobuchar.alpha") cfg.spp_options_.gps_klobuchar.alpha = ParseArray4(value);
        else if (full_key == "spp.gps_klobuchar.beta") cfg.spp_options_.gps_klobuchar.beta = ParseArray4(value);
        else if (full_key == "spp.bds_klobuchar.enabled") cfg.spp_options_.bds_klobuchar.enabled = ParseBool(value);
        else if (full_key == "spp.bds_klobuchar.alpha") cfg.spp_options_.bds_klobuchar.alpha = ParseArray4(value);
        else if (full_key == "spp.bds_klobuchar.beta") cfg.spp_options_.bds_klobuchar.beta = ParseArray4(value);
        else if (full_key == "reference_ecef.x") cfg.reference_ecef_.x = ParseDouble(value);
        else if (full_key == "reference_ecef.y") cfg.reference_ecef_.y = ParseDouble(value);
        else if (full_key == "reference_ecef.z") cfg.reference_ecef_.z = ParseDouble(value);
        else {
            throw std::invalid_argument("Config parse failed: unsupported key '" + full_key + "'.");
        }
    }

    cfg.validate();
    return cfg;
}

void Config::validate() const {
    if (input_path_.empty()) {
        throw std::invalid_argument("Config validation failed: io.input_path must not be empty.");
    }
    if (file_output_ && output_dir_.empty()) {
        throw std::invalid_argument("Config validation failed: io.output_dir must not be empty when output.file_output=true.");
    }
    if (!(spp_options_.elev_mask_deg >= 0.0 && spp_options_.elev_mask_deg <= 90.0)) {
        throw std::invalid_argument("Config validation failed: spp.elev_mask_deg must be within [0, 90] degrees.");
    }
    if (!(spp_options_.snr_mask_dbhz >= 0.0 && spp_options_.snr_mask_dbhz <= 70.0)) {
        throw std::invalid_argument("Config validation failed: spp.snr_mask_dbhz must be within [0, 70] dB-Hz.");
    }
    if (!std::isfinite(reference_ecef_.x) || !std::isfinite(reference_ecef_.y) || !std::isfinite(reference_ecef_.z)) {
        throw std::invalid_argument("Config validation failed: reference_ecef.{x,y,z} must be finite values.");
    }

    const bool use_klobuchar = spp_options_.useKlobuchar();
    const bool use_iono_free = spp_options_.useIonoFree();
    if (use_iono_free && spp_options_.enable_klobuchar_model) {
        throw std::invalid_argument("Config validation failed: spp.enable_klobuchar_model must be false when spp.frequency_mode=DUAL_FREQ_IF.");
    }
    if (use_iono_free && (spp_options_.gps_klobuchar.enabled || spp_options_.bds_klobuchar.enabled)) {
        throw std::invalid_argument("Config validation failed: system Klobuchar parameters must be disabled when spp.frequency_mode=DUAL_FREQ_IF.");
    }
    if (!use_klobuchar && spp_options_.gps_klobuchar.enabled) {
        throw std::invalid_argument("Config validation failed: spp.gps_klobuchar.enabled must be false when Klobuchar mode is inactive.");
    }
    if (!use_klobuchar && spp_options_.bds_klobuchar.enabled) {
        throw std::invalid_argument("Config validation failed: spp.bds_klobuchar.enabled must be false when Klobuchar mode is inactive.");
    }
    if (use_klobuchar && !spp_options_.gps_klobuchar.enabled && !spp_options_.bds_klobuchar.enabled) {
        throw std::invalid_argument("Config validation failed: at least one of spp.gps_klobuchar.enabled or spp.bds_klobuchar.enabled must be true in SINGLE_FREQ_KLOBUCHAR mode.");
    }
    if (!spp_options_.gps_klobuchar.enabled &&
        (HasNonZero(spp_options_.gps_klobuchar.alpha) || HasNonZero(spp_options_.gps_klobuchar.beta))) {
        throw std::invalid_argument("Config validation failed: spp.gps_klobuchar.alpha/beta must remain zero when spp.gps_klobuchar.enabled=false.");
    }
    if (!spp_options_.bds_klobuchar.enabled &&
        (HasNonZero(spp_options_.bds_klobuchar.alpha) || HasNonZero(spp_options_.bds_klobuchar.beta))) {
        throw std::invalid_argument("Config validation failed: spp.bds_klobuchar.alpha/beta must remain zero when spp.bds_klobuchar.enabled=false.");
    }
    if (!AllFinite(spp_options_.gps_klobuchar.alpha) || !AllFinite(spp_options_.gps_klobuchar.beta)) {
        throw std::invalid_argument("Config validation failed: spp.gps_klobuchar.alpha/beta must be finite values.");
    }
    if (!AllFinite(spp_options_.bds_klobuchar.alpha) || !AllFinite(spp_options_.bds_klobuchar.beta)) {
        throw std::invalid_argument("Config validation failed: spp.bds_klobuchar.alpha/beta must be finite values.");
    }
}

} // namespace gnss
