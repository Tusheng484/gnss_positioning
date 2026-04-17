#ifndef GNSS_POSITIONING_CONFIG_H
#define GNSS_POSITIONING_CONFIG_H

#include <array>
#include <string>

namespace gnss {

/**
 * @brief SPP 频率模式
 *
 * 说明：
 * - `SINGLE_FREQ_KLOBUCHAR`：单频伪距，结合 Klobuchar 模型进行电离层改正；
 * - `DUAL_FREQ_IF`：双频无电离层组合，不再使用 Klobuchar 改正。
 */
enum class SppFrequencyMode {
    SINGLE_FREQ_KLOBUCHAR,
    DUAL_FREQ_IF
};

/**
 * @brief 将 SPP 频率模式转换为可读字符串
 * @param mode 频率模式枚举值
 * @return 对应的常量字符串，便于日志和配置回显
 */
const char* ToString(SppFrequencyMode mode);

/**
 * @brief 单系统 Klobuchar 参数配置
 */
struct KlobucharConfig {
    bool enabled = false;
    std::array<double, 4> alpha{0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> beta{0.0, 0.0, 0.0, 0.0};
};

/**
 * @brief SPP 算法配置
 */
struct SppOptions {
    SppFrequencyMode frequency_mode = SppFrequencyMode::DUAL_FREQ_IF;
    bool enable_tropo = true;
    bool enable_tgd = true;
    bool enable_klobuchar_model = false;
    double elev_mask_deg = 10.0;
    double snr_mask_dbhz = 30.0;
    KlobucharConfig gps_klobuchar;
    KlobucharConfig bds_klobuchar;

    /**
     * @brief 判断是否启用双频无电离层组合
     * @return true 表示使用 IF 组合；false 表示不使用
     */
    bool useIonoFree() const { return frequency_mode == SppFrequencyMode::DUAL_FREQ_IF; }

    /**
     * @brief 判断是否启用 Klobuchar 电离层模型
     * @return true 表示单频模式下启用 Klobuchar；false 表示不使用
     */
    bool useKlobuchar() const {
        return frequency_mode == SppFrequencyMode::SINGLE_FREQ_KLOBUCHAR && enable_klobuchar_model;
    }
};

/**
 * @brief 接收机参考坐标配置
 */
struct ReferenceEcef {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/**
 * @brief 集中式运行参数配置
 *
 * 设计目标：
 * - 所有业务参数统一在该类中声明并校验；
 * - 主程序只读取一次配置文件，并通过只读接口访问；
 * - 配置校验失败时抛出标准异常，阻止后续业务逻辑在无效参数下运行。
 */
class Config {
public:
    /**
     * @brief 默认配置文件相对路径
     */
    static constexpr const char* kDefaultConfigPath = "D:\\gnss_positioning\\config\\params.yaml";

    /**
     * @brief 从 YAML 配置文件加载配置
     * @param file_path 配置文件路径，支持相对路径或绝对路径，不能为空
     * @return 校验通过后的只读配置对象
     * @exception std::invalid_argument 当配置文件缺失、格式错误或参数校验失败时抛出
     */
    static Config LoadFromYaml(const std::string& file_path);

    /**
     * @brief 获取配置文件路径
     * @return 配置文件路径字符串引用
     */
    const std::string& sourcePath() const { return source_path_; }

    /**
     * @brief 获取输入日志文件路径
     * @return 输入文件路径字符串引用
     */
    const std::string& inputPath() const { return input_path_; }

    /**
     * @brief 获取输出目录路径
     * @return 输出目录路径字符串引用
     */
    const std::string& outputDir() const { return output_dir_; }

    /**
     * @brief 获取终端输出开关
     * @return true 表示启用终端输出；false 表示静默运行
     */
    bool terminalOutputEnabled() const { return terminal_output_; }

    /**
     * @brief 获取总落盘开关
     * @return true 表示允许创建和写入输出文件；false 表示完全不落盘
     */
    bool fileOutputEnabled() const { return file_output_; }

    /**
     * @brief 获取 SPP 结果文件开关
     * @return true 表示输出 `spp_solution.txt`
     */
    bool fileSppSolutionEnabled() const { return file_spp_solution_; }

    /**
     * @brief 获取卫星状态文件开关
     * @return true 表示输出 `sat_states.txt`
     */
    bool fileSatStatesEnabled() const { return file_sat_states_; }

    /**
     * @brief 获取星历文件开关
     * @return true 表示输出广播星历文件
     */
    bool fileEphemerisEnabled() const { return file_ephemeris_; }

    /**
     * @brief 获取观测文件开关
     * @return true 表示输出原始观测文件
     */
    bool fileObservationsEnabled() const { return file_observations_; }

    /**
     * @brief 获取 SPP 解算开关
     * @return true 表示启用单点定位；false 表示仅做解析/落盘
     */
    bool sppEnabled() const { return enable_spp_; }

    /**
     * @brief 获取卫星状态计算开关
     * @return true 表示启用广播星历状态计算；false 表示跳过
     */
    bool satStateEnabled() const { return enable_sat_state_; }

    /**
     * @brief 获取 SPP 算法配置
     * @return SPP 参数结构体常量引用
     */
    const SppOptions& sppOptions() const { return spp_options_; }

    /**
     * @brief 获取参考 ECEF 坐标
     * @return 参考坐标结构体常量引用
     */
    const ReferenceEcef& referenceEcef() const { return reference_ecef_; }

private:
    /**
     * @brief 校验全部配置项
     * @exception std::invalid_argument 当任一字段越界或依赖关系不成立时抛出
     */
    void validate() const;

    std::string source_path_;
    std::string input_path_;
    std::string output_dir_;
    bool terminal_output_ = false;
    bool file_output_ = true;
    bool file_spp_solution_ = true;
    bool file_sat_states_ = true;
    bool file_ephemeris_ = true;
    bool file_observations_ = true;
    bool enable_spp_ = true;
    bool enable_sat_state_ = true;
    SppOptions spp_options_;
    ReferenceEcef reference_ecef_;
};

} // namespace gnss

#endif // GNSS_POSITIONING_CONFIG_H
