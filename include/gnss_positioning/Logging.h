/**
 * @file Logging.h
 * @brief 轻量级日志模块：提供 `LOG(INFO) << ...` 风格输出接口，供主流程参数回显与运行日志使用
 * @author Jiaoyang Wu
 * @date 2026-04-20
 *
 * 使用约定：
 * - 该模块仅依赖 C++ 标准库；
 * - 输出默认写入标准输出流；
 * - 日志级别通过宏参数传入（如 INFO/WARN/ERROR）。
 */

#ifndef GNSS_POSITIONING_LOGGING_H
#define GNSS_POSITIONING_LOGGING_H

#include <iostream>
#include <sstream>
#include <utility>

namespace gnss {

/**
 * @brief 轻量级日志消息构造器
 *
 * 用途：
 * - 为当前工程提供 `LOG(INFO) << ...` 形式的可读日志接口；
 * - 仅依赖标准库，避免为主流程重构引入额外第三方日志依赖。
 */
class LogMessage {
public:
    /**
     * @brief 构造指定级别的日志消息
     * @param level 日志级别字符串，取值示例：`"INFO"`、`"WARN"`、`"ERROR"`
     */
    explicit LogMessage(const char* level) {
        stream_ << "[" << (level ? level : "INFO") << "] ";
    }

    /**
     * @brief 析构时输出完整日志行
     * @exception noexcept 不抛出异常；若标准输出失败则由流状态记录错误
     */
    ~LogMessage() noexcept {
        std::cout << stream_.str() << std::endl;
    }

    /**
     * @brief 追加任意可流式输出的日志内容
     * @tparam T 可写入 `std::ostringstream` 的类型
     * @param value 待追加的日志字段
     * @return 当前日志对象引用，便于链式输出
     * @exception 若 `operator<<` 在内部流上失败，则由流状态反映，不主动抛出异常
     */
    template <typename T>
    LogMessage& operator<<(const T& value) {
        stream_ << value;
        return *this;
    }

private:
    std::ostringstream stream_;
};

} // namespace gnss

#define LOG(level) ::gnss::LogMessage(#level)

#endif // GNSS_POSITIONING_LOGGING_H
