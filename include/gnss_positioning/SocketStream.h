#ifndef GNSS_POSITIONING_SOCKET_STREAM_H
#define GNSS_POSITIONING_SOCKET_STREAM_H

#include <cstddef>
#include <cstdint>
#include <string>

namespace gnss {

/**
 * @brief Socket 单次读取结果
 */
enum class SocketReadStatus {
    OK,
    TIMEOUT,
    CLOSED,
    FAILED
};

/**
 * @brief Windows TCP 客户端轻量封装
 *
 * 说明：
 * - 仅负责连接、接收与关闭，不参与协议解析；
 * - 主流程按块读取后，仍逐字节交给 `NovatelDecoder` 完成消息拼接与校验。
 */
class SocketClient {
public:
    SocketClient() = default;
    ~SocketClient();

    /**
     * @brief 连接远端 TCP 服务
     * @param ip 远端 IPv4 地址
     * @param port 远端端口
     * @param timeout_ms 接收超时，单位 ms；传入 0 表示阻塞接收
     * @param error_message 失败时写入错误描述
     * @return true 表示连接成功
     */
    bool open(const std::string& ip, unsigned short port, int timeout_ms, std::string& error_message);

    /**
     * @brief 接收一批字节
     * @param buffer 输出缓冲区首地址
     * @param capacity 缓冲区容量，单位字节
     * @param bytes_received 实际收到的字节数
     * @param error_message 超时或失败时写入错误描述
     * @return 读取状态枚举
     */
    SocketReadStatus receive(uint8_t* buffer,
                             std::size_t capacity,
                             std::size_t& bytes_received,
                             std::string& error_message);

    /**
     * @brief 主动关闭连接并释放 Winsock 资源
     * @return 无
     */
    void close();

    /**
     * @brief 判断当前连接是否处于打开状态
     * @return true 表示已连接
     */
    bool isOpen() const { return socket_handle_ != kInvalidHandle; }

private:
    static constexpr std::uintptr_t kInvalidHandle = static_cast<std::uintptr_t>(-1);

    std::uintptr_t socket_handle_ = kInvalidHandle;
    bool wsa_started_ = false;
};

} // namespace gnss

#endif // GNSS_POSITIONING_SOCKET_STREAM_H
