#include "gnss_positioning/SocketStream.h"

#include <sstream>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

namespace gnss {

namespace {

std::string FormatSocketError(const char* stage, int error_code) {
    std::ostringstream oss;
    oss << stage << " failed with WSA error " << error_code;
    return oss.str();
}

} // namespace

SocketClient::~SocketClient() {
    close();
}

bool SocketClient::open(const std::string& ip,
                        unsigned short port,
                        int timeout_ms,
                        std::string& error_message) {
    close();
    error_message.clear();

    WSADATA wsa_data{};
    const int startup_ret = WSAStartup(MAKEWORD(2, 2), &wsa_data);
    if (startup_ret != 0) {
        error_message = FormatSocketError("WSAStartup", startup_ret);
        return false;
    }
    wsa_started_ = true;

    SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET) {
        error_message = FormatSocketError("socket", WSAGetLastError());
        close();
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
        error_message = "inet_pton failed: invalid IPv4 address '" + ip + "'.";
        closesocket(sock);
        close();
        return false;
    }

    if (connect(sock, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
        error_message = FormatSocketError("connect", WSAGetLastError());
        closesocket(sock);
        close();
        return false;
    }

    if (timeout_ms > 0) {
        const DWORD timeout = static_cast<DWORD>(timeout_ms);
        if (setsockopt(sock,
                       SOL_SOCKET,
                       SO_RCVTIMEO,
                       reinterpret_cast<const char*>(&timeout),
                       static_cast<int>(sizeof(timeout))) == SOCKET_ERROR) {
            error_message = FormatSocketError("setsockopt(SO_RCVTIMEO)", WSAGetLastError());
            closesocket(sock);
            close();
            return false;
        }
    }

    socket_handle_ = static_cast<std::uintptr_t>(sock);
    return true;
}

SocketReadStatus SocketClient::receive(uint8_t* buffer,
                                       std::size_t capacity,
                                       std::size_t& bytes_received,
                                       std::string& error_message) {
    bytes_received = 0;
    error_message.clear();

    if (buffer == nullptr || capacity == 0) {
        error_message = "recv failed: buffer is null or capacity is zero.";
        return SocketReadStatus::FAILED;
    }
    if (!isOpen()) {
        error_message = "recv failed: socket is not open.";
        return SocketReadStatus::FAILED;
    }

    SOCKET sock = static_cast<SOCKET>(socket_handle_);
    const int len = recv(sock, reinterpret_cast<char*>(buffer), static_cast<int>(capacity), 0);
    if (len > 0) {
        bytes_received = static_cast<std::size_t>(len);
        return SocketReadStatus::OK;
    }
    if (len == 0) {
        error_message = "remote peer closed the connection.";
        close();
        return SocketReadStatus::CLOSED;
    }

    const int error_code = WSAGetLastError();
    if (error_code == WSAETIMEDOUT) {
        error_message = "recv timeout.";
        return SocketReadStatus::TIMEOUT;
    }

    error_message = FormatSocketError("recv", error_code);
    close();
    return SocketReadStatus::FAILED;
}

void SocketClient::close() {
    if (socket_handle_ != kInvalidHandle) {
        closesocket(static_cast<SOCKET>(socket_handle_));
        socket_handle_ = kInvalidHandle;
    }
    if (wsa_started_) {
        WSACleanup();
        wsa_started_ = false;
    }
}

} // namespace gnss
