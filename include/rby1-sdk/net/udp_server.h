#pragma once

#include <stdexcept>
#include <string>
#include <utility>
#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace rb {

class UdpServer {
 public:
  explicit UdpServer(int port = 0) {
#if defined(_WIN32)
    int startup_result = WSAStartup(MAKEWORD(2, 2), &wsa_data_);
    if (startup_result) {
      std::stringstream ss;
      ss << "Cannot start WinSock (error: " << startup_result << ")";
      throw std::runtime_error(ss.str());
    }
#endif

    fd_ = socket(AF_INET6, SOCK_DGRAM, 0);
#if defined(_WIN32)
    if (fd_ == INVALID_SOCKET) {
      WSACleanup();
#else
    if (fd_ < 0) {
#endif
      throw std::runtime_error("Create udp socket failed");
    }

    int option = 0;
    if (setsockopt(fd_, IPPROTO_IPV6, IPV6_V6ONLY, (const char*)&option, sizeof(option)) == -1) {
#if defined(_WIN32)
      closesocket(fd_);
      WSACleanup();
#else
      close(fd_);
#endif
      throw std::runtime_error("Failed to set IPV6_V6ONLY");
    }

    struct sockaddr_in6 server_addr{};

    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin6_family = AF_INET6;
    server_addr.sin6_addr = in6addr_any;
    server_addr.sin6_port = htons(port);

    if (bind(fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
#if defined(_WIN32)
      closesocket(fd_);
      WSACleanup();
#else
      close(fd_);
#endif
      throw std::runtime_error("Bind failed");
    }

    // Get the port number
    struct sockaddr_in6 bound_addr{};

    socklen_t bound_addr_len = sizeof(bound_addr);
    if (getsockname(fd_, (struct sockaddr*)&bound_addr, &bound_addr_len) == -1) {
#if defined(_WIN32)
      shutdown(fd_, SD_BOTH);
      closesocket(fd_);
      WSACleanup();
#else
      shutdown(fd_, SHUT_RDWR);
      close(fd_);
#endif
      throw std::runtime_error("Failed to get socket name");
    }
    port_ = ntohs(bound_addr.sin6_port);

    // Non-blocking
#if defined(_WIN32)
    u_long mode = 1;
    if (ioctlsocket(fd_, FIONBIO, &mode) != NO_ERROR) {
      shutdown(fd_, SD_BOTH);
      closesocket(fd_);
      WSACleanup();
#else
    if (fcntl(fd_, F_SETFL, fcntl(fd_, F_GETFL) | O_NONBLOCK) < 0) {
      shutdown(fd_, SHUT_RDWR);
      close(fd_);
#endif
      throw std::runtime_error("failed to pu the socket in non-blocking mode");
    }
  }

  ~UdpServer() {
#if defined(_WIN32)
    shutdown(fd_, SD_BOTH);
    closesocket(fd_);
    WSACleanup();
#else
    shutdown(fd_, SHUT_RDWR);
    close(fd_);
#endif
  }

  int GetPort() const { return port_; }

  void ClearBuffer() const {
    constexpr size_t kBufferSize = 1024;

    char buffer[kBufferSize];

    struct sockaddr_storage client_addr{};

    socklen_t addr_len = sizeof(client_addr);
#if defined(_WIN32)
    int flags = 0;
#else
    int flags = MSG_DONTWAIT;  // Non-blocking flag
#endif

    while (recvfrom(fd_, buffer, kBufferSize, flags, (struct sockaddr*)&client_addr, &addr_len) > 0) {
      // Do nothing after receiving data (clear buffer)
    }
  }

  int RecvFrom(unsigned char* buffer, size_t max_buf_size, struct sockaddr_storage& client_addr) const {
    socklen_t addr_len = sizeof(client_addr);
    return (int)recvfrom(fd_, (char*)buffer, max_buf_size, 0, (struct sockaddr*)&client_addr, &addr_len);
  }

  void SendTo(unsigned char* buffer, size_t buf_size, const struct sockaddr_storage& client_addr) const {
    sendto(fd_, (const char*)buffer, buf_size, 0, (struct sockaddr*)&client_addr, sizeof(client_addr));
  }

 private:
#if defined(_WIN32)
  WSADATA wsa_data_{};
  SOCKET fd_{};
#else
  int fd_{};
#endif
  int port_;
};

}  // namespace rb