#pragma once

#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
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
  explicit UdpServer(const int port = 0) {
#if defined(_WIN32)
    int startup_result = WSAStartup(MAKEWORD(2, 2), &wsa_data_);
    if (startup_result) {
      std::stringstream ss;
      ss << "Cannot start WinSock (error: " << startup_result << ")";
      throw std::runtime_error(ss.str());
    }
#endif

    // --- IPv4 socket ---
    fd_v4_ = socket(AF_INET, SOCK_DGRAM, 0);
#if defined(_WIN32)
    if (fd_v4_ == INVALID_SOCKET) {
#else
    if (fd_v4_ < 0) {
#endif
      FailAndCleanup("Create IPv4 UDP socket failed");
    }

    SetSockoptsV4();
    BindV4(port);

    // get chosen port (when port == 0)
    {
      sockaddr_in bound4{};
      socklen_t blen4 = sizeof(bound4);
      if (getsockname(fd_v4_, (sockaddr*)&bound4, &blen4) < 0) {
        FailAndCleanup("Failed to get IPv4 socket name");
      }
      port_ = ntohs(bound4.sin_port);
    }

    // --- IPv6 socket ---
    fd_v6_ = socket(AF_INET6, SOCK_DGRAM, 0);
#if defined(_WIN32)
    if (fd_v6_ == INVALID_SOCKET) {
#else
    if (fd_v6_ < 0) {
#endif
      FailAndCleanup("Create IPv6 UDP socket failed");
    }

    SetSockoptsV6();

    {
      const int on = 1;
      setsockopt(fd_v6_, IPPROTO_IPV6, IPV6_V6ONLY, (const char*)&on, sizeof(on));
    }

    BindV6(port_);

    // --- Non-blocking ---
    MakeNonBlocking(fd_v4_);
    MakeNonBlocking(fd_v6_);

#if defined(_WIN32)
    DWORD no = FALSE;
    WSAIoctl(fd_v4_, SIO_UDP_CONNRESET, &no, sizeof(no), nullptr, 0, &no, nullptr, nullptr);
    WSAIoctl(fd_v6_, SIO_UDP_CONNRESET, &no, sizeof(no), nullptr, 0, &no, nullptr, nullptr);
#endif
  }

  ~UdpServer() { CloseAll(); }

  int GetPort() const { return port_; }

  int RecvFrom(unsigned char* buffer, size_t max_buf_size, struct sockaddr_storage& client_addr) const {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd_v4_, &readfds);
    FD_SET(fd_v6_, &readfds);
#if defined(_WIN32)
    SOCKET max_fd = (fd_v4_ > fd_v6_) ? fd_v4_ : fd_v6_;
#else
    const int max_fd = (fd_v4_ > fd_v6_) ? fd_v4_ : fd_v6_;
#endif

    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 1000;  // 1 ms

    if (const int ready = select((int)max_fd + 1, &readfds, nullptr, nullptr, &tv); ready <= 0) {
      return 0;
    }

    if (FD_ISSET(fd_v4_, &readfds)) {
      socklen_t len = sizeof(client_addr);
      return (int)recvfrom(fd_v4_, (char*)buffer, max_buf_size, 0, (sockaddr*)&client_addr, &len);
    }
    if (FD_ISSET(fd_v6_, &readfds)) {
      socklen_t len = sizeof(client_addr);
      return (int)recvfrom(fd_v6_, (char*)buffer, max_buf_size, 0, (sockaddr*)&client_addr, &len);
    }
    return 0;
  }

  void SendTo(unsigned char* buffer, size_t buf_size, const struct sockaddr_storage& client_addr) const {
    if (client_addr.ss_family == AF_INET) {
      const socklen_t addrlen = (socklen_t)sizeof(sockaddr_in);
      sendto(fd_v4_, (const char*)buffer, buf_size, 0, (const sockaddr*)&client_addr, addrlen);
    } else if (client_addr.ss_family == AF_INET6) {
      const socklen_t addrlen = (socklen_t)sizeof(sockaddr_in6);
      sendto(fd_v6_, (const char*)buffer, buf_size, 0, (const sockaddr*)&client_addr, addrlen);
    } else {
      // Ignore unsupported family
    }
  }

  void ClearBuffer() const {
    constexpr size_t kBufferSize = 1024;
    unsigned char buffer[kBufferSize];

    auto drain = [&](int fd) {
      for (;;) {
        sockaddr_storage tmp{};
        socklen_t len = sizeof(tmp);
#if defined(_WIN32)
        int flags = 0;
#else
        int flags = MSG_DONTWAIT;
#endif
        int r = (int)recvfrom(fd, (char*)buffer, kBufferSize, flags, (sockaddr*)&tmp, &len);
        if (r <= 0)
          break;
      }
    };
    drain(fd_v4_);
    drain(fd_v6_);
  }

 private:
  void FailAndCleanup(const char* msg) const {
    CloseAll();
    throw std::runtime_error(msg);
  }

  void CloseAll() const {
#if defined(_WIN32)
    if (fd_v4_ != INVALID_SOCKET) {
      shutdown(fd_v4_, SD_BOTH);
      closesocket(fd_v4_);
    }
    if (fd_v6_ != INVALID_SOCKET) {
      shutdown(fd_v6_, SD_BOTH);
      closesocket(fd_v6_);
    }
    WSACleanup();
#else
    if (fd_v4_ >= 0) {
      shutdown(fd_v4_, SHUT_RDWR);
      close(fd_v4_);
    }
    if (fd_v6_ >= 0) {
      shutdown(fd_v6_, SHUT_RDWR);
      close(fd_v6_);
    }
#endif
  }

  void SetSockoptsV4() {
    const int on = 1;
    setsockopt(fd_v4_, SOL_SOCKET, SO_REUSEADDR, (const char*)&on, sizeof(on));
#ifdef SO_REUSEPORT
    setsockopt(fd_v4_, SOL_SOCKET, SO_REUSEPORT, (const char*)&on, sizeof(on));
#endif
  }

  void SetSockoptsV6() {
    const int on = 1;
    setsockopt(fd_v6_, SOL_SOCKET, SO_REUSEADDR, (const char*)&on, sizeof(on));
#ifdef SO_REUSEPORT
    setsockopt(fd_v6_, SOL_SOCKET, SO_REUSEPORT, (const char*)&on, sizeof(on));
#endif
  }

  void BindV4(int port) {
    sockaddr_in addr4{};
    addr4.sin_family = AF_INET;
    addr4.sin_addr.s_addr = htonl(INADDR_ANY);
    addr4.sin_port = htons(port);
    if (bind(fd_v4_, (sockaddr*)&addr4, sizeof(addr4)) < 0) {
      FailAndCleanup("Bind failed for IPv4 socket");
    }
  }

  void BindV6(int port_same_as_v4) {
    sockaddr_in6 addr6{};
    std::memset(&addr6, 0, sizeof(addr6));
    addr6.sin6_family = AF_INET6;
    addr6.sin6_addr = in6addr_any;
    addr6.sin6_port = htons(port_same_as_v4);
    if (bind(fd_v6_, (sockaddr*)&addr6, sizeof(addr6)) < 0) {
      FailAndCleanup("Bind failed for IPv6 socket");
    }
  }

  static void MakeNonBlocking(
#if defined(_WIN32)
      SOCKET fd
#else
      int fd
#endif
  ) {
#if defined(_WIN32)
    u_long mode = 1;
    ioctlsocket(fd, FIONBIO, &mode);
#else
    if (const int flags = fcntl(fd, F_GETFL, 0); flags >= 0) {
      fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
#endif
  }

#if defined(_WIN32)
  WSADATA wsa_data_{};
  SOCKET fd_v4_{INVALID_SOCKET};
  SOCKET fd_v6_{INVALID_SOCKET};
#else
  int fd_v4_{-1};
  int fd_v6_{-1};
#endif
  int port_{0};
};

}  // namespace rb
