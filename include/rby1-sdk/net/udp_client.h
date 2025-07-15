#pragma once

#include <string>
#include <stdexcept>
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

class UdpClient {
 public:
  virtual int RecvFrom(unsigned char* buffer, size_t max_buf_size) const = 0;

  virtual void SendTo(unsigned char* buffer, size_t buf_size) const = 0;
};

class UdpIPv4Client : public UdpClient {
 public:
  UdpIPv4Client(std::string ip, int port) : ip_(std::move(ip)), port_(port) {
#if defined(_WIN32)
    int startup_result = WSAStartup(MAKEWORD(2, 2), &wsa_data_);
    if (startup_result) {
      std::stringstream ss;
      ss << "Cannot start WinSock (error: " << startup_result << ")";
      throw std::runtime_error(ss.str());
    }
#endif

    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
#if defined(_WIN32)
    if (fd_ == INVALID_SOCKET) {
      WSACleanup();
#else
    if (fd_ < 0) {
#endif
      throw std::runtime_error("Create udp socket failed");
    }

    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port_);
    if (inet_pton(AF_INET, ip_.c_str(), &server_addr_.sin_addr) <= 0) {
#if defined(_WIN32)
      closesocket(fd_);
      WSACleanup();
#else
      close(fd_);
#endif
      throw std::runtime_error("Invalid IPv4 address / Address not supported");
    }

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

  ~UdpIPv4Client() {
#if defined(_WIN32)
    shutdown(fd_, SD_BOTH);
    closesocket(fd_);
    WSACleanup();
#else
    shutdown(fd_, SHUT_RDWR);
    close(fd_);
#endif
  }

  int RecvFrom(unsigned char* buffer, size_t max_buf_size) const override {
    struct sockaddr_in server_addr {};

    socklen_t addr_len = sizeof(server_addr);
    return (int)recvfrom(fd_, buffer, max_buf_size, 0, (struct sockaddr*)&server_addr, &addr_len);
  }

  void SendTo(unsigned char* buffer, size_t buf_size) const override {
    sendto(fd_, buffer, buf_size, 0, (struct sockaddr*)&server_addr_, sizeof(server_addr_));
  }

 protected:
  std::string ip_;
  int port_;

#if defined(_WIN32)
  WSADATA wsa_data_{};
  SOCKET fd_{};
#else
  int fd_{};
#endif

  struct sockaddr_in server_addr_ {};
};

class UdpIPv6Client : public UdpClient {
 public:
  UdpIPv6Client(std::string ip, int port) : ip_(std::move(ip)), port_(port) {
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

    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin6_family = AF_INET6;
    server_addr_.sin6_port = htons(port_);
    if (inet_pton(AF_INET6, ip_.c_str(), &server_addr_.sin6_addr) <= 0) {
#if defined(_WIN32)
      closesocket(fd_);
      WSACleanup();
#else
      close(fd_);
#endif
      throw std::runtime_error("Invalid IPv6 address / Address not supported");
    }

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

  ~UdpIPv6Client() {
#if defined(_WIN32)
    shutdown(fd_, SD_BOTH);
    closesocket(fd_);
    WSACleanup();
#else
    shutdown(fd_, SHUT_RDWR);
    close(fd_);
#endif
  }

  int RecvFrom(unsigned char* buffer, size_t max_buf_size) const override {
    struct sockaddr_in6 server_addr {};

    socklen_t addr_len = sizeof(server_addr);
    return (int)recvfrom(fd_, buffer, max_buf_size, 0, (struct sockaddr*)&server_addr, &addr_len);
  }

  void SendTo(unsigned char* buffer, size_t buf_size) const override {
    sendto(fd_, buffer, buf_size, 0, (struct sockaddr*)&server_addr_, sizeof(server_addr_));
  }

 protected:
  std::string ip_;
  int port_;

#if defined(_WIN32)
  WSADATA wsa_data_{};
  SOCKET fd_{};
#else
  int fd_{};
#endif

  struct sockaddr_in6 server_addr_ {};
};

}  // namespace rb