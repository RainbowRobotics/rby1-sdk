#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace rb {

class UdpClient {
 public:
  virtual int RecvFrom(unsigned char* buffer, size_t max_buf_size) const = 0;

  virtual void SendTo(unsigned char* buffer, size_t buf_size) const = 0;
};

class UdpIPv4Client : public UdpClient {
 public:
  UdpIPv4Client(std::string ip, int port) : ip_(std::move(ip)), port_(port) {
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      throw std::runtime_error("Create udp socket failed");
    }

    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port_);
    if(inet_pton(AF_INET, ip_.c_str(), &server_addr_.sin_addr) <= 0) {
      close(fd_);
      throw std::runtime_error("Invalid IPv4 address / Address not supported");
    }

    fcntl(fd_, F_SETFL, O_NONBLOCK);  // Non-blocking
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

  int fd_;

  struct sockaddr_in server_addr_ {};
};

class UdpIPv6Client : public UdpClient {
 public:
  UdpIPv6Client(std::string ip, int port) : ip_(std::move(ip)), port_(port) {
    fd_ = socket(AF_INET6, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      throw std::runtime_error("Create udp socket failed");
    }

    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin6_family = AF_INET6;
    server_addr_.sin6_port = htons(port_);
    if (inet_pton(AF_INET6, ip_.c_str(), &server_addr_.sin6_addr) <= 0) {
      close(fd_);
      throw std::runtime_error("Invalid IPv6 address / Address not supported");
    }

    fcntl(fd_, F_SETFL, O_NONBLOCK);  // Non-blocking
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

  int fd_;

  struct sockaddr_in6 server_addr_ {};
};

}  // namespace rb