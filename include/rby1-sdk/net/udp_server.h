#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>

namespace rb {

class UdpServer {
 public:
  explicit UdpServer(int port = 0) {
    fd_ = socket(AF_INET6, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      throw std::runtime_error("Create udp socket failed");
    }

    int option = 0;
    if (setsockopt(fd_, IPPROTO_IPV6, IPV6_V6ONLY, &option, sizeof(option)) == -1) {
      close(fd_);
      throw std::runtime_error("Failed to set IPV6_V6ONLY");
    }

    struct sockaddr_in6 server_addr {};

    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin6_family = AF_INET6;
    server_addr.sin6_addr = in6addr_any;
    server_addr.sin6_port = htons(port);

    if (bind(fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
      throw std::runtime_error("Bind failed");
    }

    // Get the port number
    struct sockaddr_in6 bound_addr {};

    socklen_t bound_addr_len = sizeof(bound_addr);
    if (getsockname(fd_, (struct sockaddr*)&bound_addr, &bound_addr_len) == -1) {
      close(fd_);
      throw std::runtime_error("Failed to get socket name");
    }
    port_ = ntohs(bound_addr.sin6_port);

    fcntl(fd_, F_SETFL, O_NONBLOCK);  // Non-blocking
  }

  int GetPort() const { return port_; }

  void ClearBuffer() const {
    constexpr size_t kBufferSize = 1024;

    char buffer[kBufferSize];

    struct sockaddr_storage client_addr {};

    socklen_t addr_len = sizeof(client_addr);
    int flags = MSG_DONTWAIT;  // Non-blocking flag

    while (recvfrom(fd_, buffer, kBufferSize, flags, (struct sockaddr*)&client_addr, &addr_len) > 0) {
      // 데이터 수신 후 아무 것도 하지 않음 (버퍼 비움)
    }
  }

  int RecvFrom(unsigned char* buffer, size_t max_buf_size, struct sockaddr_storage& client_addr) const {
    socklen_t addr_len = sizeof(client_addr);
    return (int)recvfrom(fd_, buffer, max_buf_size, 0, (struct sockaddr*)&client_addr, &addr_len);
  }

  void SendTo(unsigned char* buffer, size_t buf_size, const struct sockaddr_storage& client_addr) const {
    sendto(fd_, buffer, buf_size, 0, (struct sockaddr*)&client_addr, sizeof(client_addr));
  }

 private:
  int fd_;
  int port_;
};

}  // namespace rb