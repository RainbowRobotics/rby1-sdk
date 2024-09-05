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
  explicit UdpServer(int port) {
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      throw std::runtime_error("Create udp socket failed");
    }

    struct sockaddr_in server_addr {};

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    if (bind(fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
      throw std::runtime_error("Bind failed");
    }

    fcntl(fd_, F_SETFL, O_NONBLOCK);  // Non-blocking
  }

  void ClearBuffer() const {
    constexpr size_t kBufferSize = 1024;

    char buffer[kBufferSize];

    struct sockaddr_in client_addr {};

    socklen_t addr_len = sizeof(client_addr);
    int flags = MSG_DONTWAIT;  // Non-blocking flag

    while (recvfrom(fd_, buffer, kBufferSize, flags, (struct sockaddr*)&client_addr, &addr_len) > 0) {
      // 데이터 수신 후 아무것도 하지 않음 (버퍼 비움)
    }
  }

  int RecvFrom(unsigned char* buffer, size_t max_buf_size, struct sockaddr_in& client_addr) const {
    socklen_t addr_len = sizeof(client_addr);
    return recvfrom(fd_, buffer, max_buf_size, 0, (struct sockaddr*)&client_addr, &addr_len);
  }

  void SendTo(unsigned char* buffer, size_t buf_size, const struct sockaddr_in& client_addr) const {
    sendto(fd_, buffer, buf_size, 0, (struct sockaddr*)&client_addr, sizeof(client_addr));
  }

 private:
  int fd_;
};

}  // namespace rb