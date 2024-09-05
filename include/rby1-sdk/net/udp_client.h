#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>

namespace rb {

class UdpClient {
 public:
  UdpClient(const std::string& server_ip, int port) {
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      throw std::runtime_error("Create udp socket failed");
    }
    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port);
    inet_pton(AF_INET, server_ip.c_str(), &server_addr_.sin_addr);

    fcntl(fd_, F_SETFL, O_NONBLOCK);  // Non-blocking
  }

  int RecvFrom(unsigned char* buffer, size_t max_buf_size) const {
    struct sockaddr_in server_addr {};

    socklen_t addr_len = sizeof(server_addr);
    return (int)recvfrom(fd_, buffer, max_buf_size, 0, (struct sockaddr*)&server_addr, &addr_len);
  }

  void SendTo(unsigned char* buffer, size_t buf_size) const {
    sendto(fd_, buffer, buf_size, 0, (struct sockaddr*)&server_addr_, sizeof(server_addr_));
  }

 private:
  struct sockaddr_in server_addr_ {};

  int fd_;
};

}  // namespace rb