#include <array>
#include <iostream>
#include <optional>
#include <thread>
#include "rby1-sdk/net/udp_client.h"
#include "rby1-sdk/net/udp_server.h"

using namespace std::chrono_literals;

int main() {
  auto thd1 = std::thread([] {
    std::array<unsigned char, 1024> msg{};
    rb::UdpServer server(50052);
    while (true) {
      struct sockaddr_in client_addr{};
      int len = server.RecvFrom(msg.data(), 1024, client_addr);
      if (len > 0) {
        std::cout << "[SERVER] [";
        for (int i = 0; i < len; i++) {
          if (i != 0)
            std::cout << ", ";
          std::cout << (int)msg[i];
        }
        std::cout << "]" << std::endl;
        server.SendTo(msg.data(), len, client_addr);
      }

      std::this_thread::sleep_for(1s);
    }
  });

  auto thd2 = std::thread([] {
    int count = 0;
    std::array<unsigned char, 1024> msg = {1, 2, 3, 4, 5, 6};
    rb::UdpClient client("localhost", 50052);
    while (true) {
      for (int i = 0; i < 6; i++) {
        msg[i] += count;
      }
      client.SendTo(msg.data(), 6);
      count++;

      std::this_thread::sleep_for(1s);
    }
  });

  thd1.join();
  thd2.join();

  return 0;
}