// #include <iomanip>
// #include <iostream>
// #include <thread>

// #include "network/Network.h"
// #include "rby1-sdk/log.h"
// #include "rby1-sdk/model.h"
// #include "rby1-sdk/robot.h"
// #include <QApplication>
// using namespace rb;
// using namespace std::chrono_literals;

// Network* network;

// int main(int argc, char** argv) {
//   if (argc < 2) {
//     std::cerr << "Usage: " << argv[0] << " <server address>" << std::endl;
//     return 1;
//   }

//   std::string address{argv[1]};
//   std::cout << "Initializing robot at address: " << address << std::endl;

//   auto robot = Robot<y1_model::A>::Create(address);

//   std::cout << "Attempting to connect to the robot..." << std::endl;
//   if (!robot->Connect()) {
//     std::cerr << "Error: Unable to establish connection to the robot at " << address << std::endl;
//     return 1;
//   }
//   std::cout << "Successfully connected to the robot." << std::endl;

//   std::cout << "Get 5 last logs" << std::endl;
//   const auto& logs = robot->GetLastLog(5);
//   for (const auto& log : logs) {
//     std::cout << log << std::endl;
//   }

//   std::cout << "Starting log streaming ..." << std::endl;
//   robot->StartLogStream(
//       [](const std::vector<Log>& logs) {
//         for (const auto& log : logs) {
//           std::cout << log << std::endl;
//         }
//       },
//       1.0);

  
  
//   robot->StopLogStream();
//   QApplication a(argc, argv);
//   network = new Network();
  
//   return a.exec();
// }

#include <iostream>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>

using boost::asio::ip::udp;
using json = nlohmann::json;

int main() {
    try {
        boost::asio::io_service io_service;

        // UDP 포트 5005에서 수신 대기
        udp::socket socket(io_service, udp::endpoint(udp::v4(), 5005));

        std::cout << "UDP 포트 5005에서 수신 대기 중..." << std::endl;

        while (true) {
            std::array<char, 4096> recv_buf;
            udp::endpoint remote_endpoint;
            boost::system::error_code error;

            size_t len = socket.receive_from(boost::asio::buffer(recv_buf), remote_endpoint, 0, error);

            if (error && error != boost::asio::error::message_size) {
                throw boost::system::system_error(error);
            }

            std::string data(recv_buf.data(), len);
            // JSON 파싱
            try {
                json received = json::parse(data);
                std::cout << "수신된 데이터: " << remote_endpoint.address().to_string() << ":" << remote_endpoint.port() << std::endl;
                std::cout << "타임스탬프: " << received["timestamp"] << std::endl;

                std::cout << "Axes:" << std::endl;
                for (auto& [key, value] : received["axes"].items()) {
                    std::cout << "  " << key << ": " << value << std::endl;
                }

                std::cout << "Buttons:" << std::endl;
                for (auto& [key, value] : received["buttons"].items()) {
                    std::cout << "  " << key << ": " << value << std::endl;
                }

                std::cout << "Hats:" << std::endl;
                for (auto& [key, value] : received["hats"].items()) {
                    std::cout << "  " << key << ": (" << value[0] << ", " << value[1] << ")" << std::endl;
                }

                std::cout << "--------------------------" << std::endl;
            }
            catch (json::parse_error& e) {
                std::cerr << "JSON 파싱 오류: " << e.what() << std::endl;
            }
        }

    }
    catch (std::exception& e) {
        std::cerr << "예외 발생: " << e.what() << std::endl;
    }

    return 0;
}