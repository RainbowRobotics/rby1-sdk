#include <iostream>
#include <boost/asio.hpp>
#include <vector>
#include <cstring>

using boost::asio::ip::udp;

int main() {
    try {
        boost::asio::io_service io_service;
        udp::socket socket(io_service, udp::endpoint(udp::v4(), 5005));

        std::cout << "Listening on UDP port 5005..." << std::endl;

        while (true) {
            std::array<char, 1024> recv_buf;
            udp::endpoint remote_endpoint;
            boost::system::error_code error;

            size_t len = socket.receive_from(boost::asio::buffer(recv_buf), remote_endpoint, 0, error);

            if (error && error != boost::asio::error::message_size) {
                throw boost::system::system_error(error);
            }

            // 데이터 구조
            int num_axes = 6; // 송신 측 축 개수
            int num_buttons = 11; // 송신 측 버튼 개수
            int num_hats_variables = 4; // hat 변수 4개 (왼쪽, 오른쪽, 위, 아래)

            float axes[num_axes];
            uint8_t buttons[num_buttons];
            uint8_t hat_variables[num_hats_variables]; // hat 데이터를 네 변수로 받기

            // 데이터 언패킹
            std::memcpy(axes, recv_buf.data(), num_axes * sizeof(float));
            std::memcpy(buttons, recv_buf.data() + num_axes * sizeof(float), num_buttons * sizeof(uint8_t));
            std::memcpy(hat_variables, recv_buf.data() + num_axes * sizeof(float) + num_buttons * sizeof(uint8_t), num_hats_variables * sizeof(uint8_t));

            // 데이터 출력
            std::cout << "Axes: ";
            for (int i = 0; i < num_axes; ++i) {
                std::cout << axes[i] << " ";
            }
            std::cout << std::endl;

            std::cout << "Buttons: ";
            for (int i = 0; i < num_buttons; ++i) {
                std::cout << static_cast<int>(buttons[i]) << " ";
            }
            std::cout << std::endl;

            // 데이터 출력
            std::cout << "Hats: ";
            std::cout << "Left: " << (int)hat_variables[0] << ", Right: " << (int)hat_variables[1]
                    << ", Up: " << (int)hat_variables[2] << ", Down: " << (int)hat_variables[3] << std::endl;

            std::cout << std::endl;

            std::cout << "--------------------------" << std::endl;
        }
    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return 0;
}