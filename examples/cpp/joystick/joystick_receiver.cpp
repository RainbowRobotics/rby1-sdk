#include "joystick_receiver.h"
#include <iostream>
#include <cstring>

JoystickReceiver::JoystickReceiver(int port, int num_axes, int num_buttons, int num_hats)
    : num_axes_(num_axes), num_buttons_(num_buttons), num_hats_(num_hats),
      stop_flag_(false), socket_(io_service_, udp::endpoint(udp::v4(), port)) {
    joystick_data_.axes.resize(num_axes_);
    joystick_data_.buttons.resize(num_buttons_);
    joystick_data_.hats.resize(4); // 네 방향으로 고정
}

JoystickReceiver::~JoystickReceiver() {
    stop_flag_ = true;
    if (receiver_thread_.joinable()) {
        receiver_thread_.join();
    }
}

void JoystickReceiver::start() {
    receiver_thread_ = std::thread(&JoystickReceiver::receiveLoop, this);
}

JoystickData JoystickReceiver::getJoystickData() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return joystick_data_;
}

void JoystickReceiver::receiveLoop() {
    try {
        std::array<char, 1024> recv_buf;
        udp::endpoint remote_endpoint;
        while (!stop_flag_) {
            boost::system::error_code error;
            size_t len = socket_.receive_from(boost::asio::buffer(recv_buf), remote_endpoint, 0, error);

            if (error && error != boost::asio::error::message_size) {
                std::cerr << "Receive error: " << error.message() << std::endl;
                continue;
            }

            // 데이터 언패킹
            float axes[num_axes_];
            uint8_t buttons[num_buttons_];
            uint8_t hats[4]; // 네 방향

            std::memcpy(axes, recv_buf.data(), num_axes_ * sizeof(float));
            std::memcpy(buttons, recv_buf.data() + num_axes_ * sizeof(float), num_buttons_ * sizeof(uint8_t));
            std::memcpy(hats, recv_buf.data() + num_axes_ * sizeof(float) + num_buttons_ * sizeof(uint8_t), 4 * sizeof(uint8_t));

            // 데이터 업데이트
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                joystick_data_.axes.assign(axes, axes + num_axes_);
                joystick_data_.buttons.assign(buttons, buttons + num_buttons_);
                joystick_data_.hats.assign(hats, hats + 4); // 네 방향 데이터
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception in receive loop: " << e.what() << std::endl;
    }
}
