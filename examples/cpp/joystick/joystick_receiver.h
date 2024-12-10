#ifndef JOYSTICK_RECEIVER_H
#define JOYSTICK_RECEIVER_H

#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include <vector>

using boost::asio::ip::udp;

struct JoystickData {
    std::vector<float> axes;
    std::vector<uint8_t> buttons;
    std::vector<uint8_t> hats; // [Left, Right, Up, Down]
};

class JoystickReceiver {
public:
    JoystickReceiver(int port, int num_axes, int num_buttons, int num_hats);
    ~JoystickReceiver();

    void start(); // Start receiving data in a thread
    JoystickData getJoystickData(); // Get the latest joystick data

private:
    void receiveLoop(); // Loop to receive data

    int num_axes_;
    int num_buttons_;
    int num_hats_;
    std::atomic<bool> stop_flag_;
    std::thread receiver_thread_;
    std::mutex data_mutex_;
    JoystickData joystick_data_;
    boost::asio::io_service io_service_;
    udp::socket socket_;
};

#endif // JOYSTICK_RECEIVER_H