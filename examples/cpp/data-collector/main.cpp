#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <csignal>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

#include <highfive/H5Easy.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <zmq.hpp>
#include <zmq_addon.hpp>
#ifdef REALSENSE2
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#endif

#include "rby1-sdk/base/event_loop.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"
#include "rby1-sdk/upc/master_arm.h"
#include "utils.h"

using namespace rb;
using namespace cv;
using namespace std::chrono;
using namespace std::chrono_literals;

const double kFrequency = 50;   // (Hz) = Framerate
const std::string kAll = ".*";  // NOLINT
const std::string MASTER_ARM_DEV = "/dev/rby1_master_arm";
const std::string COMMAND_POWER_OFF = "power_off";
const std::string COMMAND_POWER_ON = "power_on";
const std::string COMMAND_SERVO_ON = "servo_on";
const std::string COMMAND_CONTROL_MANAGER_INIT = "init_control_manager";
const std::string COMMAND_READY_POSE = "ready_pose";
const std::string COMMAND_ZERO_POSE = "zero_pose";
const std::string COMMAND_STOP_MOTION = "stop_motion";
const std::string COMMAND_START_TELEOPERATION = "start_teleoperation";
const std::string COMMAND_START = "start";
const std::string COMMAND_STOP = "stop";
const unsigned int kRobotDOF = y1_model::A::kRobotDOF;
const unsigned int kGripperDOF = 2;

std::shared_ptr<Robot<y1_model::A>> robot{nullptr};
std::unique_ptr<rb::RobotCommandHandler<y1_model::A>> rc_handler{nullptr};
std::unique_ptr<rb::RobotCommandStreamHandler<y1_model::A>> rcs_handler{nullptr};
std::shared_ptr<rb::upc::MasterArm> master_arm{nullptr};

zmq::context_t zmq_ctx;
zmq::socket_t srv_sock;
zmq::socket_t pub_sock;

/**
 * IMAGE
 */
const unsigned int kWidth = 424, kHeight = 240;
const unsigned int kCompressionLevel = 0;
const std::size_t kCameraN = 3;
const std::map<std::string, std::size_t> rs_serials = {{"233522071632", 0}, {"218622270382", 1}, {"128422271273", 2}};
#ifdef REALSENSE2
rs2::context rs_ctx;
std::array<rs2::pipeline, kCameraN> rs_pipelines;
#endif
bool camera_error = false;

/**
 * HDF5
 */
std::shared_ptr<HighFive::File> record_file{nullptr};
std::array<std::unique_ptr<HighFive::DataSet>, kCameraN> record_depth_dataset;
std::array<std::unique_ptr<HighFive::DataSet>, kCameraN> record_rgb_dataset;
std::unique_ptr<HighFive::DataSet> record_action_dataset;
std::unique_ptr<HighFive::DataSet> record_qpos_dataset;
std::unique_ptr<HighFive::DataSet> record_qvel_dataset;
std::unique_ptr<HighFive::DataSet> record_torque_dataset;
std::unique_ptr<HighFive::DataSet> record_ft_dataset;
Statistics stat_duration;
/**
 * RECORDED DATA
 */
std::array<Mat, kCameraN> record_depth;
std::array<Mat, kCameraN> record_rgb;
Eigen::Vector<double, kRobotDOF + kGripperDOF> record_action;
Eigen::Vector<double, kRobotDOF + kGripperDOF> record_qpos;
Eigen::Vector<double, kRobotDOF + kGripperDOF> record_qvel;
Eigen::Vector<double, kRobotDOF + kGripperDOF> record_torque;
Eigen::Vector<double, 6 * 2> record_ft;
int data_len = 0;

EventLoop publisher_ev, service_ev, robot_ev, cm_ev, camera_ev, record_ev;

bool is_camera_failed{false};
bool is_started{false};

bool robot_connected{false};
bool camera_connected{false};
bool master_arm_connected{false};
bool power{false};
bool servo{false};
bool control_manager{false};
std::string control_manager_detail;

void SignalHandler(int signum);
void InitializeService();
void DoService();
void InitializePublisher();
void Publish();
void InitializeRobot(const std::string& address);
void StopMotion();
void GoPose(const Eigen::Vector<double, 20>& body, const Eigen::Vector<double, 2>& head, double minimum_time = 5.0);
bool IsMotionDone();
void RobotPowerOff();
void RobotPowerOn();
void RobotServoOn();
void RobotControlManagerInit();
void InitializeCamera();
void InitializeMasterArm();
void InitializeH5File(const std::string& name);
template <typename T>
void AddDataIntoDataSet(std::unique_ptr<HighFive::DataSet>& dataset, std::size_t len, T* data);
template <typename T>
HighFive::DataSet CreateDataSet(const std::shared_ptr<HighFive::File>& file, const std::string& name, size_t len,
                                int compression_level = 0);

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <rpc server address> <save folder path>" << std::endl;
    return 1;
  }
  std::string address{argv[1]};
  std::string save_folder_path{argv[2]};

  signal(SIGINT, SignalHandler);

  publisher_ev.DoTask([] { InitializePublisher(); });
  service_ev.DoTask([] { InitializeService(); });
  robot_ev.DoTask([=] { InitializeRobot(address); });
  camera_ev.DoTask([] { InitializeCamera(); });
  // InitializeMasterArm();

  publisher_ev.PushTask([] { master_arm_connected = (master_arm != nullptr); });

  publisher_ev.PushCyclicTask([] { Publish(); }, 100ms);
  service_ev.PushCyclicTask([] { DoService(); }, 1ms);
  robot_ev.PushCyclicTask(
      [] {
        bool p;
        if (robot) {
          p = robot->IsConnected();
        } else {
          p = false;
        }
        publisher_ev.PushTask([p] { robot_connected = p; });
      },
      100us);
  cm_ev.PushCyclicTask(
      [] {
        if (!robot) {
          return;
        }

        try {
          auto cm = robot->GetControlManagerState();
          publisher_ev.PushTask([=] {
            control_manager = cm.state == rb::ControlManagerState::State::kEnabled;
            switch (cm.state) {
              case ControlManagerState::State::kUnknown:
                control_manager_detail = "Unknown";
                break;
              case ControlManagerState::State::kIdle:
                control_manager_detail = "Idle";
                break;
              case ControlManagerState::State::kEnabled:
                control_manager_detail = "Enabled";
                switch (cm.control_state) {
                  case ControlManagerState::ControlState::kUnknown:
                    control_manager_detail += "(Unknown)";
                    break;
                  case ControlManagerState::ControlState::kIdle:
                    control_manager_detail += "(Idle)";
                    break;
                  case ControlManagerState::ControlState::kExecuting:
                    control_manager_detail += "(Executing)";
                    break;
                  case ControlManagerState::ControlState::kSwitching:
                    control_manager_detail += "(Switching)";
                    break;
                }
                break;
              case ControlManagerState::State::kMinorFault:
                control_manager_detail = "MinorFault";
                break;
              case ControlManagerState::State::kMajorFault:
                control_manager_detail = "MajorFault";
                break;
            }
          });
        } catch (...) {
          robot = nullptr;
        }
      },
      500ms);
  camera_ev.PushCyclicTask(
      [=] {
#ifndef REALSENSE2
        camera_error = true;
#endif
        publisher_ev.PushTask([=] { camera_connected = !camera_error; });
        if (camera_error) {
          service_ev.PushTask([] { is_camera_failed = camera_error; });
          std::this_thread::sleep_for(10ms);
          return;
        }

#ifdef REALSENSE2
        int id = 0;
        for (auto&& pipe : rs_pipelines) {
          rs2::frameset fs;
          if (pipe.poll_for_frames(&fs)) {
            rs2::frame color_frame = fs.get_color_frame();
            rs2::frame depth_frame = fs.get_depth_frame();
            // rs2::frame f = rs2::colorizer().process(color_frame)
            // rs2::frame f = rs2::colorizer().process(depth_frame)
            record_ev.PushTask(
                [rgb = Mat(Size(kWidth, kHeight), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP), id] {
                  record_rgb_init |= (1 << id);
                  record_rgb[id] = rgb;
                });
            record_ev.PushTask(
                [depth = Mat(Size(kWidth, kHeight), CV_16FC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP), id] {
                  record_depth_init |= (1 << id);
                  record_depth[id] = depth;
                });
          }
          id++;
        }
#endif
      },
      10us);
  record_ev.PushCyclicTask(
      [] {
        auto start = steady_clock::now();

        if (!record_file) {
          return;
        }

        for (int i = 0; i < kCameraN; i++) {
          AddDataIntoDataSet(record_depth_dataset[i], kWidth * kHeight, record_depth[i].data);
          AddDataIntoDataSet(record_rgb_dataset[i], kWidth * kHeight * 3, record_rgb[i].data);
        }
        AddDataIntoDataSet(record_action_dataset, kRobotDOF + kGripperDOF, record_action.data());
        AddDataIntoDataSet(record_qpos_dataset, kRobotDOF + kGripperDOF, record_qpos.data());
        AddDataIntoDataSet(record_qvel_dataset, kRobotDOF + kGripperDOF, record_qvel.data());
        AddDataIntoDataSet(record_ft_dataset, 6 * 2, record_ft.data());
        data_len++;

        auto end = steady_clock::now();

        double d = (double)duration_cast<nanoseconds>(end - start).count() / 1.e9;
        stat_duration.add(d);
        if (stat_duration.count() >= kFrequency) {
          std::cout << "[" << data_len << "] " << "Count: " << stat_duration.count()
                    << ", Avg: " << stat_duration.avg() * 1e3 << " ms, Min: " << stat_duration.min() * 1e3
                    << " ms, Max: " << stat_duration.max() * 1e3 << " ms" << std::endl;
          stat_duration.reset();
        }
      },
      microseconds((long)(1e6 /* 1 sec */ / kFrequency)));
  //  record_ev.PushTask([=] {
  //    if (record_file) {
  //      return;
  //    }
  //
  //    // Make File
  //    record_file = std::make_shared<HighFive::File>(save_folder_path + "/example.h5", HighFive::File::Overwrite);
  //
  //    // Create Dataset
  //    for (int i = 0; i < kCameraN; i++) {
  //      record_depth_dataset[i] = std::make_unique<HighFive::DataSet>(CreateDataSet<std::uint16_t>(
  //          record_file, "/observations/images/cam" + std::to_string(i) + "_depth", kWidth * kHeight, kCompressionLevel));
  //      record_depth[i] = Mat(Size(kWidth, kHeight), CV_16FC1);
  //
  //      record_rgb_dataset[i] = std::make_unique<HighFive::DataSet>(
  //          CreateDataSet<std::uint8_t>(record_file, "/observations/images/cam" + std::to_string(i) + "_rgb",
  //                                      kWidth * kHeight * 3, kCompressionLevel));
  //      record_rgb[i] = Mat(Size(kWidth, kHeight), CV_8UC3);
  //    }
  //    record_action_dataset =
  //        std::make_unique<HighFive::DataSet>(CreateDataSet<double>(record_file, "/action", kRobotDOF + kGripperDOF));
  //    record_qpos_dataset = std::make_unique<HighFive::DataSet>(
  //        CreateDataSet<double>(record_file, "/observations/qpos", kRobotDOF + kGripperDOF));
  //    record_qvel_dataset = std::make_unique<HighFive::DataSet>(
  //        CreateDataSet<double>(record_file, "/observations/qvel", kRobotDOF + kGripperDOF));
  //    record_ft_dataset =
  //        std::make_unique<HighFive::DataSet>(CreateDataSet<double>(record_file, "/observations/ft_sensor", 6 * 2));
  //
  //    data_len = 0;
  //  });

  service_ev.WaitForTasks();

  return 0;
}

void SignalHandler(int signum) {
  record_ev.DoTask([] {
    if (record_file) {
      record_file->flush();
      record_file.reset();
      record_file = nullptr;
    }
  });

  camera_ev.DoTask([] {
    if (!camera_error) {
      // TODO: close rs pipelines
    }

    camera_error = true;
  });

  std::cout << "Exit" << std::endl;

  _exit(EXIT_FAILURE);
  _exit(128 + signum);
  signal(SIGTERM, SIG_DFL);
  raise(SIGTERM);
}

void InitializeService() {
  srv_sock = zmq::socket_t(zmq_ctx, zmq::socket_type::router);
  srv_sock.bind("tcp://*:5000");
}

void DoService() {
  std::vector<zmq::message_t> recv_msgs;

  try {
    zmq::recv_result_t result = zmq::recv_multipart(srv_sock, std::back_inserter(recv_msgs), zmq::recv_flags::dontwait);
    if (recv_msgs.empty()) {
      return;
    }

    auto j = nlohmann::json::parse(recv_msgs[1].to_string());
    if (!j.contains("command")) {
      return;
    }

    std::string command = j["command"];
    std::cout << "[Service] command received: " << command << std::endl;

    if (command == COMMAND_READY_POSE) {
      Eigen::Vector<double, 20> body_ready;
      Eigen::Vector<double, 2> head_ready;
      body_ready << 0, 35, -70, 35, 0, 0, -30, -10, 0, -100, 0, 40, 0, -30, 10, 0, -100, 0, 40, 0;
      head_ready << 0, 0;
      body_ready *= M_PI / 180.;
      head_ready *= M_PI / 180.;

      robot_ev.PushTask([=] { GoPose(body_ready, head_ready, 5.0); });
    } else if (command == COMMAND_ZERO_POSE) {
      robot_ev.PushTask([=] { GoPose(Eigen::Vector<double, 20>::Zero(), Eigen::Vector<double, 2>::Zero(), 5.0); });
    } else if (command == COMMAND_POWER_OFF) {
      robot_ev.PushTask([=] { RobotPowerOff(); });
    } else if (command == COMMAND_POWER_ON) {
      robot_ev.PushTask([=] { RobotPowerOn(); });
    } else if (command == COMMAND_SERVO_ON) {
      robot_ev.PushTask([=] { RobotServoOn(); });
    } else if (command == COMMAND_CONTROL_MANAGER_INIT) {
      robot_ev.PushTask([=] { RobotControlManagerInit(); });
    } else if (command == COMMAND_STOP_MOTION) {
      robot_ev.PushTask([=] { StopMotion(); });
    }
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }
}

void InitializePublisher() {
  pub_sock = zmq::socket_t(zmq_ctx, zmq::socket_type::pub);
  pub_sock.bind("tcp://*:5001");
}

void Publish() {
  nlohmann::json j;

  j["robot"] = robot_connected;
  j["camera"] = camera_connected;
  j["master_arm"] = master_arm_connected;

  j["power"] = power;
  j["servo"] = servo;
  j["control_manager"]["state"] = control_manager;
  j["control_manager"]["detail"] = control_manager_detail;

  zmq::send_multipart(pub_sock, std::array<zmq::const_buffer, 2>{zmq::str_buffer("data"), zmq::buffer(j.dump())});
}

void InitializeRobot(const std::string& address) {
  robot = Robot<y1_model::A>::Create(address);

  std::cout << "Attempting to connect to the robot..." << std::endl;
  if (!robot->Connect()) {
    std::cerr << "Error: Unable to establish connection to the robot at " << address << std::endl;
    robot = nullptr;
    return;
  }
  std::cout << "Successfully connected to the robot." << std::endl;

  robot->StartStateUpdate(
      [](const rb::RobotState<rb::y1_model::A>& state) {
        record_ev.PushTask([p = state.position, v = state.velocity, ref_p = state.target_position,
                            ft_right_force = state.ft_sensor_right.force,
                            ft_right_torque = state.ft_sensor_right.torque, ft_left_force = state.ft_sensor_left.force,
                            ft_left_torque = state.ft_sensor_left.torque] {
          record_qpos.head<kRobotDOF>() = p;
          record_qvel.head<kRobotDOF>() = v;
          record_action.head<kRobotDOF>() = ref_p;
          record_ft.block(0, 0, 3, 1) = ft_right_torque;
          record_ft.block(3, 0, 3, 1) = ft_right_force;
          record_ft.block(6, 0, 3, 1) = ft_left_torque;
          record_ft.block(9, 0, 3, 1) = ft_left_force;
        });

        publisher_ev.PushTask([s = state] {
          power = true;
          for (const auto& p : s.power_states) {
            power &= (p.state == PowerState::State::kPowerOn);
          }

          servo = true;
          for (const auto& j : s.joint_states) {
            servo &= j.is_ready;
          }
        });
      },
      100 /* Hz */);
}

void GoPose(const Eigen::Vector<double, 20>& body, const Eigen::Vector<double, 2>& head, double minimum_time) {
  std::cout << "[Robot] goto: " << body.transpose() << ", " << head.transpose() << std::endl;

  if (!robot) {
    std::cerr << "[Robot] Robot is not initialized" << std::endl;
    return;
  }

  if (!IsMotionDone()) {
    std::cerr << "[Robot] Robot is already moving" << std::endl;
    return;
  }

  rc_handler = robot->SendCommand(RobotCommandBuilder().SetCommand(
      ComponentBasedCommandBuilder()
          .SetBodyCommand(JointPositionCommandBuilder().SetPosition(body).SetMinimumTime(minimum_time))
          .SetHeadCommand(JointPositionCommandBuilder().SetPosition(head).SetMinimumTime(minimum_time))));
}

void StopMotion() {
  if (rc_handler) {
    rc_handler->Cancel();
  }

  if (rcs_handler) {
    rcs_handler->Cancel();
  }
}

bool IsMotionDone() {
  if (rc_handler) {
    if (rc_handler->IsDone()) {
      rc_handler = nullptr;
      return true;
    }
    return false;
  }

  if (rcs_handler) {
    if (rcs_handler->IsDone()) {
      rcs_handler = nullptr;
      return true;
    }
    return false;
  }

  return true;
}

void RobotPowerOff() {
  if (!robot) {
    std::cerr << "[Robot] Robot is not initialized" << std::endl;
    return;
  }

  robot->PowerOff(kAll);
  std::cout << "Robot powered off." << std::endl;
}

void RobotPowerOn() {
  if (!robot) {
    std::cerr << "[Robot] Robot is not initialized" << std::endl;
    return;
  }

  std::cout << "Checking power status..." << std::endl;
  if (!robot->IsPowerOn(kAll)) {
    std::cout << "Power is currently OFF. Attempting to power on..." << std::endl;
    if (!robot->PowerOn(kAll)) {
      std::cerr << "Error: Failed to power on the robot." << std::endl;
      robot = nullptr;
      return;
    }
    std::cout << "Robot powered on successfully." << std::endl;
  } else {
    std::cout << "Power is already ON." << std::endl;
  }

  try {
    if (robot->IsPowerOn("48v")) {
      robot->SetToolFlangeOutputVoltage("left", 12);
      robot->SetToolFlangeOutputVoltage("right", 12);
    }
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    return;
  }
}

void RobotServoOn() {
  if (!robot) {
    std::cerr << "[Robot] Robot is not initialized" << std::endl;
    return;
  }

  std::cout << "Checking servo status..." << std::endl;
  if (!robot->IsServoOn(kAll)) {
    std::cout << "Servo is currently OFF. Attempting to activate servo..." << std::endl;
    if (!robot->ServoOn(kAll)) {
      std::cerr << "Error: Failed to activate servo." << std::endl;
      robot = nullptr;
      return;
    }
    std::cout << "Servo activated successfully." << std::endl;
  } else {
    std::cout << "Servo is already ON." << std::endl;
  }
}

void RobotControlManagerInit() {
  if (!robot) {
    std::cerr << "[Robot] Robot is not initialized" << std::endl;
    return;
  }

  const auto& control_manager_state = robot->GetControlManagerState();
  if (control_manager_state.state == ControlManagerState::State::kMajorFault ||
      control_manager_state.state == ControlManagerState::State::kMinorFault) {
    std::cerr << "Warning: Detected a "
              << (control_manager_state.state == ControlManagerState::State::kMajorFault ? "Major" : "Minor")
              << " Fault in the Control Manager." << std::endl;

    std::cout << "Attempting to reset the fault..." << std::endl;
    if (!robot->ResetFaultControlManager()) {
      std::cerr << "Error: Unable to reset the fault in the Control Manager." << std::endl;
      robot = nullptr;
      return;
    }
    std::cout << "Fault reset successfully." << std::endl;
  }
  std::cout << "Control Manager state is normal. No faults detected." << std::endl;

  std::cout << "Enabling the Control Manager..." << std::endl;
  if (!robot->EnableControlManager()) {
    std::cerr << "Error: Failed to enable the Control Manager." << std::endl;
    robot = nullptr;
    return;
  }
  std::cout << "Control Manager enabled successfully." << std::endl;
}

void InitializeCamera() {
#ifdef REALSENSE2
  std::size_t dev_id;
  std::vector<std::string> serials;
  for (auto&& dev : rs_ctx.query_devices()) {
    std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    serials.push_back(serial);
    if (rs_serials.find(serial) == rs_serials.end()) {
      std::cerr << "Unknown camera detected" << std::endl;
      camera_error = true;
      return;
    }
    dev_id = rs_serials.find(serial)->second;
    std::cout << "RealSense Serial Number[" << dev_id << "]: " << serial << std::endl;

    auto advanced_dev = dev.as<rs400::advanced_mode>();
    auto advanced_sensors = advanced_dev.query_sensors();

    bool depth_found = false;
    bool color_found = false;
    rs2::sensor depth_sensor;
    rs2::sensor color_sensor;
    for (auto&& sensor : advanced_sensors) {
      std::string module_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
      std::cout << module_name << std::endl;

      if (module_name == "Stereo Module") {
        depth_sensor = sensor;
        depth_found = true;
      } else if (module_name == "RGB Camera") {
        color_sensor = sensor;
        color_found = true;
      }
    }
    if (depth_found) {
      depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
      depth_sensor.set_option(RS2_OPTION_EXPOSURE, 8500);
      depth_sensor.set_option(RS2_OPTION_GAIN, 16);
      depth_sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 1);
      try {
        if (dev_id == 0) {
          depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 1);
        } else {
          depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 2);
        }
      } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
      }
    }
    if (color_found) {
      color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
      color_sensor.set_option(RS2_OPTION_EXPOSURE, 100);  // 1/10 ms (10)
      color_sensor.set_option(RS2_OPTION_GAIN, 64);
      color_sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 1);
    }

    rs2::pipeline pipe(rs_ctx);
    rs2::config cfg;
    cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_DEPTH, kWidth, kHeight, RS2_FORMAT_Z16, (int)kFrequency);
    cfg.enable_stream(RS2_STREAM_COLOR, kWidth, kHeight, RS2_FORMAT_RGB8, (int)kFrequency);
    pipe.start(cfg);
    rs_pipelines[dev_id] = pipe;
  }
#endif
}

void InitializeMasterArm() {
  try {
    // Latency timer setting
    upc::InitializeDevice(MASTER_ARM_DEV);
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    exit(1);
  }

  master_arm = std::make_shared<upc::MasterArm>(MASTER_ARM_DEV);

  master_arm->SetModelPath(MODELS_PATH "/master_arm/model.urdf");
  master_arm->SetControlPeriod(0.02);

  auto active_ids = master_arm->Initialize();
  if (active_ids.size() != upc::MasterArm::kDOF + 2) {
    exit(1);
  }

  bool init = false;
  Eigen::Vector<double, upc::MasterArm::kDOF / 2> q_right, q_left;
  q_right.setZero();
  q_left.setZero();
  master_arm->StartControl([&](const upc::MasterArm::State& state) {
    upc::MasterArm::ControlInput input;

    if (!init) {
      q_right = state.q_joint(Eigen::seq(0, 6));
      q_left = state.q_joint(Eigen::seq(7, 13));
      init = true;
    }

    if (state.button_right.button == 1) {
      input.target_operation_mode(Eigen::seq(0, 6)).setConstant(DynamixelBus::kCurrentControlMode);
      input.target_torque(Eigen::seq(0, 6)) = state.gravity_term(Eigen::seq(0, 6));
      q_right = state.q_joint(Eigen::seq(0, 6));
    } else {
      input.target_operation_mode(Eigen::seq(0, 6)).setConstant(DynamixelBus::kCurrentBasedPositionControlMode);
      input.target_position(Eigen::seq(0, 6)) = q_right;
    }

    if (state.button_left.button == 1) {
      input.target_operation_mode(Eigen::seq(7, 13)).setConstant(DynamixelBus::kCurrentControlMode);
      input.target_torque(Eigen::seq(7, 13)) = state.gravity_term(Eigen::seq(7, 13));
      q_left = state.q_joint(Eigen::seq(7, 13));
    } else {
      input.target_operation_mode(Eigen::seq(7, 13)).setConstant(DynamixelBus::kCurrentBasedPositionControlMode);
      input.target_position(Eigen::seq(7, 13)) = q_left;
    }
    return input;
  });
}

template <typename T>
void AddDataIntoDataSet(std::unique_ptr<HighFive::DataSet>& dataset, std::size_t len, T* data) {
  auto current_dims = dataset->getSpace().getDimensions();
  size_t current_rows = current_dims[0];
  size_t new_rows = current_rows + 1;
  dataset->resize({new_rows, len});
  dataset->select({current_rows, 0}, {1, len}).write(data);
}

template <typename T>
HighFive::DataSet CreateDataSet(const std::shared_ptr<HighFive::File>& file, const std::string& name, size_t len,
                                int compression_level) {
  std::vector<size_t> dims = {0, len};
  std::vector<size_t> max_dims = {HighFive::DataSpace::UNLIMITED, len};
  HighFive::DataSetCreateProps props;
  props.add(HighFive::Chunking(std::vector<hsize_t>{1, len}));
  if (compression_level != 0) {
    props.add(HighFive::Deflate(compression_level));
  }
  return file->createDataSet<T>(name, HighFive::DataSpace(dims, max_dims), props);
}