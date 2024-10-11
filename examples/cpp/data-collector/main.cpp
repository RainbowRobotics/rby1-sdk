#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <iostream>
#include <map>
#include <memory>
#include <vector>

#include <highfive/H5Easy.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <zmq.hpp>
#include <zmq_addon.hpp>

#include "rby1-sdk/base/event_loop.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"
#include "rby1-sdk/upc/master_arm.h"

using namespace rb;
using namespace cv;
using namespace std::chrono_literals;

const std::string RPC_ADDRESS = "192.168.30.1:50051";
const std::string MASTER_ARM_DEV = "/dev/rby1_master_arm";
const std::string COMMAND_READY_POSE = "ready_pose";
const std::string COMMAND_ZERO_POSE = "zero_pose";
const std::string COMMAND_STOP_MOTION = "stop_motion";
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

rs2::context rs_ctx;
const std::size_t kCameraN = 3;
const std::map<std::string, std::size_t> rs_serials = {{"233522071632", 0}, {"218622270382", 1}, {"128422271273", 2}};
std::array<rs2::pipeline, kCameraN> rs_pipelines;
bool camera_error = false;
const unsigned int kWidth = 424, kHeight = 240;

std::shared_ptr<HighFive::File> record_file;
std::array<HighFive::DataSet, kCameraN> record_depth_dataset;
std::array<HighFive::DataSet, kCameraN> record_rgb_dataset;
HighFive::DataSet record_qpos_dataset, record_qvel_dataset, record_ft_dataset,
    record_action_dataset;
unsigned int record_depth_init = 0;
unsigned int record_rgb_init = 0;
std::array<Mat, kCameraN> record_depth;
std::array<Mat, kCameraN> record_rgb;
Eigen::Vector<double, kRobotDOF + kGripperDOF> record_qpos, record_qvel, record_action;
Eigen::Vector<double, 6 * 2> record_ft;

EventLoop service_ev, robot_ev, camera_ev, record_ev;

bool is_connected{false};
bool is_motion_done{false};
bool is_camera_failed{false};
bool is_started{false};

void InitializeService();
void DoService();
void InitializeRobot();
void StopMotion();
void GoPose(const Eigen::Vector<double, 20>& body, const Eigen::Vector<double, 2>& head, double minimum_time = 5.0);
bool IsMotionDone();
void InitializeCamera();
void InitializeMasterArm();

void signalHandler(int signum) {
  record_ev.DoTask([] {
    if (record_file) {
      record_file->flush();
      record_file.reset();
      record_file = nullptr;
    }
  });
  std::cout << "Exit" << std::endl;

  _exit(EXIT_FAILURE);
  _exit(128 + signum);
  signal(SIGTERM, SIG_DFL);
  raise(SIGTERM);
}

int main() {
  signal(SIGINT, signalHandler);

  service_ev.DoTask([] { InitializeService(); });
  robot_ev.DoTask([] { InitializeRobot(); });
  camera_ev.DoTask([] { InitializeCamera(); });
  // InitializeMasterArm();

  service_ev.PushLoopTask([] { DoService(); });
  robot_ev.PushLoopTask([] {
    bool _is_connected, _is_motion_done;

    if (!robot) {
      _is_connected = false;
      _is_motion_done = true;
    } else {
      _is_connected = true;
      _is_motion_done = IsMotionDone();
    }

    service_ev.PushTask([=] {
      is_connected = _is_connected;
      is_motion_done = _is_motion_done;
    });
  });
  camera_ev.PushLoopTask([=] {
    if (camera_error) {
      service_ev.PushTask([] { is_camera_failed = camera_error; });
      std::this_thread::sleep_for(10ms);
      return;
    }

    int id = 0;
    for (auto&& pipe : rs_pipelines) {
      rs2::frameset fs;
      if (pipe.poll_for_frames(&fs)) {
        rs2::frame color_frame = fs.get_color_frame();
        rs2::frame depth_frame = fs.get_depth_frame();
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
  });
  record_ev.PushCyclicTask(
      [] {
        if (!record_file) {
          return;
        }

        if (record_rgb_init != (1 << kCameraN) - 1 || record_depth_init != (1 << kCameraN) - 1) {
          return;
        }

        auto start = std::chrono::steady_clock::now();

        for (int i = 0; i < kCameraN; i++) {
          auto current_dims = record_depth_dataset[i].getSpace().getDimensions();
          size_t current_rows = current_dims[0];
          size_t new_rows = current_rows + 1;
          record_depth_dataset[i].resize({new_rows, kWidth * kHeight});
          record_depth_dataset[i].select({current_rows, 0}, {1, kWidth * kHeight}).write(record_depth[i].data);
        }

        for (int i = 0; i < kCameraN; i++) {
          auto current_dims = record_rgb_dataset[i].getSpace().getDimensions();
          size_t current_rows = current_dims[0];
          size_t new_rows = current_rows + 1;
          record_rgb_dataset[i].resize({new_rows, kWidth * kHeight * 3});
          record_rgb_dataset[i].select({current_rows, 0}, {1, kWidth * kHeight * 3}).write(record_rgb[i].data);
        }

        {
          auto current_dims = record_action_dataset.getSpace().getDimensions();
          size_t current_rows = current_dims[0];
          size_t new_rows = current_rows + 1;
          record_action_dataset.resize({new_rows, kRobotDOF + kGripperDOF});
          record_action_dataset.select({current_rows, 0}, {1, kRobotDOF + kGripperDOF}).write(record_action.data());
        }

        {
          auto current_dims = record_qpos_dataset.getSpace().getDimensions();
          size_t current_rows = current_dims[0];
          size_t new_rows = current_rows + 1;
          record_qpos_dataset.resize({new_rows, kRobotDOF + kGripperDOF});
          record_qpos_dataset.select({current_rows, 0}, {1, kRobotDOF + kGripperDOF}).write(record_qpos.data());
        }

        {
          auto current_dims = record_qvel_dataset.getSpace().getDimensions();
          size_t current_rows = current_dims[0];
          size_t new_rows = current_rows + 1;
          record_qvel_dataset.resize({new_rows, kRobotDOF + kGripperDOF});
          record_qvel_dataset.select({current_rows, 0}, {1, kRobotDOF + kGripperDOF}).write(record_qvel.data());
        }

        {
          auto current_dims = record_ft_dataset.getSpace().getDimensions();
          size_t current_rows = current_dims[0];
          size_t new_rows = current_rows + 1;
          record_ft_dataset.resize({new_rows, 6 * 2});
          record_ft_dataset.select({current_rows, 0}, {1, 6 * 2}).write(record_ft.data());
        }

        static int count = 0;
        std::cout
            << count++ << " "
            << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count() /
                   1e6
            << std::endl;
      },
      1000ms / 30);
  record_ev.PushTask([] {
    if (record_file) {
      return;
    }

    record_file = std::make_shared<HighFive::File>("example.h5", HighFive::File::Overwrite);
    for (int i = 0; i < kCameraN; i++) {
      {
        std::vector<size_t> dims = {0, kWidth * kHeight};
        std::vector<size_t> max_dims = {HighFive::DataSpace::UNLIMITED, kWidth * kHeight};
        HighFive::DataSetCreateProps props;
        props.add(HighFive::Chunking(std::vector<hsize_t>{1, kWidth * kHeight}));
        props.add(HighFive::Deflate(1));
        record_depth_dataset[i] = record_file->createDataSet<std::uint16_t>(
            "/observations/images/cam" + std::to_string(i) + "_depth", HighFive::DataSpace(dims, max_dims), props);
      }

      {
        std::vector<size_t> dims = {0, kWidth * kHeight * 3};
        std::vector<size_t> max_dims = {HighFive::DataSpace::UNLIMITED, kWidth * kHeight * 3};
        HighFive::DataSetCreateProps props;
        props.add(HighFive::Chunking(std::vector<hsize_t>{1, kWidth * kHeight * 3}));
        props.add(HighFive::Deflate(1));
        record_rgb_dataset[i] = record_file->createDataSet<std::uint8_t>(
            "/observations/images/cam" + std::to_string(i) + "_rgb", HighFive::DataSpace(dims, max_dims), props);
      }
    }

    {
      std::vector<size_t> dims = {0, kRobotDOF + kGripperDOF};
      std::vector<size_t> max_dims = {HighFive::DataSpace::UNLIMITED, kRobotDOF + kGripperDOF};
      HighFive::DataSetCreateProps props;
      props.add(HighFive::Chunking(std::vector<hsize_t>{1, kRobotDOF + kGripperDOF}));
      record_action_dataset = record_file->createDataSet<double>("/action", HighFive::DataSpace(dims, max_dims), props);
      record_qpos_dataset =
          record_file->createDataSet<double>("/observations/qpos", HighFive::DataSpace(dims, max_dims), props);
      record_qvel_dataset =
          record_file->createDataSet<double>("/observations/qvel", HighFive::DataSpace(dims, max_dims), props);
    }

    {
      std::vector<size_t> dims = {0, 6 * 2};
      std::vector<size_t> max_dims = {HighFive::DataSpace::UNLIMITED, 6 * 2};
      HighFive::DataSetCreateProps props;
      props.add(HighFive::Chunking(std::vector<hsize_t>{1, 6 * 2}));
      record_ft_dataset = record_file->createDataSet<double>("/observations/ft_sensor",
                                                                  HighFive::DataSpace(dims, max_dims), props);
    }
  });

  service_ev.WaitForTasks();

  return 0;
}

void InitializeService() {
  srv_sock = zmq::socket_t(zmq_ctx, zmq::socket_type::sub);
  srv_sock.bind("tcp://*:5000");

  srv_sock.set(zmq::sockopt::subscribe, "");
}

void DoService() {
  std::vector<zmq::message_t> recv_msgs;

  try {
    zmq::recv_result_t result = zmq::recv_multipart(srv_sock, std::back_inserter(recv_msgs), zmq::recv_flags::dontwait);
    if (recv_msgs.size() == 0) {
      return;
    }

    auto j = nlohmann::json::parse(recv_msgs[1].to_string());
    if (!j.contains("command")) {
      return;
    }

    std::string command = j["command"];
    if (command == COMMAND_READY_POSE) {
      Eigen::Vector<double, 20> body_ready;
      Eigen::Vector<double, 2> head_ready;
      double minimum_time = 5.0;
      body_ready << 0, 35, -70, 35, 0, 0, -30, -10, 0, -100, 0, 40, 0, -30, 10, 0, -100, 0, 40, 0;
      head_ready << 0, 0;
      body_ready *= M_PI / 180.;
      head_ready *= M_PI / 180.;

      robot_ev.PushTask([=] { GoPose(body_ready, head_ready); });
    } else if (command == COMMAND_ZERO_POSE) {
      robot_ev.PushTask([=] { GoPose(Eigen::Vector<double, 20>::Zero(), Eigen::Vector<double, 2>::Zero()); });
    } else if (command == COMMAND_STOP_MOTION) {
      robot_ev.PushTask([=] {
        if (!robot) {
          return;
        }
      });
    }
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }
}

void InitializeRobot() {
  const std::string kAll = ".*";

  robot = Robot<y1_model::A>::Create(RPC_ADDRESS);

  std::cout << "Attempting to connect to the robot..." << std::endl;
  if (!robot->Connect()) {
    std::cerr << "Error: Unable to establish connection to the robot at " << RPC_ADDRESS << std::endl;
    robot = nullptr;
    return;
  }
  std::cout << "Successfully connected to the robot." << std::endl;

  // std::cout << "Checking power status..." << std::endl;
  // if (!robot->IsPowerOn(kAll)) {
  //   std::cout << "Power is currently OFF. Attempting to power on..." << std::endl;
  //   if (!robot->PowerOn(kAll)) {
  //     std::cerr << "Error: Failed to power on the robot." << std::endl;
  //     robot = nullptr;
  //     return;
  //   }
  //   std::cout << "Robot powered on successfully." << std::endl;
  // } else {
  //   std::cout << "Power is already ON." << std::endl;
  // }

  // std::cout << "Checking servo status..." << std::endl;
  // if (!robot->IsServoOn(kAll)) {
  //   std::cout << "Servo is currently OFF. Attempting to activate servo..." << std::endl;
  //   if (!robot->ServoOn(kAll)) {
  //     std::cerr << "Error: Failed to activate servo." << std::endl;
  //     robot = nullptr;
  //     return;
  //   }
  //   std::cout << "Servo activated successfully." << std::endl;
  // } else {
  //   std::cout << "Servo is already ON." << std::endl;
  // }

  // const auto& control_manager_state = robot->GetControlManagerState();
  // if (control_manager_state.state == ControlManagerState::State::kMajorFault ||
  //     control_manager_state.state == ControlManagerState::State::kMinorFault) {
  //   std::cerr << "Warning: Detected a "
  //             << (control_manager_state.state == ControlManagerState::State::kMajorFault ? "Major" : "Minor")
  //             << " Fault in the Control Manager." << std::endl;

  //   std::cout << "Attempting to reset the fault..." << std::endl;
  //   if (!robot->ResetFaultControlManager()) {
  //     std::cerr << "Error: Unable to reset the fault in the Control Manager." << std::endl;
  //     robot = nullptr;
  //     return;
  //   }
  //   std::cout << "Fault reset successfully." << std::endl;
  // }
  // std::cout << "Control Manager state is normal. No faults detected." << std::endl;

  // std::cout << "Enabling the Control Manager..." << std::endl;
  // if (!robot->EnableControlManager()) {
  //   std::cerr << "Error: Failed to enable the Control Manager." << std::endl;
  //   robot = nullptr;
  //   return;
  // }
  // std::cout << "Control Manager enabled successfully." << std::endl;

  // try {
  //   if (robot->IsPowerOn("48v")) {
  //     robot->SetToolFlangeOutputVoltage("left", 12);
  //     robot->SetToolFlangeOutputVoltage("right", 12);
  //   }
  // } catch (std::exception& e) {
  //   std::cerr << e.what() << std::endl;
  //   robot = nullptr;
  //   return;
  // }

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
      },
      50 /* Hz */);
}

void GoPose(const Eigen::Vector<double, 20>& body, const Eigen::Vector<double, 2>& head, double minimum_time) {
  if (!robot) {
    return;
  }

  if (!IsMotionDone()) {
    return;
  }

  auto handler = robot->SendCommand(RobotCommandBuilder().SetCommand(
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
  }

  if (rcs_handler) {
    if (rcs_handler->IsDone()) {
      if (is_started) {
        std::cerr << "Stop unexpectedly" << std::endl;

        record_ev.PushTask([] {
          if (record_file) {
            record_file->flush();
            record_file.reset();
            record_file = nullptr;
          }
        });
      }

      rcs_handler = nullptr;
      return true;
    }
  }

  return false;
}

void InitializeCamera() {
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
    cfg.enable_stream(RS2_STREAM_DEPTH, kWidth, kHeight, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, kWidth, kHeight, RS2_FORMAT_RGB8, 30);
    pipe.start(cfg);
    rs_pipelines[dev_id] = pipe;
  }
}

void InitializeMasterArm() {
  try {
    // Latency timer setting
    upc::InitializeDevice("/dev/rby1_master_arm");
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    exit(1);
  }

  master_arm = std::make_shared<upc::MasterArm>("/dev/rby1_master_arm");

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