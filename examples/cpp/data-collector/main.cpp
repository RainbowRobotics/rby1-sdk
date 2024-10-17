#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#define NO_TELEOP

#include <csignal>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

#include <boost/filesystem.hpp>
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

const double kFrequency = 60;   // (Hz) = Framerate
const std::string kAll = ".*";  // NOLINT
const double kMasterArmLimitGain = 0.5;
const std::string MASTER_ARM_DEV = "/dev/rby1_master_arm";
const std::string GRIPPER_DEV = "/dev/rby1_gripper";
const double kGripperControlFrequency = 50;  // (Hz)
const std::string COMMAND_POWER_OFF = "power_off";
const std::string COMMAND_POWER_ON = "power_on";
const std::string COMMAND_SERVO_ON = "servo_on";
const std::string COMMAND_CONTROL_MANAGER_INIT = "init_control_manager";
const std::string COMMAND_READY_POSE = "ready_pose";
const std::string COMMAND_ZERO_POSE = "zero_pose";
const std::string COMMAND_STOP_MOTION = "stop_motion";
const std::string COMMAND_START_TELEOPERATION = "start_teleoperation";
const std::string COMMAND_START_RECORDING = "start_recording";
const std::string COMMAND_STOP_RECORDING = "stop_recording";
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
std::array<rs2::colorizer, kCameraN> rs_colorizer;
#endif
bool camera_error = true;

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
std::atomic<int> data_count = 0;

/**
 * Tele-operation
 */
Eigen::Vector<double, upc::MasterArm::kDOF> ma_qmin;
Eigen::Vector<double, upc::MasterArm::kDOF> ma_qmax;
Eigen::Vector<double, upc::MasterArm::kDOF> ma_torque_limit;
Eigen::Vector<double, upc::MasterArm::kDOF> ma_viscous_term;
bool ma_init = false;
Eigen::Vector<double, upc::MasterArm::kDOF / 2> ma_q_right, ma_q_left;
bool tele_right_button, tele_left_button;
Eigen::Vector<double, upc::MasterArm::kDOF> tele_q;
std::shared_ptr<rb::dyn::Robot<rb::y1_model::A::kRobotDOF>> robot_dyn;
std::shared_ptr<rb::dyn::State<rb::y1_model::A::kRobotDOF>> robot_dyn_state;
Eigen::Vector<double, rb::y1_model::A::kRobotDOF> q_lower_limit, q_upper_limit;
Eigen::Vector<double, rb::y1_model::A::kRobotDOF> q_joint_ref;

/**
 * Gripper
 */
std::unique_ptr<rb::DynamixelBus> gripper;
int gripper_init_step{0};
steady_clock::time_point gripper_step_start_time;
Eigen::Vector<int, 2> gripper_operation_mode;
Eigen::Vector<double, 2> gripper_torque_command;
Eigen::Vector<double, 2> gripper_position_command;
Eigen::Vector<double, 2> gripper_position_command_prev;
std::vector<double> gripper_torque_constant;
Eigen::Vector<double, 2> gripper_min, gripper_max;

EventLoop publisher_ev, service_ev, robot_ev, cm_ev, camera_ev, record_ev, gripper_ev;
std::future<void> image_future;

bool is_camera_failed{false};
bool is_started{false};

std::string upc_address;
std::string save_folder_path;

bool robot_connected{false};
bool camera_connected{false};
bool master_arm_connected{false};
bool power{false};
bool servo{false};
bool control_manager{false};
bool recording{false};
std::string control_manager_detail;
unsigned long storage_free{0};
unsigned long storage_available{0};
unsigned long storage_capacity{0};
std::array<Mat, kCameraN> rgb_images, depth_images;

void SignalHandler(int signum);
void InitializeService();
void DoService();
void InitializePublisher();
void UpdateStat();
void Publish();
void InitializeRobot(const std::string& address);
void StopMotion();
void GoPose(const Eigen::Vector<double, 20>& body, const Eigen::Vector<double, 2>& head, double minimum_time = 5.0);
bool IsMotionDone();
void RobotPowerOff();
void RobotPowerOn();
void RobotServoOn();
void RobotControlManagerInit();
void InitializeGripper();
void ControlGripper();
void InitializeCamera();
void InitializeMasterArm();
void StartRecording(const std::string& file_path);
void StopRecording();
void StartTeleoperation();
template <typename T>
void AddDataIntoDataSet(std::unique_ptr<HighFive::DataSet>& dataset, const std::vector<std::size_t>& shape, T* data);
template <typename T>
HighFive::DataSet CreateDataSet(const std::shared_ptr<HighFive::File>& file, const std::string& name,
                                const std::vector<std::size_t>& shape, int compression_level = 0);

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <rpc server address> <save folder path>" << std::endl;
    return 1;
  }
  upc_address = std::string(argv[1]);
  save_folder_path = std::string(argv[2]);

  signal(SIGINT, SignalHandler);

#ifndef NO_TELEOP
  try {
    // Latency timer setting
    upc::InitializeDevice(GRIPPER_DEV);
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    exit(1);
  }
#endif

  publisher_ev.DoTask([] { InitializePublisher(); });
  service_ev.DoTask([] { InitializeService(); });
  robot_ev.DoTask([=] { InitializeRobot(upc_address); });
  camera_ev.DoTask([] { InitializeCamera(); });
#ifndef NO_TELEOP
  gripper_ev.DoTask([] { InitializeGripper(); });
  InitializeMasterArm();
#endif

  publisher_ev.PushTask([] { master_arm_connected = (master_arm != nullptr); });

  publisher_ev.PushCyclicTask(
      [] {
        UpdateStat();
        Publish();
      },
      100ms);
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

        if (rcs_handler) {

          if (rcs_handler->IsDone()) {
            rcs_handler = nullptr;
            return;
          }

          static double right_arm_minimum_time = 1., left_arm_minimum_time = 1.;
          static double lpf_update_ratio = 0.05;

          if (tele_right_button) {
            //right hand position control mode
            q_joint_ref.block(0, 0, 7, 1) =
                q_joint_ref.block(0, 0, 7, 1) * (1 - lpf_update_ratio) + tele_q.block(0, 0, 7, 1) * lpf_update_ratio;
          } else {
            right_arm_minimum_time = 1.0;
          }

          if (tele_left_button) {
            //left hand position control mode
            q_joint_ref.block(7, 0, 7, 1) =
                q_joint_ref.block(7, 0, 7, 1) * (1 - lpf_update_ratio) + tele_q.block(7, 0, 7, 1) * lpf_update_ratio;
          } else {
            left_arm_minimum_time = 1.0;
          }

          for (int i = 0; i < 14; i++) {
            q_joint_ref(i) = std::clamp(q_joint_ref(i), q_lower_limit(i), q_upper_limit(i));
          }

          Eigen::Vector<double, 7> target_position_left = q_joint_ref.block(7, 0, 7, 1);
          Eigen::Vector<double, 7> target_position_right = q_joint_ref.block(0, 0, 7, 1);
          Eigen::Vector<double, 7> acc_limit, vel_limit;

          robot_dyn->ComputeForwardKinematics(robot_dyn_state);
          auto T_12 = robot_dyn->ComputeTransformation(robot_dyn_state, 1, 2);
          auto T_13 = robot_dyn->ComputeTransformation(robot_dyn_state, 1, 3);
          Eigen::Vector3d center = (rb::math::SE3::GetPosition(T_12) + rb::math::SE3::GetPosition(T_13)) / 2.;
          double yaw = atan2(center(1), center(0));
          double pitch = atan2(-center(2), center(0)) + 10 * M_PI / 180.;
          yaw = std::clamp(yaw, -0.523, 0.523);
          pitch = std::clamp(pitch, -0.35, 1.57);

          acc_limit.setConstant(1200.0);
          acc_limit *= M_PI / 180.;

          vel_limit << 160, 160, 160, 160, 330, 330, 330;
          vel_limit *= M_PI / 180.;

          right_arm_minimum_time *= 0.99;
          right_arm_minimum_time = std::max(right_arm_minimum_time, 0.015);

          left_arm_minimum_time *= 0.99;
          left_arm_minimum_time = std::max(left_arm_minimum_time, 0.015);

          try {
            RobotCommandBuilder command_builder;

            command_builder.SetCommand(
                ComponentBasedCommandBuilder()
                    .SetHeadCommand(JointPositionCommandBuilder()
                                        .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(4.))
                                        .SetMinimumTime(0)
                                        .SetPosition(Eigen::Vector2d{yaw, pitch})
                                        .SetVelocityLimit(vel_limit / 10)
                                        .SetAccelerationLimit(acc_limit / 10))
                    .SetBodyCommand(
                        BodyComponentBasedCommandBuilder()
                            .SetRightArmCommand(JointPositionCommandBuilder()
                                                    .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(4.))
                                                    .SetMinimumTime(right_arm_minimum_time)
                                                    .SetPosition(target_position_right)
                                                    .SetVelocityLimit(vel_limit)
                                                    .SetAccelerationLimit(acc_limit))
                            .SetLeftArmCommand(JointPositionCommandBuilder()
                                                   .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(4.))
                                                   .SetMinimumTime(left_arm_minimum_time)
                                                   .SetPosition(target_position_left)
                                                   .SetVelocityLimit(vel_limit)
                                                   .SetAccelerationLimit(acc_limit))));

            rcs_handler->SendCommand(command_builder);

          } catch (...) {}
        }
      },
<<<<<<< Updated upstream
      5ms);
#ifndef NO_TELEOP
=======
      10ms);
>>>>>>> Stashed changes
  gripper_ev.PushCyclicTask([] { ControlGripper(); }, nanoseconds((long)(1e9 / kGripperControlFrequency)));
#endif
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
                    control_manager_detail += " (Unknown)";
                    break;
                  case ControlManagerState::ControlState::kIdle:
                    control_manager_detail += " (Idle)";
                    break;
                  case ControlManagerState::ControlState::kExecuting:
                    control_manager_detail += " (Executing)";
                    break;
                  case ControlManagerState::ControlState::kSwitching:
                    control_manager_detail += " (Switching)";
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
        std::vector<rs2::frame> new_frames;
        for (auto&& pipe : rs_pipelines) {
          rs2::frameset fs;
          if (pipe.poll_for_frames(&fs)) {
            for (const rs2::frame& f : fs)
              new_frames.emplace_back(f);
          }
        }

        for (const auto& frame : new_frames) {
          auto serial = rs2::sensor_from_frame(frame)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
          if (rs_serials.find(serial) == rs_serials.end()) {
            continue;
          }
          int id = rs_serials.find(serial)->second;

          rs2_stream stream_type = frame.get_profile().stream_type();
          if (stream_type == RS2_STREAM_COLOR) {
            record_ev.PushTask([rgb = Mat(Size(kWidth, kHeight), CV_8UC3, (void*)frame.get_data(), Mat::AUTO_STEP),
                                id] { record_rgb[id] = rgb; });
            publisher_ev.PushTask([rgb = Mat(Size(kWidth, kHeight), CV_8UC3, (void*)frame.get_data(), Mat::AUTO_STEP),
                                   id] { rgb_images[id] = rgb; });
          } else if (stream_type == RS2_STREAM_DEPTH) {
            rs2::depth_frame depth = frame.as<rs2::depth_frame>();

            record_ev.PushTask([depth = Mat(Size(kWidth, kHeight), CV_16FC1, (void*)depth.get_data(), Mat::AUTO_STEP),
                                id] { record_depth[id] = depth; });
            rs2::frame colored_depth = rs_colorizer[id].process(depth);
            publisher_ev.PushTask(
                [depth = Mat(Size(kWidth, kHeight), CV_8UC3, (void*)colored_depth.get_data(), Mat::AUTO_STEP), id] {
                  depth_images[id] = depth;
                });
          }
        }
#endif
      },
      10us);
  record_ev.PushCyclicTask(
      [] {
        auto start = steady_clock::now();

        publisher_ev.PushTask([r = (record_file != nullptr)] { recording = r; });

        if (!record_file) {
          return;
        }

        for (int i = 0; i < kCameraN; i++) {
          AddDataIntoDataSet(record_depth_dataset[i], {kHeight, kWidth}, record_depth[i].data);
          AddDataIntoDataSet(record_rgb_dataset[i], {kHeight, kWidth, 3}, record_rgb[i].data);
        }
        AddDataIntoDataSet(record_action_dataset, {kRobotDOF + kGripperDOF}, record_action.data());
        AddDataIntoDataSet(record_qpos_dataset, {kRobotDOF + kGripperDOF}, record_qpos.data());
        AddDataIntoDataSet(record_qvel_dataset, {kRobotDOF + kGripperDOF}, record_qvel.data());
        AddDataIntoDataSet(record_torque_dataset, {kRobotDOF + kGripperDOF}, record_torque.data());
        AddDataIntoDataSet(record_ft_dataset, {6 * 2}, record_ft.data());
        data_count++;

        auto end = steady_clock::now();

        double d = (double)duration_cast<nanoseconds>(end - start).count() / 1.e9;
        stat_duration.add(d);
        if (stat_duration.count() >= kFrequency) {
          std::cout << "[" << data_count.load() << "] " << "Count: " << stat_duration.count()
                    << ", Avg: " << stat_duration.avg() * 1e3 << " ms, Min: " << stat_duration.min() * 1e3
                    << " ms, Max: " << stat_duration.max() * 1e3 << " ms" << std::endl;
          stat_duration.reset();
        }
      },
      microseconds((long)(1e6 /* 1 sec */ / kFrequency)));

  service_ev.WaitForTasks();

  return 0;
}

void SignalHandler(int signum) {
  robot_ev.DoTask([=] {
    if (!robot) {
      return;
    }

    robot->PowerOff("12v");
  });
  robot_ev.Stop();

  record_ev.DoTask([] {
    if (record_file) {
      record_file->flush();
      record_file.reset();
      record_file = nullptr;
    }
  });
  record_ev.Stop();

  camera_ev.DoTask([] {
    if (!camera_error) {
      // TODO: close rs pipelines
#ifdef REALSENSE2
      for (auto&& pipe : rs_pipelines) {
        pipe.stop();
      }
#endif
    }

    camera_error = true;
  });
  camera_ev.Stop();

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
      body_ready << 0, 40, -70, 30, 0, 0, -30, -10, 0, -100, 0, 40, 0, -30, 10, 0, -100, 0, 40, 0;
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
    } else if (command == COMMAND_START_TELEOPERATION) {
      robot_ev.PushTask([=] { StartTeleoperation(); });
    } else if (command == COMMAND_STOP_MOTION) {
      robot_ev.PushTask([=] { StopMotion(); });
    } else if (command == COMMAND_START_RECORDING) {
      std::string name{"example"};
      if (j.contains("name")) {
        name = j["name"];
      }
      record_ev.PushTask([=] { StartRecording(save_folder_path + "/" + name + ".h5"); });
    } else if (command == COMMAND_STOP_RECORDING) {
      record_ev.PushTask([=] { StopRecording(); });
    }
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }
}

void InitializePublisher() {
  pub_sock = zmq::socket_t(zmq_ctx, zmq::socket_type::pub);
  pub_sock.bind("tcp://*:5001");

  for (int i = 0; i < kCameraN; i++) {
    rgb_images[i] = Mat(Size(kWidth, kHeight), CV_8UC3);
    depth_images[i] = Mat(Size(kWidth, kHeight), CV_8UC3);
  }
}

void UpdateStat() {
  using namespace boost::filesystem;
  space_info si = space(save_folder_path);
  storage_free = si.free / (1 << 20);  // (MB)
  storage_available = si.available / (1 << 20);
  storage_capacity = si.capacity / (1 << 20);
}

void Publish() {
  nlohmann::json j;

  j["robot"] = robot_connected;
  j["camera"] = camera_connected;
  j["master_arm"] = master_arm_connected;

  j["storage_available"] = storage_available;
  j["storage_free"] = storage_free;
  j["storage_capacity"] = storage_capacity;

  j["power"] = power;
  j["servo"] = servo;
  j["control_manager"]["state"] = control_manager;
  j["control_manager"]["detail"] = control_manager_detail;

  j["recording"] = recording;
  j["recording_count"] = data_count.load();

  zmq::send_multipart(pub_sock, std::array<zmq::const_buffer, 2>{zmq::str_buffer("data"), zmq::buffer(j.dump())});

  static auto image_pub_time = steady_clock::now();
  auto current_time = steady_clock::now();
  double d = (double)duration_cast<nanoseconds>(current_time - image_pub_time).count() / 1.e9;
  if (d >= 0.3 /* s */) {
    image_pub_time = current_time;

    bool done = true;
    if (image_future.valid()) {
      if (image_future.wait_for(0s) == std::future_status::timeout) {
        done = false;
      }
    }

    if (done) {
      image_future = std::async([_rgb_images = rgb_images, _depth_images = depth_images] {
        std::vector<uint8_t> buf;

        nlohmann::json image_j;
        for (int i = 0; i < kCameraN; i++) {
          cv::imencode(".jpg", _rgb_images[i], buf);
          image_j["cam" + std::to_string(i) + "_rgb"] = nlohmann::json::binary_t(buf);

          cv::imencode(".jpg", _depth_images[i], buf);
          image_j["cam" + std::to_string(i) + "_depth"] = nlohmann::json::binary_t(buf);
        }

        publisher_ev.PushTask([=] {
          zmq::send_multipart(pub_sock,
                              std::array<zmq::const_buffer, 2>{zmq::str_buffer("image"), zmq::buffer(image_j.dump())});
        });
      });
    }
  }
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

  robot->PowerOn("12v");  // TODO

  robot->StartStateUpdate(
      [](const rb::RobotState<rb::y1_model::A>& state) {
        record_ev.PushTask([s = state] {
          record_qpos.head<kRobotDOF>() = s.position;
          record_qvel.head<kRobotDOF>() = s.velocity;
          record_action.head<kRobotDOF>() = s.target_position;
          record_torque.head<kRobotDOF>() = s.torque;
          record_ft.block(0, 0, 3, 1) = s.ft_sensor_right.torque;
          record_ft.block(3, 0, 3, 1) = s.ft_sensor_right.force;
          record_ft.block(6, 0, 3, 1) = s.ft_sensor_left.torque;
          record_ft.block(9, 0, 3, 1) = s.ft_sensor_left.force;
        });

        publisher_ev.PushTask([s = state] {
          power = true;
          for (const auto& p : s.power_states) {
            power &= (p.state == PowerState::State::kPowerOn);
          }

          servo = true;
          for (const auto& j : s.joint_states) {
            servo &= (j.time_since_last_update.tv_sec < 1. && j.is_ready);
          }
        });

        robot_ev.PushTask([s = state] {
          robot_dyn_state->SetQ(s.position);

          if (rcs_handler) {
            return;
          }

          q_joint_ref = s.position.block(2 + 6, 0, 14, 1);
        });
      },
      100 /* Hz */);
      
  robot->SetParameter("joint_position_command.cutoff_frequency", "20.0");

  robot_dyn = robot->GetDynamics();
  robot_dyn_state = robot_dyn->MakeState({"base", "link_head_0", "ee_right", "ee_left"}, y1_model::A::kRobotJointNames);
  robot_dyn_state->SetGravity({0, 0, 0, 0, 0, -9.81});

  q_upper_limit = robot_dyn->GetLimitQUpper(robot_dyn_state).block(2 + 6, 0, 14, 1);
  q_lower_limit = robot_dyn->GetLimitQLower(robot_dyn_state).block(2 + 6, 0, 14, 1);
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

  if (!IsMotionDone()) {
    StopMotion();
  }

  robot->PowerOff("^((?!12v)).*");
  std::cout << "Robot powered off." << std::endl;
}

void RobotPowerOn() {
  if (!robot) {
    std::cerr << "[Robot] Robot is not initialized" << std::endl;
    return;
  }

  std::cout << "Checking power status..." << std::endl;
  if (!robot->IsPowerOn("^((?!12v)).*")) {
    std::cout << "Power is currently OFF. Attempting to power on..." << std::endl;
    if (!robot->PowerOn("^((?!12v)).*")) {
      std::cerr << "Error: Failed to power on the robot." << std::endl;
      robot = nullptr;
      return;
    }
    std::cout << "Robot powered on successfully." << std::endl;
  } else {
    std::cout << "Power is already ON." << std::endl;
  }

  std::this_thread::sleep_for(500ms);

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

void StartTeleoperation() {
  if (!robot) {
    std::cerr << "[Robot] Robot is not initialized" << std::endl;
    return;
  }

  if (!IsMotionDone()) {
    std::cerr << "[Robot] Robot is already moving" << std::endl;
    return;
  }

  rcs_handler = robot->CreateCommandStream();
  tele_right_button = tele_left_button = false;
}

void InitializeCamera() {
  camera_error = true;

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
    cfg.enable_stream(RS2_STREAM_COLOR, kWidth, kHeight, RS2_FORMAT_BGR8, (int)kFrequency);
    pipe.start(cfg);
    rs_pipelines[dev_id] = pipe;
    rs_colorizer[dev_id] = rs2::colorizer();
  }

  camera_error = false;
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
  master_arm->SetControlPeriod(0.01);

  ma_qmin << -360, -90, 0, -135, -90, 0, -360,  //
      -360, 10, -90, -135, -90, 0, -360;
  ma_qmax << 360, -10, 90, -20, 90, 80, 360,  //
      360, 90, 0, -20, 90, 80, 360;
  ma_qmin *= M_PI / 180.;
  ma_qmax *= M_PI / 180.;
  ma_torque_limit.setConstant(3);
  ma_viscous_term << 0.01, 0.01, 0.01, 0.01, 0.005, 0.005, 0.001,  //
      0.01, 0.01, 0.01, 0.01, 0.005, 0.005, 0.001;

  auto active_ids = master_arm->Initialize();
  if (active_ids.size() != upc::MasterArm::kDOF + 2) {
    exit(1);
  }

  ma_init = false;
  ma_q_right.setZero();
  ma_q_left.setZero();
  master_arm->StartControl([](const upc::MasterArm::State& state) {
    upc::MasterArm::ControlInput input;

    if (!ma_init) {
      ma_q_right = state.q_joint(Eigen::seq(0, 6));
      ma_q_left = state.q_joint(Eigen::seq(7, 13));
      ma_init = true;
    }

    if (state.button_right.button == 1) {
      input.target_operation_mode(Eigen::seq(0, 6)).setConstant(DynamixelBus::kCurrentControlMode);
      input.target_torque(Eigen::seq(0, 6)) =
          (state.gravity_term(Eigen::seq(0, 6)) +
           (ma_qmin(Eigen::seq(0, 6)) - state.q_joint(Eigen::seq(0, 6))).cwiseMax(0) * kMasterArmLimitGain +
           (ma_qmax(Eigen::seq(0, 6)) - state.q_joint(Eigen::seq(0, 6))).cwiseMin(0) * kMasterArmLimitGain +
           (state.qvel_joint(Eigen::seq(0, 6)).array() * ma_viscous_term(Eigen::seq(0, 6)).array()).matrix())
              .cwiseMin(ma_torque_limit(Eigen::seq(0, 6)))
              .cwiseMax(-ma_torque_limit(Eigen::seq(0, 6)));
      ma_q_right = state.q_joint(Eigen::seq(0, 6));
    } else {
      input.target_operation_mode(Eigen::seq(0, 6)).setConstant(DynamixelBus::kCurrentBasedPositionControlMode);
      input.target_position(Eigen::seq(0, 6)) = ma_q_right;
    }

    if (state.button_left.button == 1) {
      input.target_operation_mode(Eigen::seq(7, 13)).setConstant(DynamixelBus::kCurrentControlMode);
      input.target_torque(Eigen::seq(7, 13)) =
          (state.gravity_term(Eigen::seq(7, 13)) +
           (ma_qmin(Eigen::seq(7, 13)) - state.q_joint(Eigen::seq(7, 13))).cwiseMax(0) * kMasterArmLimitGain +
           (ma_qmax(Eigen::seq(7, 13)) - state.q_joint(Eigen::seq(7, 13))).cwiseMin(0) * kMasterArmLimitGain +
           (state.qvel_joint(Eigen::seq(7, 13)).array() * ma_viscous_term(Eigen::seq(7, 13)).array()).matrix())
              .cwiseMin(ma_torque_limit(Eigen::seq(7, 13)))
              .cwiseMax(-ma_torque_limit(Eigen::seq(7, 13)));
      ma_q_left = state.q_joint(Eigen::seq(7, 13));
    } else {
      input.target_operation_mode(Eigen::seq(7, 13)).setConstant(DynamixelBus::kCurrentBasedPositionControlMode);
      input.target_position(Eigen::seq(7, 13)) = ma_q_left;
    }

    gripper_ev.PushTask([r = state.button_right.trigger, l = state.button_left.trigger] {
      if (gripper_init_step < 4)
        return;

      gripper_operation_mode << DynamixelBus::kCurrentBasedPositionControlMode,
          DynamixelBus::kCurrentBasedPositionControlMode;
      gripper_position_command =
          (Eigen::Vector2d{1. - r / 1000., 1. - l / 1000.}.array() * (gripper_max - gripper_min).array()).matrix() +
          gripper_min;
    });

    robot_ev.PushTask([rb = state.button_right.button, lb = state.button_left.button,
                       r = state.q_joint(Eigen::seq(0, 6)), l = state.q_joint(Eigen::seq(7, 13))] {
      if (!rcs_handler) {
        return;
      }

      tele_right_button = rb;
      tele_left_button = lb;
      tele_q.head<7>() = r;
      tele_q.tail<7>() = l;
    });

    return input;
  });
}

void StartRecording(const std::string& file_path) {
  if (record_file) {
    return;
  }

  // Make File
  record_file = std::make_shared<HighFive::File>(file_path, HighFive::File::Overwrite);

  record_file->createAttribute("frequency", kFrequency);

  // Create Dataset
  for (int i = 0; i < kCameraN; i++) {
    record_depth_dataset[i] = std::make_unique<HighFive::DataSet>(CreateDataSet<std::uint16_t>(
        record_file, "/observations/images/cam" + std::to_string(i) + "_depth", {kHeight, kWidth}, kCompressionLevel));
    record_depth[i] = Mat(Size(kWidth, kHeight), CV_16FC1);

    record_rgb_dataset[i] = std::make_unique<HighFive::DataSet>(CreateDataSet<std::uint8_t>(
        record_file, "/observations/images/cam" + std::to_string(i) + "_rgb", {kHeight, kWidth, 3}, kCompressionLevel));
    record_rgb[i] = Mat(Size(kWidth, kHeight), CV_8UC3);
  }
  record_action_dataset =
      std::make_unique<HighFive::DataSet>(CreateDataSet<double>(record_file, "/action", {kRobotDOF + kGripperDOF}));
  record_qpos_dataset = std::make_unique<HighFive::DataSet>(
      CreateDataSet<double>(record_file, "/observations/qpos", {kRobotDOF + kGripperDOF}));
  record_qvel_dataset = std::make_unique<HighFive::DataSet>(
      CreateDataSet<double>(record_file, "/observations/qvel", {kRobotDOF + kGripperDOF}));
  record_torque_dataset = std::make_unique<HighFive::DataSet>(
      CreateDataSet<double>(record_file, "/observations/torque", {kRobotDOF + kGripperDOF}));
  record_ft_dataset =
      std::make_unique<HighFive::DataSet>(CreateDataSet<double>(record_file, "/observations/ft_sensor", {6 * 2}));

  data_count = 0;
}

void StopRecording() {
  if (!record_file) {
    return;
  }

  record_file->flush();
  record_file.reset();
  record_file = nullptr;
}

void InitializeGripper() {
  gripper_torque_constant = {1.6591, 1.6591};
  gripper_min.setConstant(100);
  gripper_max.setConstant(-100);

  gripper = std::make_unique<rb::DynamixelBus>(GRIPPER_DEV);

  if (!gripper->OpenPort()) {
    std::cerr << "[Gripper] Failed to initialize" << std::endl;
    return;
  }

  if (!gripper->SetBaudRate(DynamixelBus::kDefaultBaudrate)) {
    std::cerr << "[Gripper] Failed to initialize" << std::endl;
    return;
  }

  gripper->SetTorqueConstant(gripper_torque_constant);
}

void ControlGripper() {
  if (gripper_init_step == 0) {
    static int success = 0;
    for (int id = 0; id < 2; id++) {
      if (!gripper->Ping(id)) {
        success = 0;
        return;
      }
    }
    if (success++ >= 20) {
      for (int id = 0; id < 2; id++) {
        if (!gripper->SendOperationMode(id, DynamixelBus::kCurrentControlMode))
          ;
        gripper->SendTorqueEnable(id, DynamixelBus::kTorqueEnable);
      }

      gripper_init_step = 1;
      gripper_step_start_time = steady_clock::now();
      return;
    }
  } else if (gripper_init_step == 1) {
    gripper_operation_mode = {DynamixelBus::kCurrentControlMode, DynamixelBus::kCurrentControlMode};
    gripper_torque_command = {1, 1};

    double d = duration_cast<nanoseconds>(steady_clock::now() - gripper_step_start_time).count() / 1e9;
    if (d > 2.) {
      gripper_init_step = 2;
      gripper_step_start_time = steady_clock::now();
    }
  } else if (gripper_init_step == 2) {
    gripper_operation_mode = {DynamixelBus::kCurrentControlMode, DynamixelBus::kCurrentControlMode};
    gripper_torque_command = {-1, -1};

    double d = duration_cast<nanoseconds>(steady_clock::now() - gripper_step_start_time).count() / 1e9;
    if (d > 2.) {
      gripper_init_step = 3;
      gripper_step_start_time = steady_clock::now();
    }
  } else if (gripper_init_step == 3) {
    gripper_operation_mode = {DynamixelBus::kCurrentControlMode, DynamixelBus::kCurrentControlMode};
    gripper_torque_command = {0, 0};
    gripper_init_step = 4;
  }

  Eigen::Vector<int, 2> operation_mode;
  Eigen::Vector<double, 2> q, qvel, torque;

  auto temp_operation_mode_vector = gripper->BulkReadOperationMode({0, 1});
  if (temp_operation_mode_vector.has_value()) {
    for (auto const& ret : temp_operation_mode_vector.value()) {
      operation_mode[ret.first] = ret.second;
    }
  } else {
    return;
  }

  auto temp_ms_vector = gripper->BulkReadMotorState({0, 1});
  if (temp_ms_vector.has_value()) {
    for (auto const& ret : temp_ms_vector.value()) {
      q[ret.first] = ret.second.position;
      qvel[ret.first] = ret.second.velocity;
      torque[ret.first] = ret.second.current * gripper_torque_constant[ret.first];
    }
  } else {
    return;
  }

  std::vector<std::pair<int, int>> changed_id_mode;
  std::vector<int> changed_id;

  std::vector<std::pair<int, double>> id_position;
  std::vector<std::pair<int, double>> id_torque;

  for (int i = 0; i < 2; i++) {
    if (operation_mode[i] != gripper_operation_mode[i]) {
      changed_id.push_back(i);
      changed_id_mode.emplace_back(i, gripper_operation_mode[i]);

      if (gripper_operation_mode[i] == DynamixelBus::kCurrentBasedPositionControlMode) {
        gripper_position_command_prev[i] = q[i];
      }
    } else {
      if (operation_mode[i] == DynamixelBus::kCurrentControlMode) {
        id_torque.emplace_back(i, gripper_torque_command[i]);
      } else if (operation_mode[i] == DynamixelBus::kCurrentBasedPositionControlMode) {
        gripper_position_command_prev[i] = 0.8 * gripper_position_command_prev[i] + 0.2 * gripper_position_command[i];

        id_torque.emplace_back(i, 1.0);
        id_position.emplace_back(i, gripper_position_command_prev[i]);
      }
    }
  }

  gripper_min = gripper_min.cwiseMin(q);
  gripper_max = gripper_max.cwiseMax(q);

  gripper->BulkWriteTorqueEnable(changed_id, 0);
  gripper->BulkWriteOperationMode(changed_id_mode);
  gripper->BulkWriteTorqueEnable(changed_id, 1);

  gripper->BulkWriteSendTorque(id_torque);
  gripper->BulkWriteSendPosition(id_position);

  //

  record_ev.PushTask([q, qvel, torque, operation_mode, id_position] {
    record_qpos.tail<kGripperDOF>() = q;
    record_qvel.tail<kGripperDOF>() = qvel;
    record_torque.tail<kGripperDOF>() = torque;
    record_action.tail<kGripperDOF>() = q;
    for (const auto& id : id_position) {
      record_action.tail<kGripperDOF>()(id.first) = id.second;
    }
  });
}

template <typename T>
void AddDataIntoDataSet(std::unique_ptr<HighFive::DataSet>& dataset, const std::vector<std::size_t>& shape, T* data) {
  auto current_dims = dataset->getSpace().getDimensions();
  size_t current_rows = current_dims[0];
  size_t new_rows = current_rows + 1;
  std::vector<std::size_t> resize = {new_rows}, offset = {current_rows}, count = {1};
  for (const auto& l : shape) {
    resize.push_back(l);
    offset.push_back(0);
    count.push_back(l);
  }
  dataset->resize(resize);
  dataset->select(offset, count).write_raw(data);
}

template <typename T>
HighFive::DataSet CreateDataSet(const std::shared_ptr<HighFive::File>& file, const std::string& name,
                                const std::vector<std::size_t>& shape, int compression_level) {
  std::vector<size_t> dims = {0};
  std::vector<size_t> max_dims = {HighFive::DataSpace::UNLIMITED};
  std::vector<hsize_t> chunk_size = {1};
  for (const auto& l : shape) {
    dims.push_back(l);
    max_dims.push_back(l);
    chunk_size.push_back(l);
  }
  HighFive::DataSetCreateProps props;
  props.add(HighFive::Chunking(chunk_size));
  if (compression_level != 0) {
    props.add(HighFive::Deflate(compression_level));
  }
  return file->createDataSet<T>(name, HighFive::DataSpace(dims, max_dims), props);
}