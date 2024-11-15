#include "rby1-sdk/upc/master_arm.h"

#include <iostream>

#include <csignal>
#include <cstdlib>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

#include <QApplication>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

using namespace rb;
using namespace std::chrono_literals;
const std::string kAll = ".*";

enum class ControlMode {
  DUAL_ARM,
  TORSO,
  WHEEL,
};

ControlMode control_mode = ControlMode::DUAL_ARM;

auto master_arm = std::make_shared<upc::MasterArm>("/dev/rby1_master_arm");

void signalHandler(int signum) {

  master_arm->StopControl();
  exit(signum);
}

void onButton1Clicked() {
  control_mode = ControlMode::DUAL_ARM;
  qDebug("Control Mode: Dual Arm!");
}

void onButton2Clicked() {
  control_mode = ControlMode::TORSO;
  qDebug("Control Mode: Torso!");
}

void onButton3Clicked() {
  control_mode = ControlMode::WHEEL;
  qDebug("Control Mode: Wheel!");
}

int main_window(int _argc, char** _argv) {

  QApplication app(_argc, _argv);

  // Create a main window widget
  QWidget window;
  window.setWindowTitle("Master Arm Control Window");

  // Create two buttons
  QPushButton* button_dual_arm_joint_mapping = new QPushButton("Dual Arm Joint Mapping");
  QPushButton* button_torso_task_mapping = new QPushButton("Torso Task Mapping");
  QPushButton* button_wheel_velocity_mapping = new QPushButton("Wheel Velocity Mapping");
  QPushButton* button_test = new QPushButton("QUIT");

  // Connect the buttons to their respective functions
  QObject::connect(button_dual_arm_joint_mapping, &QPushButton::clicked, &onButton1Clicked);
  QObject::connect(button_torso_task_mapping, &QPushButton::clicked, &onButton2Clicked);
  QObject::connect(button_wheel_velocity_mapping, &QPushButton::clicked, &onButton3Clicked);
  QObject::connect(button_test, &QPushButton::clicked, &app, &QApplication::quit);

  // Set up a vertical layout and add the buttons
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(button_dual_arm_joint_mapping);
  layout->addWidget(button_torso_task_mapping);
  layout->addWidget(button_wheel_velocity_mapping);
  layout->addWidget(button_test);

  // Set the layout for the main window and show it
  window.setLayout(layout);
  window.resize(300, 200);

  window.show();

  return app.exec();
}

std::pair<Eigen::Vector<double, 3>, Eigen::Vector<double, 3>> calc_diff(
    std::shared_ptr<dyn::Robot<upc::MasterArm::kDOF>> robot, std::shared_ptr<dyn::State<upc::MasterArm::kDOF>> state,
    Eigen::Vector<double, upc::MasterArm::kDOF> q_target, Eigen::Vector<double, upc::MasterArm::kDOF / 2> q_right,
    Eigen::Vector<double, upc::MasterArm::kDOF / 2> q_left) {
  Eigen::Vector<double, upc::MasterArm::kDOF> q;
  Eigen::Matrix<double, 4, 4> T_right, T_left, T_right_target, T_left_target;

  q.topRows(upc::MasterArm::kDOF / 2) = q_right;
  q.bottomRows(upc::MasterArm::kDOF / 2) = q_left;

  state->SetQ(q_target);
  robot->ComputeForwardKinematics(state);
  T_right_target = robot->ComputeTransformation(state, 0, 7);
  T_left_target = robot->ComputeTransformation(state, 0, 14);

  state->SetQ(q);
  robot->ComputeForwardKinematics(state);
  T_right = robot->ComputeTransformation(state, 0, 7);
  T_left = robot->ComputeTransformation(state, 0, 14);

  Eigen::Matrix<double, 4, 4> T_right_diff = T_right_target.inverse() * T_right;
  Eigen::Matrix<double, 4, 4> T_left_diff = T_left_target.inverse() * T_left;

  Eigen::Vector<double, 3> diff_right, diff_left;
  diff_right.topRows(3) = math::SE3::Log(T_right_diff).topRows(3);
  diff_left.topRows(3) = math::SE3::Log(T_left_diff).topRows(3);

  int maxIndex;
  Eigen::Vector<double, 3> unit_axis;

  diff_right.cwiseAbs().maxCoeff(&maxIndex);
  unit_axis.setZero();
  unit_axis(maxIndex) = 1.;
  diff_right = diff_right.cwiseProduct(unit_axis);

  diff_left.cwiseAbs().maxCoeff(&maxIndex);
  unit_axis.setZero();
  unit_axis(maxIndex) = 1.;
  diff_left = diff_left.cwiseProduct(unit_axis);

  diff_right = diff_right.unaryExpr([](double v) {
    double abs_v = std::abs(v);
    if (abs_v < 0.05) {
      return 0.0;
    } else if (abs_v <= 0.2) {
      double scaled = (abs_v - 0.05) / (0.2 - 0.05);
      return v < 0 ? -scaled : scaled;
    } else {
      return v < 0 ? -1.0 : 1.0;
    }
  });

  diff_left = diff_left.unaryExpr([](double v) {
    double abs_v = std::abs(v);
    if (abs_v < 0.05) {
      return 0.0;
    } else if (abs_v <= 0.2) {
      double scaled = (abs_v - 0.05) / (0.2 - 0.05);
      return v < 0 ? -scaled : scaled;
    } else {
      return v < 0 ? -1.0 : 1.0;
    }
  });

  return std::make_pair(diff_right, diff_left);
}

int main(int argc, char** argv) {

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <server address>" << std::endl;
    return 1;
  }

  std::string address{argv[1]};

  auto robot = Robot<y1_model::A>::Create(address);

  std::cout << "Attempting to connect to the robot..." << std::endl;
  if (!robot->Connect()) {
    std::cerr << "Error: Unable to establish connection to the robot at " << address << std::endl;
    return 1;
  }

  std::this_thread::sleep_for(1s);

  std::cout << "Checking power status..." << std::endl;
  if (!robot->IsPowerOn(kAll)) {
    std::cout << "Power is currently OFF. Attempting to power on..." << std::endl;
    if (!robot->PowerOn(kAll)) {
      std::cerr << "Error: Failed to power on the robot." << std::endl;
      return 1;
    }
    std::cout << "Robot powered on successfully." << std::endl;
  } else {
    std::cout << "Power is already ON." << std::endl;
  }

  std::cout << "Checking servo status..." << std::endl;
  if (!robot->IsServoOn(kAll)) {
    std::cout << "Servo is currently OFF. Attempting to activate servo..." << std::endl;
    if (!robot->ServoOn(kAll)) {
      std::cerr << "Error: Failed to activate servo." << std::endl;
      return 1;
    }
    std::cout << "Servo activated successfully." << std::endl;
  } else {
    std::cout << "Servo is already ON." << std::endl;
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
      return 1;
    }
    std::cout << "Fault reset successfully." << std::endl;
  }
  std::cout << "Control Manager state is normal. No faults detected." << std::endl;

  std::cout << "Enabling the Control Manager..." << std::endl;
  if (!robot->EnableControlManager()) {
    std::cerr << "Error: Failed to enable the Control Manager." << std::endl;
    return 1;
  }
  std::cout << "Control Manager enabled successfully." << std::endl;

  robot->SetParameter("joint_position_command.cutoff_frequency", "10.0");
  std::cout << robot->GetParameter("joint_position_command.cutoff_frequency") << std::endl;


  std::this_thread::sleep_for(1s);

  signal(SIGINT, signalHandler);

  try {
    // Latency timer setting
    upc::InitializeDevice("/dev/rby1_master_arm");
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  master_arm->SetModelPath(MODELS_PATH "/master_arm/model.urdf");
  master_arm->SetControlPeriod(0.01);

  auto active_ids = master_arm->Initialize();
  if (active_ids.size() != upc::MasterArm::kDOF + 2) {
    return 1;
  }
  auto gain_list = master_arm->GetMasterRightArmPositionPIDGains();

  for (auto i = 0; i < gain_list.size(); i++) {
    std::cout << "RIGHT [" << i << "]: " << gain_list[i].p_gain << ", " << gain_list[i].i_gain << ", "
              << gain_list[i].d_gain << std::endl;
  }

  gain_list = master_arm->GetMasterLeftArmPositionPIDGains();
  for (auto i = 0; i < gain_list.size(); i++) {
    std::cout << "LEFT [" << i << "]: " << gain_list[i].p_gain << ", " << gain_list[i].i_gain << ", "
              << gain_list[i].d_gain << std::endl;
  }

  bool init = false;
  Eigen::Vector<double, upc::MasterArm::kDOF / 2> q_right, q_left;
  q_right.setZero();
  q_left.setZero();

  Eigen::Vector<double, upc::MasterArm::kDOF / 2> q_default_right, q_default_left;

  q_default_right << 45, -30, 0, -135, -30, 90, 0;
  q_default_left << 45, 30, 0, -135, 30, 90, 0;

  q_default_right = q_default_right * 3.141592 / 180.;
  q_default_left = q_default_left * 3.141592 / 180.;

  Eigen::Matrix<double, 14, 1> q_joint_ref;
  q_joint_ref.setZero();
  {
    auto state = robot->GetState();
    q_joint_ref = state.position.block(2 + 6, 0, 14, 1);
  }

  auto dyn = robot->GetDynamics();
  auto dyn_state = dyn->MakeState({"base", "link_torso_5", "ee_right", "ee_left"}, y1_model::A::kRobotJointNames);

  Eigen::Vector<double, 14> q_lower_limit, q_upper_limit;
  q_upper_limit = dyn->GetLimitQUpper(dyn_state).block(2 + 6, 0, 14, 1);
  q_lower_limit = dyn->GetLimitQLower(dyn_state).block(2 + 6, 0, 14, 1);

  double right_arm_minimum_time = 1.;
  double left_arm_minimum_time = 1.;
  double torso_minimum_time = 1.;
  double wheel_minimum_time = 1.;
  double lpf_update_ratio = 0.2;

  std::cout << "Start to send command to robot." << std::endl;

  Eigen::Matrix<double, 20, 1> q_joint_ref_20x1;
  q_joint_ref_20x1.setZero();
  q_joint_ref_20x1.block(0, 0, 6, 1) << 0, 30, -60, 30, 0, 0;
  q_joint_ref_20x1 *= 3.141592 / 180.;
  
  robot
      ->SendCommand(RobotCommandBuilder().SetCommand(
          ComponentBasedCommandBuilder().SetBodyCommand(BodyComponentBasedCommandBuilder().SetTorsoCommand(
              JointPositionCommandBuilder()
                  .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(1.))
                  .SetMinimumTime(5)
                  .SetPosition(q_joint_ref_20x1.topRows(6))))))
      ->Get();
  

  double update_ratio = 0.;
  ControlMode control_mode_old = control_mode;

  std::unique_ptr<RobotCommandStreamHandler<y1_model::A>> stream;

  Eigen::Matrix<double, 4, 4> T_torso, T_right, T_left;
  T_torso.setIdentity();
  T_right.setIdentity();
  T_left.setIdentity();

  Eigen::Vector<double, 3> add_pos = Eigen::Vector<double, 3>::Zero();
  Eigen::Vector<double, 3> add_ori = Eigen::Vector<double, 3>::Zero();

  int control_cnt = 0;

  master_arm->StartControl([&](const upc::MasterArm::State& state) {
    upc::MasterArm::ControlInput input;

    if (!init) {
      q_right = state.q_joint(Eigen::seq(0, 6));
      q_left = state.q_joint(Eigen::seq(7, 13));
      q_joint_ref.topRows(7) = q_right;
      q_joint_ref.bottomRows(7) = q_left;
      init = true;
    }

    if (control_mode_old != control_mode) {
      update_ratio = 1.0;

      if (stream != nullptr) {
        std::cout<<"RESETRESETRESETRESETRESETRESETRESET111"<<std::endl;
        stream.reset();
        std::cout<<"RESETRESETRESETRESETRESETRESETRESET222"<<std::endl;
      }

      std::cout<<"GetState111"<<std::endl;
      auto state = robot->GetState();
      std::cout<<"GetState222"<<std::endl;
      q_joint_ref_20x1 = state.position.block(2, 0, 20, 1).matrix();
      Eigen::Vector<double, 24> dyn_q;
      dyn_q.setZero();
      dyn_q.block(2, 0, 20, 1) = q_joint_ref_20x1;
      dyn_state->SetQ(dyn_q);
      std::cout<<"ComputeForwardKinematics111"<<std::endl;
      dyn->ComputeForwardKinematics(dyn_state);
      std::cout<<"ComputeForwardKinematics222"<<std::endl;

      T_torso = dyn->ComputeTransformation(dyn_state, 0, 1);
      T_right = dyn->ComputeTransformation(dyn_state, 0, 2);
      T_left = dyn->ComputeTransformation(dyn_state, 0, 3);

      add_pos.setZero();
      add_ori.setZero();

      right_arm_minimum_time = 1.0;
      left_arm_minimum_time = 1.0;
      torso_minimum_time = 1.0;
      wheel_minimum_time = 5.0;
      control_cnt = 0;
    }

    control_mode_old = control_mode;

    if (control_mode == ControlMode::DUAL_ARM) {
      // dual arm control mode
      std::cout<<"DUAL_ARM START"<<std::endl;
      if (stream == nullptr || stream->IsDone()) {
        std::cout<<"ArmStream11111111111111"<<std::endl;
        stream = robot->CreateCommandStream();
        std::cout<<"ArmStream22222222222222"<<std::endl;
      }

      if (state.button_right.button == 1) {
        input.target_operation_mode(Eigen::seq(0, 6)).setConstant(DynamixelBus::kCurrentControlMode);
        input.target_torque(Eigen::seq(0, 6)) = state.gravity_term(Eigen::seq(0, 6));
        q_right = state.q_joint(Eigen::seq(0, 6));
      } else {
        input.target_operation_mode(Eigen::seq(0, 6)).setConstant(DynamixelBus::kCurrentBasedPositionControlMode);
        input.target_position(Eigen::seq(0, 6)) = q_right * (1. - update_ratio) + q_default_right * update_ratio;
      }

      if (state.button_left.button == 1) {
        input.target_operation_mode(Eigen::seq(7, 13)).setConstant(DynamixelBus::kCurrentControlMode);
        input.target_torque(Eigen::seq(7, 13)) = state.gravity_term(Eigen::seq(7, 13));
        q_left = state.q_joint(Eigen::seq(7, 13));
      } else {
        input.target_operation_mode(Eigen::seq(7, 13)).setConstant(DynamixelBus::kCurrentBasedPositionControlMode);
        input.target_position(Eigen::seq(7, 13)) = q_left * (1. - update_ratio) + q_default_left * update_ratio;
      }

            if (control_cnt++ > 30) {
        control_cnt = 999;


      q_joint_ref_20x1.block(6, 0, 14, 1) =
          q_joint_ref_20x1.block(6, 0, 14, 1) * (1 - lpf_update_ratio) + state.q_joint * lpf_update_ratio;

      Eigen::Matrix<double, 24, 1> q_joint_for_collision;
      q_joint_for_collision.setZero();
      q_joint_for_collision.block(2, 0, 6, 1) = q_joint_ref_20x1.block(0, 0, 6, 1);
      q_joint_for_collision.block(2 + 6, 0, 14, 1) = state.q_joint;

      dyn_state->SetQ(q_joint_for_collision);
      dyn->ComputeForwardKinematics(dyn_state);
      auto res_col = dyn->DetectCollisionsOrNearestLinks(dyn_state, 1);
      bool is_coliision = false;

      if (res_col[0].distance < 0.02) {
        is_coliision = true;
      }

      if (state.button_right.button && !is_coliision) {
        //right hand position control mode
        q_joint_ref.block(0, 0, 7, 1) =
            q_joint_ref.block(0, 0, 7, 1) * (1 - lpf_update_ratio) + state.q_joint.block(0, 0, 7, 1) * lpf_update_ratio;
        // q_joint_ref.block(0, 0, 7, 1) = state.q_joint.block(0, 0, 7, 1);
      } else {
        right_arm_minimum_time = 1.0;
      }

      if (state.button_left.button && !is_coliision) {
        //left hand position control mode
        q_joint_ref.block(7, 0, 7, 1) =
            q_joint_ref.block(7, 0, 7, 1) * (1 - lpf_update_ratio) + state.q_joint.block(7, 0, 7, 1) * lpf_update_ratio;
        // q_joint_ref.block(7, 0, 7, 1) = state.q_joint.block(7, 0, 7, 1);
      } else {
        left_arm_minimum_time = 1.0;
      }

      for (int i = 0; i < 14; i++) {
        q_joint_ref(i) = std::clamp(q_joint_ref(i), q_lower_limit(i), q_upper_limit(i));
      }
      Eigen::Vector<double, 7> target_position_left = q_joint_ref.block(7, 0, 7, 1);
      Eigen::Vector<double, 7> target_position_right = q_joint_ref.block(0, 0, 7, 1);
      Eigen::Vector<double, 7> acc_limit, vel_limit;

      acc_limit.setConstant(1200.0);
      acc_limit *= 3.141592 / 180.;

      vel_limit << 160, 160, 160, 160, 330, 330, 330;
      vel_limit *= 3.141592 / 180.;

      right_arm_minimum_time *= 0.99;
      right_arm_minimum_time = std::max(right_arm_minimum_time, 0.01);

      left_arm_minimum_time *= 0.99;
      left_arm_minimum_time = std::max(left_arm_minimum_time, 0.01);

      try {
        RobotCommandBuilder command_builder;

        command_builder.SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
            BodyComponentBasedCommandBuilder()
                .SetLeftArmCommand(JointPositionCommandBuilder()
                                       .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(4.))
                                       .SetMinimumTime(left_arm_minimum_time)
                                       .SetPosition(target_position_left)
                                       .SetVelocityLimit(vel_limit)
                                       .SetAccelerationLimit(acc_limit))
                .SetRightArmCommand(JointPositionCommandBuilder()
                                        .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(4.))
                                        .SetMinimumTime(right_arm_minimum_time)
                                        .SetPosition(target_position_right)
                                        .SetVelocityLimit(vel_limit)
                                        .SetAccelerationLimit(acc_limit))));

        std::cout<<"arm11111111111111111111111"<<std::endl;
        stream->SendCommand(command_builder);
        std::cout<<"arm22222222222222222222222"<<std::endl;
      } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
      }
            }

    } else if (control_mode == ControlMode::TORSO) {
      // torso control mode
      std::cout<<"TORSO START"<<std::endl;
      if (stream == nullptr || stream->IsDone()) {
        std::cout<<"TorsoStream11111111111111"<<std::endl;
        stream = robot->CreateCommandStream();
        std::cout<<"TorsoStream2222222222222222"<<std::endl;
      }

      input.target_operation_mode(Eigen::seq(0, 6)).setConstant(DynamixelBus::kCurrentBasedPositionControlMode);
      input.target_position(Eigen::seq(0, 6)) = q_default_right * (1. - update_ratio) + q_right * update_ratio;

      input.target_operation_mode(Eigen::seq(7, 13)).setConstant(DynamixelBus::kCurrentBasedPositionControlMode);
      input.target_position(Eigen::seq(7, 13)) = q_default_left * (1. - update_ratio) + q_left * update_ratio;

      q_right = state.q_joint(Eigen::seq(0, 6));
      q_left = state.q_joint(Eigen::seq(7, 13));

      if (control_cnt++ > 30) {
        control_cnt = 999;
        auto ret =
            calc_diff(master_arm->get_dny_robot(), master_arm->get_dyn_state(), input.target_position, q_right, q_left);

        Eigen::Vector<double, 3> rpy_right = ret.first;
        Eigen::Vector<double, 3> rpy_left = ret.second;

        Eigen::Vector<double, 3> temp_rpy_right;
        temp_rpy_right(0) = -rpy_right(1);
        temp_rpy_right(1) = rpy_right(0);
        temp_rpy_right(2) = rpy_right(2);

        double lpf_update_ratio2 = 0.2;
        add_pos = add_pos * (1. - lpf_update_ratio2) + temp_rpy_right * 0.005 * lpf_update_ratio2;
        add_ori = add_ori * (1. - lpf_update_ratio2) + rpy_left * 0.01 * lpf_update_ratio2;

        Eigen::Vector<double, 3> temp_ori = math::SO3::Log(T_torso.block(0, 0, 3, 3)) + add_ori;
        Eigen::Vector<double, 3> temp_pos = T_torso.block(0, 3, 3, 1) + add_pos;

        for (int i = 0; i < 3; i++) {
          temp_ori(i) = std::min(std::max(temp_ori(i), -1.), 1.);
          if (i == 2) {
            temp_pos(i) = std::min(std::max(temp_pos(i), 0.9), 1.3);
          } else {
            temp_pos(i) = std::min(std::max(temp_pos(i), -0.2), 0.2);
          }
        }

        T_torso.block(0, 0, 3, 3) = math::SO3::Exp(temp_ori);
        T_torso.block(0, 3, 3, 1) = temp_pos;
        T_torso.block(0, 3, 3, 1) = temp_pos;

        torso_minimum_time *= 0.99;
        torso_minimum_time = std::max(torso_minimum_time, 0.01);

        try {
          RobotCommandBuilder command_builder;
          double stop_orientation_tracking_error = 1e-5;
          double stop_position_tracking_error = 1e-5;
          double minimum_time = 0.01;

          command_builder.SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
              BodyCommandBuilder().SetCommand(BodyComponentBasedCommandBuilder().SetTorsoCommand(
                  CartesianCommandBuilder()
                      .AddTarget("base", "link_torso_5", T_torso, 1, 3.141592, 3)
                      .SetMinimumTime(torso_minimum_time)
                      .SetStopOrientationTrackingError(stop_orientation_tracking_error)
                      .SetStopPositionTrackingError(stop_position_tracking_error)
                      .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(100))))));

          std::cout<<"torso11111111111111111111111"<<std::endl;
          stream->SendCommand(command_builder);
          std::cout<<"torso22222222222222222222222"<<std::endl;
        } catch (const std::exception& e) {
          std::cerr << "Error: " << e.what() << std::endl;
        }
      }

    } else if (control_mode == ControlMode::WHEEL) {
      // wheel control mode
      if (stream == nullptr || stream->IsDone()) {
        stream = robot->CreateCommandStream();
      }

      input.target_operation_mode(Eigen::seq(0, 6)).setConstant(DynamixelBus::kCurrentBasedPositionControlMode);
      input.target_position(Eigen::seq(0, 6)) = q_default_right * (1. - update_ratio) + q_right * update_ratio;

      input.target_operation_mode(Eigen::seq(7, 13)).setConstant(DynamixelBus::kCurrentBasedPositionControlMode);
      input.target_position(Eigen::seq(7, 13)) = q_default_left * (1. - update_ratio) + q_left * update_ratio;

      q_right = state.q_joint(Eigen::seq(0, 6));
      q_left = state.q_joint(Eigen::seq(7, 13));

      if (control_cnt++ > 30) {
        control_cnt = 999;

        auto ret =
            calc_diff(master_arm->get_dny_robot(), master_arm->get_dyn_state(), input.target_position, q_right, q_left);

        double v_ref_wheel_right = ret.first(1);
        double v_ref_wheel_left = ret.second(1);

        Eigen::Vector<double, 2> wheel_velocity, wheel_acceleration;
        wheel_velocity << v_ref_wheel_right * 3.14 * 2., v_ref_wheel_left * 3.14 * 2.;
        wheel_acceleration.setConstant(100./10.);

        wheel_minimum_time *= 0.99;
        wheel_minimum_time = std::max(wheel_minimum_time, 0.01);
        try {
          RobotCommandBuilder command_builder;
          command_builder.SetCommand(
              ComponentBasedCommandBuilder().SetMobilityCommand(MobilityCommandBuilder().SetCommand(
                  JointVelocityCommandBuilder()
                      .SetVelocity(wheel_velocity)
                      .SetAccelerationLimit(wheel_acceleration)
                      .SetMinimumTime(wheel_minimum_time)
                      .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(100)))));

          // std::cout<<"wheel11111111111111111111111"<<std::endl;
          stream->SendCommand(command_builder);
          // std::cout<<"wheel22222222222222222222222"<<std::endl;

        } catch (const std::exception& e) {
          std::cerr << "Error: " << e.what() << std::endl;
        }
      }
    }

    update_ratio = update_ratio * 0.95;

    if (update_ratio < 0) {
      update_ratio = 0;
    }

    static int cnt = 0;

    if (cnt++ % 10 == 0) {
      std::cout << "button : [" << state.button_right.button << ", " << state.button_left.button << "]" << std::endl;
      std::cout << "trigger : [" << state.button_right.trigger << ", " << state.button_left.trigger << "]" << std::endl;
    }

    return input;
  });

  { auto ret = main_window(argc, argv); }

  master_arm->StopControl();

  std::cout << "The program has been terminated" << std::endl;

  return 0;
}