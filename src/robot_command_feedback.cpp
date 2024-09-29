#include "rby1-sdk/robot_command_feedback.h"

#include "rb/api/robot_command.pb.h"

namespace rb {

class RobotCommandFeedbackParserImpl {
 public:
  void ParseCommandHeaderFeedback(CommandHeaderFeedback& self, api::CommandHeader::Feedback* feedback) {
    self.valid_ = true;

    self.finished_ = feedback->finished();
  }

  void ParseStopCommandFeedback(StopCommandFeedback& self, api::StopCommand::Feedback* feedback) {
    self.valid_ = true;

    if (feedback->has_command_header_feedback()) {
      ParseCommandHeaderFeedback(self.command_header_, feedback->mutable_command_header_feedback());
    }
  }

  void ParseRealtimeControlCommandFeedback(RealtimeControlCommandFeedback& self,
                                           api::RealTimeControlCommand::Feedback* feedback) {
    self.valid_ = true;

    if (feedback->has_command_header_feedback()) {
      ParseCommandHeaderFeedback(self.command_header_, feedback->mutable_command_header_feedback());
    }
  }

  void ParseSE2VelocityCommandFeedback(SE2VelocityCommandFeedback& self, api::SE2VelocityCommand::Feedback* feedback) {
    self.valid_ = true;

    if (feedback->has_command_header_feedback()) {
      ParseCommandHeaderFeedback(self.command_header_, feedback->mutable_command_header_feedback());
    }
  }

  void ParseJogCommandFeedback(JogCommandFeedback& self, api::JogCommand::Feedback* feedback) {
    self.valid_ = true;

    if (feedback->has_command_header_feedback()) {
      ParseCommandHeaderFeedback(self.command_header_, feedback->mutable_command_header_feedback());
    }
    self.target_joint_name_ = feedback->target_joint_name();
  }

  void ParseJointVelocityCommandFeedback(JointVelocityCommandFeedback& self,
                                         api::JointVelocityCommand::Feedback* feedback) {
    self.valid_ = true;

    if (feedback->has_command_header_feedback()) {
      ParseCommandHeaderFeedback(self.command_header_, feedback->mutable_command_header_feedback());
    }
  }

  void ParseJointPositionCommandFeedback(JointPositionCommandFeedback& self,
                                         api::JointPositionCommand::Feedback* feedback) {
    self.valid_ = true;

    if (feedback->has_command_header_feedback()) {
      ParseCommandHeaderFeedback(self.command_header_, feedback->mutable_command_header_feedback());
    }
  }

  void ParseCartesianCommandFeedback(CartesianCommandFeedback& self, api::CartesianCommand::Feedback* feedback) {
    self.valid_ = true;

    if (feedback->has_command_header_feedback()) {
      ParseCommandHeaderFeedback(self.command_header_, feedback->mutable_command_header_feedback());
    }
    for (const auto& tracking_error : feedback->tracking_errors()) {
      self.tracking_errors_.push_back({tracking_error.position_error(), tracking_error.rotation_error()});
    }
  }

  void ParseGravityCompensationCommandFeedback(GravityCompensationCommandFeedback& self,
                                               api::GravityCompensationCommand::Feedback* feedback) {
    self.valid_ = true;

    if (feedback->has_command_header_feedback()) {
      ParseCommandHeaderFeedback(self.command_header_, feedback->mutable_command_header_feedback());
    }
  }

  void ParseImpedanceControlCommandFeedback(ImpedanceControlCommandFeedback& self,
                                            api::ImpedanceControlCommand::Feedback* feedback) {
    self.valid_ = true;

    if (feedback->has_command_header_feedback()) {
      ParseCommandHeaderFeedback(self.command_header_, feedback->mutable_command_header_feedback());
    }
    self.tracking_error_.position_error = feedback->tracking_error().position_error();
    self.tracking_error_.rotation_error = feedback->tracking_error().rotation_error();
  }

  void ParseOptimalControlCommandFeedback(OptimalControlCommandFeedback& self,
                                          api::OptimalControlCommand::Feedback* feedback) {
    self.valid_ = true;

    if (feedback->has_command_header_feedback()) {
      ParseCommandHeaderFeedback(self.command_header_, feedback->mutable_command_header_feedback());
    }
    self.total_cost_ = feedback->total_cost();
    self.cartesian_costs_ = std::vector<double>(
        feedback->cartesian_costs().begin(), feedback->cartesian_costs().begin() + feedback->cartesian_costs().size());
    self.center_of_mass_cost_ = feedback->center_of_mass_cost();
    self.joint_position_costs_ =
        std::vector<double>(feedback->joint_position_costs().begin(),
                            feedback->joint_position_costs().begin() + feedback->joint_position_costs().size());
  }

  void ParseWholeBodyCommandFeedback(WholeBodyCommandFeedback& self, api::WholeBodyCommand::Feedback* feedback) {
    self.valid_ = true;

    if (feedback->has_command_header_feedback()) {
      ParseCommandHeaderFeedback(self.command_header_, feedback->mutable_command_header_feedback());
    }

    switch (feedback->feedback_case()) {
      case api::WholeBodyCommand_Feedback::kStopCommandFeedback:
        ParseStopCommandFeedback(self.stop_command_, feedback->mutable_stop_command_feedback());
        break;
      case api::WholeBodyCommand_Feedback::kRealTimeControlCommandFeedback:
        ParseRealtimeControlCommandFeedback(self.realtime_control_command_,
                                            feedback->mutable_real_time_control_command_feedback());
        break;
      case api::WholeBodyCommand_Feedback::FEEDBACK_NOT_SET:
        break;
    }
  }

  void ParseHeadCommandFeedback(HeadCommandFeedback& self, api::HeadCommand::Feedback* feedback) {
    self.valid_ = true;

    if (feedback->has_command_header_feedback()) {
      ParseCommandHeaderFeedback(self.command_header_, feedback->mutable_command_header_feedback());
    }

    switch (feedback->feedback_case()) {
      case api::HeadCommand_Feedback::kJointPositionCommandFeedback:
        ParseJointPositionCommandFeedback(self.joint_position_command_,
                                          feedback->mutable_joint_position_command_feedback());
        break;
      case api::HeadCommand_Feedback::FEEDBACK_NOT_SET:
        break;
    }
  }

  void ParseBodyCommandFeedback(BodyCommandFeedback& self, api::BodyCommand::Feedback* feedback) {
    self.valid_ = true;

    if (feedback->has_command_header_feedback()) {
      ParseCommandHeaderFeedback(self.command_header_, feedback->mutable_command_header_feedback());
    }

    switch (feedback->feedback_case()) {
      case api::BodyCommand_Feedback::kJointPositionCommandFeedback:
        ParseJointPositionCommandFeedback(self.joint_position_command_,
                                          feedback->mutable_joint_position_command_feedback());
        break;
      case api::BodyCommand_Feedback::kOptimalControlCommandFeedback:
        ParseOptimalControlCommandFeedback(self.optimal_control_command_,
                                           feedback->mutable_optimal_control_command_feedback());
        break;
      case api::BodyCommand_Feedback::kGravityCompensationCommandFeedback:
        ParseGravityCompensationCommandFeedback(self.gravity_compensation_command_,
                                                feedback->mutable_gravity_compensation_command_feedback());
        break;
      case api::BodyCommand_Feedback::kCartesianCommandFeedback:
        ParseCartesianCommandFeedback(self.cartesian_command_, feedback->mutable_cartesian_command_feedback());
        break;
      case api::BodyCommand_Feedback::kBodyComponentBasedCommandFeedback:
        ParseBodyComponentBasedCommandFeedback(self.body_component_based_command_,
                                               feedback->mutable_body_component_based_command_feedback());
        break;
      case api::BodyCommand_Feedback::FEEDBACK_NOT_SET:
        break;
    }
  }

  void ParseMobilityCommandFeedback(MobilityCommandFeedback& self, api::MobilityCommand::Feedback* feedback) {
    self.valid_ = true;

    if (feedback->has_command_header_feedback()) {
      ParseCommandHeaderFeedback(self.command_header_, feedback->mutable_command_header_feedback());
    }

    switch (feedback->feedback_case()) {
      case api::MobilityCommand_Feedback::kJointVelocityCommandFeedback:
        ParseJointVelocityCommandFeedback(self.joint_velocity_command_,
                                          feedback->mutable_joint_velocity_command_feedback());
        break;
      case api::MobilityCommand_Feedback::kSe2VelocityCommandFeedback:
        ParseSE2VelocityCommandFeedback(self.se2_velocity_command_, feedback->mutable_se2_velocity_command_feedback());
        break;
      case api::MobilityCommand_Feedback::FEEDBACK_NOT_SET:
        break;
    }
  }

  void ParseArmCommandFeedback(ArmCommandFeedback& self, api::ArmCommand::Feedback* feedback) {
    self.valid_ = true;

    if (feedback->has_command_header_feedback()) {
      ParseCommandHeaderFeedback(self.command_header_, feedback->mutable_command_header_feedback());
    }

    switch (feedback->feedback_case()) {
      case api::ArmCommand_Feedback::kJointPositionCommandFeedback:
        ParseJointPositionCommandFeedback(self.joint_position_command_,
                                          feedback->mutable_joint_position_command_feedback());
        break;
      case api::ArmCommand_Feedback::kGravityCompensationCommandFeedback:
        ParseGravityCompensationCommandFeedback(self.gravity_compensation_command_,
                                                feedback->mutable_gravity_compensation_command_feedback());
        break;
      case api::ArmCommand_Feedback::kCartesianCommandFeedback:
        ParseCartesianCommandFeedback(self.cartesian_command_, feedback->mutable_cartesian_command_feedback());
        break;
      case api::ArmCommand_Feedback::kImpedanceControlCommandFeedback:
        ParseImpedanceControlCommandFeedback(self.impedance_control_command_,
                                             feedback->mutable_impedance_control_command_feedback());
        break;
      case api::ArmCommand_Feedback::FEEDBACK_NOT_SET:
        break;
    }
  }

  void ParseTorsoCommandFeedback(TorsoCommandFeedback& self, api::TorsoCommand::Feedback* feedback) {
    self.valid_ = true;

    if (feedback->has_command_header_feedback()) {
      ParseCommandHeaderFeedback(self.command_header_, feedback->mutable_command_header_feedback());
    }

    switch (feedback->feedback_case()) {
      case api::TorsoCommand_Feedback::kJointPositionCommandFeedback:
        ParseJointPositionCommandFeedback(self.joint_position_command_,
                                          feedback->mutable_joint_position_command_feedback());
        break;
      case api::TorsoCommand_Feedback::kGravityCompensationCommandFeedback:
        ParseGravityCompensationCommandFeedback(self.gravity_compensation_command_,
                                                feedback->mutable_gravity_compensation_command_feedback());
        break;
      case api::TorsoCommand_Feedback::kCartesianCommandFeedback:
        ParseCartesianCommandFeedback(self.cartesian_command_, feedback->mutable_cartesian_command_feedback());
        break;
      case api::TorsoCommand_Feedback::kImpedanceControlCommandFeedback:
        ParseImpedanceControlCommandFeedback(self.impedance_control_command_,
                                             feedback->mutable_impedance_control_command_feedback());
        break;
      case api::TorsoCommand_Feedback::kOptimalControlCommandFeedback:
        ParseOptimalControlCommandFeedback(self.optimal_control_command_,
                                           feedback->mutable_optimal_control_command_feedback());
        break;
      case api::TorsoCommand_Feedback::FEEDBACK_NOT_SET:
        break;
    }
  }

  void ParseComponentBasedCommandFeedback(ComponentBasedCommandFeedback& self,
                                          api::ComponentBasedCommand::Feedback* feedback) {
    self.valid_ = true;

    if (feedback->has_command_header_feedback()) {
      ParseCommandHeaderFeedback(self.command_header_, feedback->mutable_command_header_feedback());
    }

    if (feedback->has_head_command_feedback()) {
      ParseHeadCommandFeedback(self.head_command_, feedback->mutable_head_command_feedback());
    }

    if (feedback->has_body_command_feedback()) {
      ParseBodyCommandFeedback(self.body_command_, feedback->mutable_body_command_feedback());
    }

    if (feedback->has_mobility_command_feedback()) {
      ParseMobilityCommandFeedback(self.mobility_command_, feedback->mutable_mobility_command_feedback());
    }
  }

  void ParseBodyComponentBasedCommandFeedback(BodyComponentBasedCommandFeedback& self,
                                              api::BodyComponentBasedCommand::Feedback* feedback) {
    self.valid_ = true;

    if (feedback->has_command_header_feedback()) {
      ParseCommandHeaderFeedback(self.command_header_, feedback->mutable_command_header_feedback());
    }

    if (feedback->has_right_arm_command_feedback()) {
      ParseArmCommandFeedback(self.right_arm_command_, feedback->mutable_right_arm_command_feedback());
    }

    if (feedback->has_left_arm_command_feedback()) {
      ParseArmCommandFeedback(self.left_arm_command_, feedback->mutable_left_arm_command_feedback());
    }

    if (feedback->has_torso_command_feedback()) {
      ParseTorsoCommandFeedback(self.torso_command_, feedback->mutable_torso_command_feedback());
    }
  }

  void Parse(RobotCommandFeedback& self, api::RobotCommand::Feedback* feedback) {
    self.valid_ = true;

    if (feedback->has_command_header_feedback()) {
      ParseCommandHeaderFeedback(self.command_header_, feedback->mutable_command_header_feedback());
    }

    self.status_ = static_cast<RobotCommandFeedback::Status>(feedback->status());
    self.finish_code_ = static_cast<RobotCommandFeedback::FinishCode>(feedback->finish_code());

    switch (feedback->feedback_case()) {
      case api::RobotCommand_Feedback::kWholeBodyCommandFeedback:
        ParseWholeBodyCommandFeedback(self.whole_body_command_, feedback->mutable_whole_body_command_feedback());
        break;
      case api::RobotCommand_Feedback::kComponentBasedCommandFeedback:
        ParseComponentBasedCommandFeedback(self.component_based_command_,
                                           feedback->mutable_component_based_command_feedback());
        break;
      case api::RobotCommand_Feedback::kJogCommandFeedback:
        ParseJogCommandFeedback(self.jog_command_, feedback->mutable_jog_command_feedback());
        break;
      case api::RobotCommand_Feedback::FEEDBACK_NOT_SET:
        break;
    }
  }
};

RobotCommandFeedbackParser::RobotCommandFeedbackParser() : impl_(std::make_unique<RobotCommandFeedbackParserImpl>()) {}

RobotCommandFeedbackParser::~RobotCommandFeedbackParser() = default;

void RobotCommandFeedbackParser::Parse(RobotCommandFeedback& self, void* msg) {
  impl_->Parse(self, static_cast<api::RobotCommand::Feedback*>(msg));
}

}  // namespace rb