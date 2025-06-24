#include <functional>
#include <regex>
#include <utility>

#include "grpcpp/grpcpp.h"
#include "nlohmann/json.hpp"

#include "google/protobuf/util/time_util.h"

#include "rb/api/robot_command.pb.h"
#include "rb/api/robot_info.pb.h"
#include "rb/api/robot_state.pb.h"

#include "rb/api/control_manager_service.grpc.pb.h"
#include "rb/api/file_service.grpc.pb.h"
#include "rb/api/joint_operation_service.grpc.pb.h"
#include "rb/api/led_service.grpc.pb.h"
#include "rb/api/log_service.grpc.pb.h"
#include "rb/api/parameter_service.grpc.pb.h"
#include "rb/api/ping_service.grpc.pb.h"
#include "rb/api/power_service.grpc.pb.h"
#include "rb/api/robot_command_service.grpc.pb.h"
#include "rb/api/robot_info_service.grpc.pb.h"
#include "rb/api/robot_state_service.grpc.pb.h"
#include "rb/api/serial_service.grpc.pb.h"
#include "rb/api/system_service.grpc.pb.h"
#include "rb/api/tool_flange_service.grpc.pb.h"

#include "rby1-sdk/base/event_loop.h"
#include "rby1-sdk/base/time_util.h"
#include "rby1-sdk/log.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/net/real_time_control_protocol.h"
#include "rby1-sdk/net/udp_server.h"
#include "rby1-sdk/robot.h"

using namespace std::chrono_literals;

namespace {

void InitializeRequestHeader(rb::api::RequestHeader* req_header) {
  const auto& ts = rb::TimepointToTimespec(std::chrono::system_clock::now());

  req_header->mutable_request_timestamp()->set_seconds(ts.tv_sec);
  req_header->mutable_request_timestamp()->set_nanos(ts.tv_nsec);
}

struct timespec DurationToTimespec(const google::protobuf::Duration& duration) {
  struct timespec t{};

  t.tv_sec = duration.seconds();
  t.tv_nsec = duration.nanos();
  return t;
}

int64_t TimestampInNs(const google::protobuf::Timestamp& time) {
  return (int64_t)time.seconds() * (int64_t)rb::kNanosecondsInSecond + (int64_t)time.nanos();
}

std::string gRPCStatusToString(grpc::StatusCode status_code) {
  switch (status_code) {
    case grpc::OK: {
      return "OK";
    }
    case grpc::CANCELLED: {
      return "CANCELLED";
    }
    case grpc::UNKNOWN: {
      return "UNKNOWN";
    }
    case grpc::INVALID_ARGUMENT: {
      return "INVALID_ARGUMENT";
    }
    case grpc::DEADLINE_EXCEEDED: {
      return "DEADLINE_EXCEEDED";
    }
    case grpc::NOT_FOUND: {
      return "NOT_FOUND";
    }
    case grpc::ALREADY_EXISTS: {
      return "ALREADY_EXISTS";
    }
    case grpc::PERMISSION_DENIED: {
      return "PERMISSION_DENIED";
    }
    case grpc::UNAUTHENTICATED: {
      return "UNAUTHENTICATED";
    }
    case grpc::RESOURCE_EXHAUSTED: {
      return "RESOURCE_EXHAUSTED";
    }
    case grpc::FAILED_PRECONDITION: {
      return "FAILED_PRECONDITION";
    }
    case grpc::ABORTED: {
      return "ABORTED";
    }
    case grpc::OUT_OF_RANGE: {
      return "OUT_OF_RANGE";
    }
    case grpc::UNIMPLEMENTED: {
      return "UNIMPLEMENTED";
    }
    case grpc::INTERNAL: {
      return "INTERNAL";
    }
    case grpc::UNAVAILABLE: {
      return "UNAVAILABLE";
    }
    case grpc::DATA_LOSS: {
      return "DATA_LOSS";
    }
    case grpc::DO_NOT_USE: {
      return "DO_NOT_USE";
    }
  }
  return "UNKNOWN";
}

}  // namespace

namespace rb::api {

rb::ControlManagerState ProtoToControlManagerState(const api::ControlManagerState& msg) {
  rb::ControlManagerState cms{};

  cms.state = static_cast<rb::ControlManagerState::State>(msg.state());
  cms.time_scale = msg.time_scale();
  cms.control_state = static_cast<rb::ControlManagerState::ControlState>(msg.control_state());
  for (const auto& idx : msg.enabled_joint_idx()) {
    cms.enabled_joint_idx.push_back(idx);
  }
  cms.unlimited_mode_enabled = msg.unlimited_mode_enabled();

  return cms;
}

rb::BatteryInfo ProtoToBatteryInfo(const api::BatteryInfo& /* msg */) {
  return {};
}

rb::PowerInfo ProtoToPowerInfo(const api::PowerInfo& msg) {
  rb::PowerInfo pi;

  pi.name = msg.name();

  return pi;
}

rb::EMOInfo ProtoToEMOInfo(const api::EMOInfo& msg) {
  rb::EMOInfo ei;

  ei.name = msg.name();

  return ei;
}

rb::JointInfo ProtoToJointInfo(const api::JointInfo& msg) {
  rb::JointInfo ji;

  ji.name = msg.name();
  ji.has_brake = msg.has_brake();
  ji.product_name = msg.product_name();
  ji.firmware_version = msg.firmware_version();

  return ji;
}

rb::RobotInfo ProtoToRobotInfo(const api::RobotInfo& msg) {
  rb::RobotInfo info;

  info.version = msg.version();

  info.sdk_version = msg.sdk_version();

  info.robot_version = msg.robot_version();

  info.robot_model_name = msg.robot_model_name();

  info.robot_model_version = msg.robot_model_version();

  info.sdk_commit_id = msg.sdk_commit_id();

  if (msg.has_battery_info()) {
    info.battery_info = ProtoToBatteryInfo(msg.battery_info());
  }

  for (const auto& pi : msg.power_infos()) {
    info.power_infos.push_back(ProtoToPowerInfo(pi));
  }

  for (const auto& ei : msg.emo_infos()) {
    info.emo_infos.push_back(ProtoToEMOInfo(ei));
  }

  info.degree_of_freedom = msg.degree_of_freedom();

  for (const auto& ji : msg.joint_infos()) {
    info.joint_infos.push_back(ProtoToJointInfo(ji));
  }

  for (const auto& idx : msg.mobility_joint_idx()) {
    info.mobility_joint_idx.push_back(idx);
  }

  for (const auto& idx : msg.body_joint_idx()) {
    info.body_joint_idx.push_back(idx);
  }

  for (const auto& idx : msg.head_joint_idx()) {
    info.head_joint_idx.push_back(idx);
  }

  for (const auto& idx : msg.torso_joint_idx()) {
    info.torso_joint_idx.push_back(idx);
  }

  for (const auto& idx : msg.right_arm_joint_idx()) {
    info.right_arm_joint_idx.push_back(idx);
  }

  for (const auto& idx : msg.left_arm_joint_idx()) {
    info.left_arm_joint_idx.push_back(idx);
  }

  return info;
}

}  // namespace rb::api

namespace rb {

template <typename T>
class RobotCommandHandlerImpl {
 public:
  RobotCommandHandlerImpl(std::shared_ptr<RobotImpl<T>> robot, const rb::api::RobotCommandRequest& req)
      : robot_(std::move(robot)), done_(false) {
    robot_->robot_command_service_->async()->RobotCommand(&context_, &req, &res_,
                                                          [=](grpc::Status status) { OnResponseReceived(status); });
  }

  ~RobotCommandHandlerImpl() {
    Cancel();
    Wait();
  }

  bool IsDone() const { return done_.load(); }

  void Wait() {
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock, [this] { return done_.load(); });
  }

  bool WaitFor(int timeout_ms) {
    std::unique_lock<std::mutex> lock(mtx_);
    return cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] { return done_.load(); });
  }

  void Cancel() {
    if (!IsDone()) {
      context_.TryCancel();
    }
  }

  api::RobotCommand::Feedback Get() {
    Wait();
    if (!status_.ok()) {
      throw std::runtime_error("gRPC call failed: " + status_.error_message());
    }
    return res_.feedback();
  }

  grpc::Status GetStatus() const { return status_; }

  void OnResponseReceived(grpc::Status status) {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      status_ = std::move(status);
      done_ = true;
    }
    cv_.notify_all();
  }

 private:
  std::shared_ptr<RobotImpl<T>> robot_;
  grpc::ClientContext context_;
  rb::api::RobotCommandResponse res_;
  grpc::Status status_;
  std::atomic<bool> done_;
  std::mutex mtx_;
  std::condition_variable cv_;
};

template <typename T>
class RobotCommandStreamHandlerImpl
    : public grpc::ClientBidiReactor<api::RobotCommandRequest, api::RobotCommandResponse> {
 public:
  RobotCommandStreamHandlerImpl(std::shared_ptr<RobotImpl<T>> robot, api::RobotCommandService::Stub* stub, int priority)
      : robot_(std::move(robot)),
        priority_(priority),
        done_(false),
        first_command_(true),
        write_done_(true),
        read_done_(true) {
    stub->async()->RobotCommandStream(&context_, this);
    StartCall();
  }

  ~RobotCommandStreamHandlerImpl() override {
    Cancel();
    Wait();
  }

  bool IsDone() const { return done_.load(); }

  void Wait() {
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock, [this] { return done_.load(); });
  }

  bool WaitFor(int timeout_ms) {
    std::unique_lock<std::mutex> lock(mtx_);
    return cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] { return done_.load(); });
  }

  api::RobotCommand::Feedback SendCommand(const RobotCommandBuilder& builder, int timeout_ms = 1000) {
    {
      std::unique_lock<std::mutex> lock(command_mtx_);
      if (done_) {
        throw std::runtime_error("This stream is expired");
      }
      if (write_done_.load() && read_done_.load())
        ;
      else {
        throw std::runtime_error("Last command does not finish yet");
      }

      write_done_.store(false);
      read_done_.store(false);
    }

    api::RobotCommandRequest req;
    req.set_allocated_robot_command(static_cast<api::RobotCommand::Request*>(builder.Build()));
    if (first_command_) {
      req.set_priority(priority_);
      first_command_ = false;
    }

    StartWrite(&req);
    StartRead(&res_);

    {
      std::unique_lock<std::mutex> lock(command_mtx_);
      if (!command_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                                [this] { return write_done_.load() && read_done_.load(); })) {
        Cancel();
        Wait();
        throw std::runtime_error("Timeout");
      }

      return res_.feedback();
    }
  }

  api::RobotCommand::Feedback RequestFeedback(int timeout_ms = 1000) {
    if (first_command_) {
      throw std::runtime_error("Send command first");
    }

    return SendCommand(rb::RobotCommandBuilder(), timeout_ms);
  }

  void Cancel() {
    if (!IsDone()) {
      context_.TryCancel();
    }
  }

  void OnWriteDone(bool ok) override {
    {
      std::lock_guard<std::mutex> lock(command_mtx_);
      write_done_ = true;
    }
    command_cv_.notify_all();
  }

  void OnReadDone(bool ok) override {
    {
      std::lock_guard<std::mutex> lock(command_mtx_);
      read_done_ = true;
    }
    command_cv_.notify_all();
  }

  void OnDone(const grpc::Status& s) override {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      status_ = s;
      done_ = true;
    }
    cv_.notify_all();
  }

 private:
  std::shared_ptr<RobotImpl<T>> robot_;
  grpc::ClientContext context_;

  api::RobotCommandResponse res_;
  grpc::Status status_;
  std::atomic<bool> done_;
  std::mutex mtx_;
  std::condition_variable cv_;

  bool first_command_;
  int priority_;

  std::atomic<bool> write_done_, read_done_;
  std::mutex command_mtx_;
  std::condition_variable command_cv_;
};

class SerialStreamImpl : public grpc::ClientBidiReactor<api::OpenSerialStreamRequest, api::OpenSerialStreamResponse> {
 public:
  SerialStreamImpl(api::SerialService::Stub* stub, const std::string& device_path, int baudrate, int bytesize,
                   char parity, int stopbits, int timeout_ms = 1000)
      : device_path_(device_path),
        baudrate_(baudrate),
        bytesize_(bytesize),
        parity_(parity),
        stopbits_(stopbits),
        timeout_ms_(timeout_ms) {
    stub->async()->OpenSerialStream(&context_, this);
    StartCall();
    Read(&res_);
  }

  ~SerialStreamImpl() override {
    Cancel();
    Wait();
  }

  bool IsDone() const { return done_.load(); }

  bool IsOpened() const { return is_opened_.load(); }

  bool IsCancelled() const { return IsDone(); }

  void Wait() {
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock, [this] { return done_.load(); });
  }

  bool WaitFor(int timeout_ms) {
    std::unique_lock<std::mutex> lock(mtx_);
    return cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] { return done_.load(); });
  }

  void SetReadCallback(const std::function<void(const std::string&)>& read_cb) {
    std::unique_lock<std::mutex> lock(callback_mtx_);
    read_cb_ = read_cb;
  }

  bool Connect(bool verbose) {
    if (IsOpened()) {
      if (verbose) {
        std::cerr << "Already opened" << std::endl;
      }
      return false;
    }

    if (IsDone()) {
      if (verbose) {
        std::cerr << "Stream is already disconnected" << std::endl;
      }
      return false;
    }

    api::OpenSerialStreamRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    auto& connect = *req.mutable_connect();
    connect.set_device_path(device_path_);
    connect.set_baudrate(baudrate_);
    connect.mutable_bytesize()->set_value(bytesize_);
    connect.mutable_parity()->set_value(parity_);
    connect.mutable_stopbits()->set_value(stopbits_);

    std::unique_lock<std::mutex> lock(connect_mtx_);
    connect_result_ = std::nullopt;

    Write(&req);

    if (!connect_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms_),
                              [this] { return connect_result_.has_value(); })) {
      Cancel();
      if (verbose) {
        std::cerr << "Timeout" << std::endl;
      }
      return false;
    }
    if (connect_result_->has_response_header()) {
      auto& response_header = connect_result_->response_header();
      if (response_header.has_error()) {
        if (response_header.error().code() != api::CommonError::CODE_OK) {
          std::stringstream ss;
          ss << "Connect Call Error: " << response_header.error().code() << ", " << response_header.error().message();
          if (verbose) {
            std::cerr << ss.str() << std::endl;
          }
          return false;
        }
      }
    }
    if (!connect_result_->has_connect_result()) {
      if (verbose) {
        std::cerr << "Response has no connect result" << std::endl;
      }
      return false;
    }
    if (!connect_result_->connect_result().success()) {
      if (verbose) {
        std::cerr << "Connect result: Fail (" << connect_result_->connect_result().message() << ")" << std::endl;
      }
      return false;
    }
    is_opened_ = connect_result_->connect_result().success();
    return is_opened_;
  }

  void Disconnect() {
    Cancel();
    Wait();
  }

  bool WriteData(const std::string& data) {
    if (!IsOpened()) {
      return false;
    }

    if (IsDone()) {
      return false;
    }

    api::OpenSerialStreamRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    auto& write = *req.mutable_write();
    write.set_data(data);

    std::unique_lock<std::mutex> lock(write_mtx_);
    write_result_ = std::nullopt;

    Write(&req);

    if (!write_cv_.wait_for(lock, std::chrono::seconds(1), [this] { return write_result_.has_value(); })) {
      Cancel();
      throw std::runtime_error("Timeout");
    }
    if (write_result_->has_response_header()) {
      auto& response_header = write_result_->response_header();
      if (response_header.has_error()) {
        if (response_header.error().code() != api::CommonError::CODE_OK) {
          std::stringstream ss;
          ss << "gRPC Error: " << response_header.error().code() << ", " << response_header.error().message();
          throw std::runtime_error(ss.str());
        }
      }
    }
    if (!write_result_->has_write_result()) {
      std::cout << "********************** " << data << std::endl;
      throw std::runtime_error("Response has no write result");
    }
    return write_result_->write_result().success();
  }

  bool WriteData(const char* data) { return WriteData(std::string(data)); }

  bool WriteData(const char* data, int n) { return WriteData(std::string(data, n)); }

  bool WriteByteData(char ch) { return WriteData(&ch, 1); }

  void Write(api::OpenSerialStreamRequest* req) {
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock, [=] { return done_ || write_done_; });
    if (done_) {
      return;
    }
    write_done_ = false;
    StartWrite(req);
  }

  void Read(api::OpenSerialStreamResponse* res) {
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock, [=] { return done_ || read_done_; });
    if (done_) {
      return;
    }
    read_done_ = false;
    StartRead(res);
  }

  void Cancel() {
    if (!IsDone()) {
      context_.TryCancel();
    }
  }

  void OnWriteDone(bool ok) override {
    {
      std::unique_lock<std::mutex> lock(mtx_);
      write_done_ = true;
    }
    cv_.notify_all();
  }

  void OnReadDone(bool ok) override {
    api::OpenSerialStreamResponse res;
    {
      std::unique_lock<std::mutex> lock(mtx_);
      res = res_;
      read_done_ = true;
    }
    cv_.notify_all();

    if (ok) {
      if (res.has_connect_result()) {
        {
          std::unique_lock<std::mutex> lock(connect_mtx_);
          connect_result_ = res;
        }
        connect_cv_.notify_all();
      } else if (res.has_write_result()) {
        {
          std::unique_lock<std::mutex> lock(write_mtx_);
          write_result_ = res;
        }
        write_cv_.notify_all();
      } else if (res.has_read_data()) {
        std::unique_lock<std::mutex> lock(callback_mtx_);
        if (read_cb_) {
          read_cb_(res.read_data());
        }
      }

      Read(&res_);

    } else {
      //
    }
  }

  void OnDone(const grpc::Status& s) override {
    {
      std::unique_lock<std::mutex> lock(mtx_);
      status_ = s;
      done_ = true;
    }
    cv_.notify_all();
  }

 private:
  grpc::ClientContext context_;
  std::atomic<bool> is_opened_{};

  std::string device_path_;
  int baudrate_;
  int bytesize_;
  char parity_;
  int stopbits_;
  int timeout_ms_;

  api::OpenSerialStreamResponse res_;
  grpc::Status status_;

  std::mutex mtx_;
  std::condition_variable cv_;
  bool write_done_{true}, read_done_{true};
  std::atomic<bool> done_{false};

  std::mutex callback_mtx_;
  std::function<void(const std::string&)> read_cb_;

  std::mutex connect_mtx_;
  std::condition_variable connect_cv_;
  std::optional<api::OpenSerialStreamResponse> connect_result_;

  std::mutex write_mtx_;
  std::condition_variable write_cv_;
  std::optional<api::OpenSerialStreamResponse> write_result_;
};

template <typename T>
class RobotImpl : public std::enable_shared_from_this<RobotImpl<T>> {
 public:
  explicit RobotImpl(std::string address)
      : address_(std::move(address)), connected_(false), time_sync_established_(false), time_sync_estimate_(0) {}

  ~RobotImpl() {
    StopStateUpdate();
    StopLogStream();
    Disconnect();
  }

  std::string GetAddress() { return address_; }

  bool Connect(int max_retries = 5, int timeout_ms = 1000, std::function<bool()> signal_check = nullptr) {
    if (connected_) {
      return true;
    }

    using namespace std::chrono;

    grpc::ChannelArguments args;
    args.SetInt(GRPC_ARG_MAX_RECONNECT_BACKOFF_MS, timeout_ms);
    channel_ = grpc::CreateCustomChannel(address_, grpc::InsecureChannelCredentials(), args);
    ping_service_ = api::PingService::NewStub(channel_);
    power_service_ = api::PowerService::NewStub(channel_);
    joint_operation_service_ = api::JointOperationService::NewStub(channel_);
    control_manager_service_ = api::ControlManagerService::NewStub(channel_);
    robot_info_service_ = api::RobotInfoService::NewStub(channel_);
    robot_command_service_ = api::RobotCommandService::NewStub(channel_);
    robot_state_service_ = api::RobotStateService::NewStub(channel_);
    log_service_ = api::LogService::NewStub(channel_);
    parameter_service_ = api::ParameterService::NewStub(channel_);
    led_service_ = api::LEDService::NewStub(channel_);
    system_service_ = api::SystemService::NewStub(channel_);
    serial_service_ = api::SerialService::NewStub(channel_);
    tool_flange_service_ = api::ToolFlangeService::NewStub(channel_);
    file_service_ = api::FileService::NewStub(channel_);

    int retries = 0;
    while (retries < max_retries) {
      grpc_connectivity_state state = channel_->GetState(true);
      if (state == GRPC_CHANNEL_READY) {
        connected_ = true;
        return true;
      }
      if (channel_->WaitForConnected(system_clock::now() + milliseconds(timeout_ms))) {
        connected_ = true;
        return true;
      }
      if (signal_check && !signal_check()) {
        return false;
      }
      retries++;
    }
    return false;
  }

  void Disconnect() {
    if (!connected_) {
      return;
    }

    connected_ = false;
  }

  [[nodiscard]] bool IsConnected() const {
    if (!connected_) {
      return false;
    }

    grpc_connectivity_state state = channel_->GetState(false);
    return state == GRPC_CHANNEL_READY || state == GRPC_CHANNEL_IDLE;
  }

  api::RobotInfo GetRobotInfo() const {
    api::GetRobotInfoRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    api::GetRobotInfoResponse res;
    grpc::ClientContext context;
    grpc::Status status = robot_info_service_->GetRobotInfo(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    return res.robot_info();
  }

  double GetTimeScale() const {
    api::GetTimeScaleRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    api::GetTimeScaleResponse res;
    grpc::ClientContext context;
    grpc::Status status = control_manager_service_->GetTimeScale(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    return res.time_scale();
  }

  double SetTimeScale(double time_scale) {
    api::SetTimeScaleRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_time_scale(time_scale);

    api::SetTimeScaleResponse res;
    grpc::ClientContext context;
    grpc::Status status = control_manager_service_->SetTimeScale(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    return res.current_time_scale();
  }

  bool PowerOn(const std::string& dev_name) const {
    api::PowerCommandRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_name(dev_name);
    req.set_command(api::PowerCommandRequest::COMMAND_POWER_ON);

    api::PowerCommandResponse res;
    grpc::ClientContext context;
    grpc::Status status = power_service_->PowerCommand(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    return res.status() == api::PowerCommandResponse::STATUS_SUCCESS;
  }

  bool PowerOff(const std::string& dev_name) const {
    api::PowerCommandRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_name(dev_name);
    req.set_command(api::PowerCommandRequest::COMMAND_POWER_OFF);

    api::PowerCommandResponse res;
    grpc::ClientContext context;
    grpc::Status status = power_service_->PowerCommand(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    return res.status() == api::PowerCommandResponse::STATUS_SUCCESS;
  }

  bool IsPowerOn(const std::string& dev_name) const {
    const auto& info = GetRobotInfo();
    const auto& state = GetState();

    if (info.power_infos_size() != state.power_states.size()) {
      throw std::runtime_error("size of power infos is not match to state");
    }
    size_t n_powers = info.power_infos_size();

    bool flag = false;
    std::regex re(dev_name);
    for (int i = 0; i < n_powers; i++) {
      if (regex_match(info.power_infos(i).name(), re)) {
        flag = true;
        if (state.power_states[i].state != PowerState::State::kPowerOn) {
          return false;
        }
      }
    }
    if (!flag) {
      throw std::runtime_error("No matched dev name:" + dev_name);
    }
    return true;
  }

  bool ServoOn(const std::string& dev_name) const {
    api::JointCommandRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_name(dev_name);
    req.set_command(api::JointCommandRequest::COMMAND_SERVO_ON);

    api::JointCommandResponse res;
    grpc::ClientContext context;
    grpc::Status status = power_service_->JointCommand(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    return res.status() == api::JointCommandResponse::STATUS_SUCCESS;
  }

  bool IsServoOn(const std::string& dev_name) const {
    const auto& info = GetRobotInfo();
    const auto& state = GetState();

    if (info.joint_infos_size() != state.joint_states.size()) {
      throw std::runtime_error("size of joint infos is not match to state");
    }
    size_t n_joints = info.joint_infos_size();

    std::regex re(dev_name);
    for (int i = 0; i < n_joints; i++) {
      if (regex_match(info.joint_infos(i).name(), re)) {
        if (!state.joint_states[i].is_ready ||
            TimespecToDouble(state.joint_states[i].time_since_last_update) > 0.1 /* 100ms */) {
          return false;
        }
      }
    }
    return true;
  }

  bool ServoOff(const std::string& dev_name) const {
    api::ServoOffRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_name(dev_name);

    api::ServoOffResponse res;
    grpc::ClientContext context;
    grpc::Status status = joint_operation_service_->ServoOff(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    return res.status() == api::ServoOffResponse::STATUS_SUCCESS;
  }

  bool SetPositionGain(const std::string& dev_name, std::optional<uint16_t> p_gain, std::optional<uint16_t> i_gain,
                       std::optional<uint16_t> d_gain) const {
    api::SetPositionPIDGainRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_name(dev_name);

    if (p_gain.has_value()) {
      req.mutable_p_gain()->set_value(static_cast<uint32_t>(p_gain.value()));
    }
    if (i_gain.has_value()) {
      req.mutable_i_gain()->set_value(static_cast<uint32_t>(i_gain.value()));
    }
    if (d_gain.has_value()) {
      req.mutable_d_gain()->set_value(static_cast<uint32_t>(d_gain.value()));
    }

    api::SetPositionPIDGainResponse res;
    grpc::ClientContext context;
    grpc::Status status = joint_operation_service_->SetPositionPIDGain(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    return res.status() == api::SetPositionPIDGainResponse::STATUS_SUCCESS;
  }

  std::vector<rb::PIDGain> GetTorsoPositionPIDGains() const {
    api::GetPositionPIDGainRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    req.set_target_component(api::GetPositionPIDGainRequest::TORSO);
    api::GetPositionPIDGainResponse res;
    grpc::ClientContext context;
    grpc::Status status = joint_operation_service_->GetPositionPIDGain(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    auto res_gains = res.position_gain();
    if (res_gains.empty()) {
      throw std::runtime_error("Failed to retrieve torso PID gains");
    }

    std::vector<rb::PIDGain> result;
    for (const auto& gain : res_gains) {
      result.push_back(rb::PIDGain{static_cast<uint16_t>(gain.p_gain()), static_cast<uint16_t>(gain.i_gain()),
                                   static_cast<uint16_t>(gain.d_gain())});
    }

    return result;
  }

  std::vector<rb::PIDGain> GetRightArmPositionPIDGains() const {
    api::GetPositionPIDGainRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    req.set_target_component(api::GetPositionPIDGainRequest::RIGHT_ARM);
    api::GetPositionPIDGainResponse res;
    grpc::ClientContext context;
    grpc::Status status = joint_operation_service_->GetPositionPIDGain(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }
    auto res_gains = res.position_gain();
    if (res_gains.empty()) {
      throw std::runtime_error("Failed to retrieve right arm PID gains");
    }

    std::vector<rb::PIDGain> result;
    for (const auto& gain : res_gains) {
      result.push_back(rb::PIDGain{static_cast<uint16_t>(gain.p_gain()), static_cast<uint16_t>(gain.i_gain()),
                                   static_cast<uint16_t>(gain.d_gain())});
    }

    return result;
  }

  std::vector<rb::PIDGain> GetLeftArmPositionPIDGains() const {
    api::GetPositionPIDGainRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    req.set_target_component(api::GetPositionPIDGainRequest::LEFT_ARM);
    api::GetPositionPIDGainResponse res;
    grpc::ClientContext context;
    grpc::Status status = joint_operation_service_->GetPositionPIDGain(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }
    auto res_gains = res.position_gain();
    if (res_gains.empty()) {
      throw std::runtime_error("Failed to retrieve right arm PID gains");
    }

    std::vector<rb::PIDGain> result;
    for (const auto& gain : res_gains) {
      result.push_back(rb::PIDGain{static_cast<uint16_t>(gain.p_gain()), static_cast<uint16_t>(gain.i_gain()),
                                   static_cast<uint16_t>(gain.d_gain())});
    }
    return result;
  }

  std::vector<rb::PIDGain> GetHeadPositionPIDGains() const {
    api::GetPositionPIDGainRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    req.set_target_component(api::GetPositionPIDGainRequest::HEAD);
    api::GetPositionPIDGainResponse res;
    grpc::ClientContext context;
    grpc::Status status = joint_operation_service_->GetPositionPIDGain(&context, req, &res);

    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }
    auto res_gains = res.position_gain();
    if (res_gains.empty()) {
      throw std::runtime_error("Failed to retrieve head PID gains");
    }

    std::vector<rb::PIDGain> result;
    for (const auto& gain : res_gains) {
      result.push_back(rb::PIDGain{static_cast<uint16_t>(gain.p_gain()), static_cast<uint16_t>(gain.i_gain()),
                                   static_cast<uint16_t>(gain.d_gain())});
    }
    return result;
  }

  rb::PIDGain GetPositionPIDGain(const std::string& dev_name) const {
    api::GetPositionPIDGainRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    req.set_dev_name(dev_name);
    api::GetPositionPIDGainResponse res;
    grpc::ClientContext context;
    grpc::Status status = joint_operation_service_->GetPositionPIDGain(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }
    auto res_gains = res.position_gain();
    auto result =
        rb::PIDGain{static_cast<uint16_t>(res_gains[0].p_gain()), static_cast<uint16_t>(res_gains[0].i_gain()),
                    static_cast<uint16_t>(res_gains[0].d_gain())};
    return result;
  }

  bool BreakEngage(const std::string& dev_name) const {
    api::JointCommandRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_name(dev_name);
    req.set_command(api::JointCommandRequest::COMMAND_BRAKE_ENGAGE);

    api::JointCommandResponse res;
    grpc::ClientContext context;
    grpc::Status status = power_service_->JointCommand(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    return res.status() == api::JointCommandResponse::STATUS_SUCCESS;
  }

  bool BreakRelease(const std::string& dev_name) const {
    api::JointCommandRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_name(dev_name);
    req.set_command(api::JointCommandRequest::COMMAND_BRAKE_RELEASE);

    api::JointCommandResponse res;
    grpc::ClientContext context;
    grpc::Status status = power_service_->JointCommand(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    return res.status() == api::JointCommandResponse::STATUS_SUCCESS;
  }

  bool HomeOffsetReset(const std::string& dev_name) const {
    api::JointCommandRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_name(dev_name);
    req.set_command(api::JointCommandRequest::COMMAND_HOME_OFFSET_RST);

    api::JointCommandResponse res;
    grpc::ClientContext context;
    grpc::Status status = power_service_->JointCommand(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    return res.status() == api::JointCommandResponse::STATUS_SUCCESS;
  }

  bool SetPresetPosition(const std::string& joint_name) const {
    api::SetPresetPositionRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_name(joint_name);

    api::SetPresetPositionResponse res;
    grpc::ClientContext context;
    grpc::Status status = joint_operation_service_->SetPresetPosition(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }
    if (res.has_response_header()) {
      if (res.response_header().has_error()) {
        if (res.response_header().error().code() == api::CommonError::CODE_OK) {
          return true;
        }
      }
    }
    return false;
  }

  bool EnableControlManager(bool unlimited_mode_enabled) const {
    api::ControlManagerCommandRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_command(api::ControlManagerCommandRequest::COMMAND_ENABLE);
    req.mutable_unlimited_mode_enabled()->set_value(unlimited_mode_enabled);

    api::ControlManagerCommandResponse res;
    grpc::ClientContext context;
    grpc::Status status = control_manager_service_->ControlManagerCommand(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    return res.control_manager_state().state() == api::ControlManagerState::CONTROL_MANAGER_STATE_ENABLED;
  }

  bool DisableControlManager() const {
    api::ControlManagerCommandRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_command(api::ControlManagerCommandRequest::COMMAND_DISABLE);

    api::ControlManagerCommandResponse res;
    grpc::ClientContext context;
    grpc::Status status = control_manager_service_->ControlManagerCommand(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    return res.control_manager_state().state() == api::ControlManagerState::CONTROL_MANAGER_STATE_IDLE;
  }

  bool ResetFaultControlManager() const {
    api::ControlManagerCommandRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_command(api::ControlManagerCommandRequest::COMMAND_RESET_FAULT);

    api::ControlManagerCommandResponse res;
    grpc::ClientContext context;
    grpc::Status status = control_manager_service_->ControlManagerCommand(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    return res.control_manager_state().state() == api::ControlManagerState::CONTROL_MANAGER_STATE_IDLE;
  }

  bool CancelControl() const {
    api::CancelControlRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    api::CancelControlResponse res;
    grpc::ClientContext context;
    grpc::Status status = control_manager_service_->CancelControl(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }
    if (res.has_response_header()) {
      if (res.response_header().has_error()) {
        if (res.response_header().error().code() == api::CommonError::CODE_OK) {
          return true;
        }
      }
    }
    return false;
  }

  bool SetToolFlangeOutputVoltage(const std::string& name, int voltage) const {
    api::ToolFlangePowerCommandRequest::Command cmd;
    if (voltage == 0) {
      cmd = api::ToolFlangePowerCommandRequest::COMMAND_POWER_OFF;
    } else if (voltage == 12) {
      cmd = api::ToolFlangePowerCommandRequest::COMMAND_POWER_12V;
    } else if (voltage == 24) {
      cmd = api::ToolFlangePowerCommandRequest::COMMAND_POWER_24V;
    } else {
      return false;
    }

    api::ToolFlangePowerCommandRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_name(name);
    req.set_command(cmd);

    api::ToolFlangePowerCommandResponse res;
    grpc::ClientContext context;
    grpc::Status status = power_service_->ToolFlangePowerCommand(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }
    if (res.has_response_header()) {
      if (res.response_header().has_error()) {
        if (res.response_header().error().code() == api::CommonError::CODE_OK) {
          return true;
        }
      }
    }

    return false;
  }

  bool SetToolFlangeDigitalOutput(const std::string& name, unsigned int channel, bool state) const {
    api::SetToolFlangeDigitalOutputRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_name(name);
    auto& single = *req.mutable_single();
    single.set_channel(channel);
    single.set_state(state);

    api::SetToolFlangeDigitalOutputResponse res;
    grpc::ClientContext context;
    grpc::Status status = tool_flange_service_->SetDigitalOutput(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }
    if (res.has_response_header()) {
      if (res.response_header().has_error()) {
        if (res.response_header().error().code() == api::CommonError::CODE_OK) {
          return true;
        }
      }
    }

    return false;
  }

  bool SetToolFlangeDigitalOutputDual(const std::string& name, bool state_0, bool state_1) const {
    api::SetToolFlangeDigitalOutputRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_name(name);
    auto& dual = *req.mutable_dual();
    dual.set_state_0(state_0);
    dual.set_state_1(state_1);

    api::SetToolFlangeDigitalOutputResponse res;
    grpc::ClientContext context;
    grpc::Status status = tool_flange_service_->SetDigitalOutput(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }
    if (res.has_response_header()) {
      if (res.response_header().has_error()) {
        if (res.response_header().error().code() == api::CommonError::CODE_OK) {
          return true;
        }
      }
    }

    return false;
  }

  void StartStateUpdate(const std::function<void(const RobotState<T>&, const ControlManagerState&)>& cb, double rate) {
    if (state_reader_) {
      return;
    }

    state_reader_ = std::make_unique<StateReader>(this->shared_from_this(), robot_state_service_.get(), cb, rate);
  }

  void StopStateUpdate() {
    if (!state_reader_) {
      return;
    }

    state_reader_.reset();
  }

  void StartLogStream(const std::function<void(const std::vector<Log>&)>& cb, double rate) {
    if (log_reader_) {
      return;
    }

    log_reader_ = std::make_unique<LogReader>(this->shared_from_this(), log_service_.get(), cb, rate);
  }

  void StopLogStream() {
    if (!log_reader_) {
      return;
    }

    log_reader_.reset();
  }

  RobotState<T> GetState() const {
    api::GetRobotStateRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    api::GetRobotStateResponse res;
    grpc::ClientContext context;
    grpc::Status status = robot_state_service_->GetRobotState(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    return ProtoToRobotState(res.robot_state());
  }

  std::vector<Log> GetLastLog(unsigned int count) const {
    api::GetLastLogRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_log_count(static_cast<int32_t>(count));

    api::GetLastLogResponse res;
    grpc::ClientContext context;
    grpc::Status status = log_service_->GetLastLog(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    std::vector<Log> logs;
    for (const auto& log : res.logs()) {
      logs.push_back(ProtoToLog(log));
    }

    return logs;
  }

  std::vector<std::string> GetFaultLogList() const {
    api::GetFaultLogListRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    api::GetFaultLogListResponse res;
    grpc::ClientContext context;
    grpc::Status status = log_service_->GetFaultLogList(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.has_response_header()) {
      if (res.response_header().has_error()) {
        if (res.response_header().error().code() != api::CommonError::CODE_OK) {
          throw std::runtime_error("GetFaultLogList failed: " + res.response_header().error().message());
        }
      }
    }

    std::vector<std::string> fault_log_list;
    for (const auto& log : res.fault_log_list()) {
      fault_log_list.push_back(log);
    }

    return fault_log_list;
  }

  ControlManagerState GetControlManagerState() const {
    api::GetControlManagerStateRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    api::GetControlManagerStateResponse res;
    grpc::ClientContext context;
    grpc::Status status = robot_state_service_->GetControlManagerState(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    return api::ProtoToControlManagerState(res.control_manager_state());
  }

  std::unique_ptr<RobotCommandHandler<T>> SendCommand(const RobotCommandBuilder& builder, int priority = 1) {
    api::RobotCommandRequest req;
    req.set_allocated_robot_command(static_cast<api::RobotCommand::Request*>(builder.Build()));
    req.set_priority(priority);
    return std::unique_ptr<RobotCommandHandler<T>>(
        new RobotCommandHandler<T>(std::make_unique<RobotCommandHandlerImpl<T>>(this->shared_from_this(), req)));
  }

  std::unique_ptr<RobotCommandStreamHandler<T>> CreateCommandStream(int priority = 1) {
    return std::unique_ptr<RobotCommandStreamHandler<T>>(
        new RobotCommandStreamHandler<T>(std::make_unique<RobotCommandStreamHandlerImpl<T>>(
            this->shared_from_this(), robot_command_service_.get(), priority)));
  }

  bool Control(std::function<ControlInput<T>(const ControlState<T>&)> control, int port, int priority) {
    std::shared_ptr<UdpServer> server_;
    try {
      server_ = std::make_shared<UdpServer>(port);
    } catch (std::exception& e) {
      std::cerr << e.what() << std::endl;
      return false;
    }

    api::RobotCommandRequest req;
    req.set_allocated_robot_command(
        static_cast<api::RobotCommand::Request*>(RobotCommandBuilder()
                                                     .SetCommand(WholeBodyCommandBuilder().SetCommand(
                                                         RealTimeControlCommandBuilder().SetPort(server_->GetPort())))
                                                     .Build()));
    req.set_priority(priority);

    auto handler = std::unique_ptr<RobotCommandHandler<T>>(
        new RobotCommandHandler<T>(std::make_unique<RobotCommandHandlerImpl<T>>(this->shared_from_this(), req)));

    bool finished{false};

    constexpr size_t kMaxPacketLength = 4096;
    std::array<unsigned char, kMaxPacketLength> send_packet{};
    int recv_pkt_size_{0};
    std::array<unsigned char, kMaxPacketLength> recv_packet{};

    struct sockaddr_storage client_addr{};

    while (!handler->IsDone()) {
      {
        int len =
            server_->RecvFrom(recv_packet.data() + recv_pkt_size_, kMaxPacketLength - recv_pkt_size_, client_addr);
        if (len > 0) {
          recv_pkt_size_ += len;
        }
      }

      int tmp = 0;
      while (tmp < recv_pkt_size_) {
        auto rv = ValidateRTProtocol(recv_packet.data() + tmp, recv_pkt_size_ - tmp);
        if (rv.first) {  // Receive command
          auto packet_pointer = recv_packet.data() + tmp;
          tmp += rv.second;

          int dof = (int)GetDoFRobotStateRTProtocol(packet_pointer);
          if (dof == T::kRobotDOF) {
            ControlState<T> state;

            ParseRobotStateRTProtocol(packet_pointer, nullptr, &state.t, state.is_ready.data(), state.position.data(),
                                      state.velocity.data(), state.current.data(), state.torque.data());

            if (!finished) {
              ControlInput<T> input = control(state);

              int len =
                  BuildRobotCommandRTPacket(send_packet.data(), T::kRobotDOF, input.mode.data(), input.target.data(),
                                            input.feedback_gain.data(), input.feedforward_torque.data(), input.finish);
              server_->SendTo(send_packet.data(), len, client_addr);

              finished = input.finish;
            }
            break;
          }
        } else {
          if (rv.second == 0) {
            break;
          } else {
            tmp += rv.second;
            // Invalid packet
          }
        }
      }
      if (tmp > 0) {
        std::rotate(recv_packet.begin(), recv_packet.begin() + tmp, recv_packet.end());
        recv_pkt_size_ -= tmp;
      }

      std::this_thread::sleep_for(10us);
    }

    return finished;
  }

  bool ResetOdometry(double angle, const Eigen::Vector<double, 2>& position) {
    api::SE2Pose pose;
    pose.set_angle(angle);
    pose.mutable_position()->set_x(position(0));
    pose.mutable_position()->set_y(position(1));

    api::ResetOdometryRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    *req.mutable_initial_pose() = pose;

    api::ResetOdometryResponse res;
    grpc::ClientContext context;
    grpc::Status status = robot_state_service_->ResetOdometry(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.has_response_header()) {
      if (res.response_header().has_error()) {
        if (res.response_header().error().code() == api::CommonError::CODE_OK) {
          return true;
        }
      }
    }
    return false;
  }

  std::vector<std::pair<std::string, int>> GetParameterList() {
    api::GetParameterListRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    api::GetParameterListResponse res;
    grpc::ClientContext context;
    grpc::Status status = parameter_service_->GetParameterList(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.has_response_header()) {
      if (res.response_header().has_error()) {
        if (res.response_header().error().code() != api::CommonError::CODE_OK) {
          throw std::runtime_error("GetParameter failed: " + res.response_header().error().message());
        }
      }
    }

    std::vector<std::pair<std::string, int>> rv;
    for (const auto& param : res.parameters()) {
      rv.emplace_back(param.name(), param.type());
    }

    return rv;
  }

  bool SetParameter(const std::string& name, const std::string& value, bool write_db) {
    std::stringstream ss;
    ss << std::boolalpha << "{\"value\":" << value << ",\"write_db\":" << write_db << "}";

    api::SetParameterRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_name(name);
    req.set_parameter(ss.str());

    api::SetParameterResponse res;
    grpc::ClientContext context;
    grpc::Status status = parameter_service_->SetParameter(&context, req, &res);
    if (!status.ok()) {
      std::stringstream error_msg_ss;
      error_msg_ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
                   << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        error_msg_ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(error_msg_ss.str());
    }

    if (res.response_header().has_error()) {
      return res.response_header().error().code() == api::CommonError::CODE_OK;
    }

    return true;
  }

  std::string GetParameter(const std::string& name) {
    api::GetParameterRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_name(name);

    api::GetParameterResponse res;
    grpc::ClientContext context;
    grpc::Status status = parameter_service_->GetParameter(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.has_response_header()) {
      if (res.response_header().has_error()) {
        if (res.response_header().error().code() != api::CommonError::CODE_OK) {
          throw std::runtime_error("GetParameter failed: " + res.response_header().error().message());
        }
      }
    }

    auto j = nlohmann::json::parse(res.parameter());
    if (j.contains("value")) {
      return j.at("value").dump();
    }

    return "";
  }

  void ResetAllParameters() {
    api::ResetAllParametersRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    api::ResetAllParametersResponse res;
    grpc::ClientContext context;
    grpc::Status status = parameter_service_->ResetAllParameters(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }
  }

  bool ResetParameter(const std::string& name) {
    api::ResetParameterRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_name(name);

    api::ResetParameterResponse res;
    grpc::ClientContext context;
    grpc::Status status = parameter_service_->ResetParameter(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.response_header().has_error()) {
      return res.response_header().error().code() == api::CommonError::CODE_OK;
    }

    return true;
  }

  void ResetAllParametersToDefault() {
    api::ResetAllParametersToDefaultRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    api::ResetAllParametersToDefaultResponse res;
    grpc::ClientContext context;
    grpc::Status status = parameter_service_->ResetAllParametersToDefault(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }
  }

  bool FactoryResetParameter(const std::string& name) {
    api::FactoryResetParameterRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_name(name);

    api::FactoryResetParameterResponse res;
    grpc::ClientContext context;
    grpc::Status status = parameter_service_->FactoryResetParameter(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.response_header().has_error()) {
      return res.response_header().error().code() == api::CommonError::CODE_OK;
    }

    return true;
  }

  void FactoryResetAllParameters() {
    api::FactoryResetAllParametersRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    api::FactoryResetAllParametersResponse res;
    grpc::ClientContext context;
    grpc::Status status = parameter_service_->FactoryResetAllParameters(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }
  }

  bool ResetParameterToDefault(const std::string& name) {
    api::ResetParameterToDefaultRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_name(name);

    api::ResetParameterToDefaultResponse res;
    grpc::ClientContext context;
    grpc::Status status = parameter_service_->ResetParameterToDefault(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.response_header().has_error()) {
      return res.response_header().error().code() == api::CommonError::CODE_OK;
    }

    return true;
  }

  std::string GetRobotModel() const {
    api::GetRobotModelRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    api::GetRobotModelResponse res;
    grpc::ClientContext context;
    grpc::Status status = robot_info_service_->GetRobotModel(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    return res.model();
  }

  bool ImportRobotModel(const std::string& name, const std::string& model) {
    api::ImportRobotModelRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_name(name);
    req.set_model(model);

    api::ImportRobotModelResponse res;
    grpc::ClientContext context;
    grpc::Status status = robot_info_service_->ImportRobotModel(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.has_response_header()) {
      if (res.response_header().has_error()) {
        if (res.response_header().error().code() != api::CommonError::CODE_OK) {
          return false;
        }
      }
    }
    return true;
  }

  bool SyncTime() {
    api::PingRequest req;
    api::PingResponse res;
    grpc::ClientContext context;

    int64_t client_req_utc_time;
    int64_t client_req_time, client_res_time;
    int64_t robot_recv_req_time, robot_send_res_time;

    client_req_utc_time = TimespecInNs(TimepointToTimespec(std::chrono::system_clock::now()));
    client_req_time = TimespecInNs(GetCurrentTime());
    InitializeRequestHeader(req.mutable_request_header());
    grpc::Status status = ping_service_->Ping(&context, req, &res);
    client_res_time = TimespecInNs(GetCurrentTime());
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.has_response_header()) {
      const auto& res_header = res.response_header();

      if (res_header.has_request_received_timestamp()) {
        robot_recv_req_time = TimestampInNs(res_header.request_received_timestamp());
      } else {
        return false;
      }

      if (res_header.has_response_timestamp()) {
        robot_send_res_time = TimestampInNs(res_header.response_timestamp());
      } else {
        return false;
      }

      if (client_res_time < client_req_time) {
        return false;
      }

      if (robot_send_res_time < robot_recv_req_time) {
        return false;
      }

      time_sync_established_ = true;
      time_sync_estimate_ = client_req_utc_time -
                            (robot_recv_req_time -
                             ((client_res_time - client_req_time) - (robot_send_res_time - robot_recv_req_time)) / 2);
      return true;
    }

    return false;
  }

  bool HasEstablishedTimeSync() { return time_sync_established_; }

  bool StartTimeSync(long period_sec) {
    if (time_sync_) {
      return false;
    }

    time_sync_ = std::make_unique<EventLoop>();
    time_sync_->PushCyclicTask([=] { SyncTime(); }, std::chrono::seconds(period_sec));

    return true;
  }

  bool StopTimeSync() {
    if (!time_sync_) {
      return false;
    }

    time_sync_->Pause();
    time_sync_->WaitForTasks();
    time_sync_.reset();

    return true;
  }

  std::shared_ptr<dyn::Robot<T::kRobotDOF>> GetDynamics(std::string urdf_model) {
    if (urdf_model.empty()) {
      urdf_model = GetRobotModel();
    }

    std::shared_ptr<dyn::Robot<T::kRobotDOF>> dyn_robot;
    try {
      rb::dyn::RobotConfiguration rc = dyn::LoadRobotFromURDFData(urdf_model, "base");
      dyn_robot = std::make_shared<dyn::Robot<T::kRobotDOF>>(rc);
    } catch (std::exception& e) {
      std::stringstream ss;
      ss << "Failed to load dynamics model: ";
      ss << e.what();
      throw std::runtime_error(ss.str());
    }

    return dyn_robot;
  }

  bool SetLEDColor(const rb::Color& color, double duration, double transition_time, bool blinking,
                   double blinking_freq) {
    api::SetLEDColorRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    api::Color c;
    c.set_red(color.r);
    c.set_green(color.g);
    c.set_blue(color.b);

    *req.mutable_color() = c;
    int duration_seconds = (int)duration;
    int duration_nanos = (int)((duration - duration_seconds) * 1e9);
    req.mutable_duration()->set_seconds(duration_seconds);
    req.mutable_duration()->set_nanos(duration_nanos);
    int transition_time_seconds = (int)transition_time;
    int transition_time_nanos = (int)((transition_time - transition_time_seconds) * 1e9);
    req.mutable_transition_time()->set_seconds(transition_time_seconds);
    req.mutable_transition_time()->set_nanos(transition_time_nanos);
    req.set_blinking(blinking);
    req.mutable_blinking_freq()->set_value(blinking_freq);

    api::SetLEDColorResponse res;
    grpc::ClientContext context;
    grpc::Status status = led_service_->SetLEDColor(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.response_header().has_error()) {
      return res.response_header().error().code() == api::CommonError::CODE_OK;
    }
    return true;
  }

  std::tuple<struct timespec, std::string, std::string> GetSystemTime() const {
    api::GetSystemTimeRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    api::GetSystemTimeResponse res;
    grpc::ClientContext context;
    grpc::Status status = system_service_->GetSystemTime(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    struct timespec utc_time{};

    if (res.has_utc_time()) {
      utc_time.tv_sec = res.utc_time().seconds();
      utc_time.tv_nsec = res.utc_time().nanos();
    }

    return {utc_time, res.time_zone(), res.local_time()};
  }

  bool SetSystemTime(struct timespec utc_time, std::optional<std::string> time_zone) const {
    api::SetSystemTimeRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    auto& req_utc_time = *req.mutable_utc_time();
    req_utc_time.set_seconds(utc_time.tv_sec);
    req_utc_time.set_nanos((int)utc_time.tv_nsec);
    if (time_zone.has_value()) {
      *req.mutable_time_zone() = time_zone.value();
    }

    api::SetSystemTimeResponse res;
    grpc::ClientContext context;
    grpc::Status status = system_service_->SetSystemTime(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.response_header().has_error()) {
      return res.response_header().error().code() == api::CommonError::CODE_OK;
    }
    return true;
  }

  bool SetBatteryLevel(double level) const {
    api::SetBatteryLevelRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_level(level);

    api::SetBatteryLevelResponse res;
    grpc::ClientContext context;
    grpc::Status status = system_service_->SetBatteryLevel(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.response_header().has_error()) {
      return res.response_header().error().code() == api::CommonError::CODE_OK;
    }
    return true;
  }

  bool SetBatteryConfig(double cutoff_voltage, double fully_charged_voltage,
                        const std::array<double, 4>& coefficients) const {
    api::SetBatteryConfigRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_cut_off_voltage(cutoff_voltage);
    req.set_fully_charged_voltage(fully_charged_voltage);
    req.clear_coefficients();
    for (const auto& c : coefficients) {
      req.add_coefficients(c);
    }

    api::SetBatteryConfigResponse res;
    grpc::ClientContext context;
    grpc::Status status = system_service_->SetBatteryConfig(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.response_header().has_error()) {
      return res.response_header().error().code() == api::CommonError::CODE_OK;
    }
    return true;
  }

  bool ResetBatteryConfig() const {
    api::ResetBatteryConfigRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    api::ResetBatteryConfigResponse res;
    grpc::ClientContext context;
    grpc::Status status = system_service_->ResetBatteryConfig(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.response_header().has_error()) {
      return res.response_header().error().code() == api::CommonError::CODE_OK;
    }
    return true;
  }

  bool WaitForControlReady(long timeout_ms) const {
    api::WaitForControlReadyRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    if (timeout_ms >= 0) {
      auto& t = *req.mutable_timeout();
      t.set_seconds(timeout_ms / 1000);
      t.set_nanos((timeout_ms % 1000) * 1000000);
    }

    api::WaitForControlReadyResponse res;
    grpc::ClientContext context;
    grpc::Status status = control_manager_service_->WaitForControlReady(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    return res.ready();
  }

  bool ResetNetworkSetting() const {
    api::ResetNetworkSettingRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    api::ResetNetworkSettingResponse res;
    grpc::ClientContext context;
    grpc::Status status = system_service_->ResetNetworkSetting(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.response_header().has_error()) {
      return res.response_header().error().code() == api::CommonError::CODE_OK;
    }
    return true;
  }

  std::vector<WifiNetwork> ScanWifi() const {
    api::ScanWifiRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    api::ScanWifiResponse res;
    grpc::ClientContext context;
    grpc::Status status = system_service_->ScanWifi(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.response_header().has_error()) {
      if (res.response_header().error().code() != api::CommonError::CODE_OK) {
        std::cerr << "Error: " << res.response_header().error().message() << std::endl;
        return {};
      }
    }

    std::vector<WifiNetwork> wifi_networks;

    for (const auto& n : res.networks()) {
      WifiNetwork network;
      network.ssid = n.ssid();
      network.signal_strength = n.signal_strength();
      network.secured = n.secured();
      wifi_networks.push_back(network);
    }

    return wifi_networks;
  }

  bool ConnectWifi(const std::string& ssid, const std::string& password, bool use_dhcp, const std::string& ip_address,
                   const std::string& gateway, const std::vector<std::string>& dns) const {
    api::ConnectWifiRequest req;
    InitializeRequestHeader(req.mutable_request_header());
    req.set_ssid(ssid);
    req.set_password(password);
    req.set_use_dhcp(use_dhcp);
    req.set_ip_address(ip_address);
    req.set_gateway(gateway);
    for (const auto& d : dns) {
      *req.add_dns() = d;
    }

    api::ConnectWifiResponse res;
    grpc::ClientContext context;
    grpc::Status status = system_service_->ConnectWifi(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.response_header().has_error()) {
      return res.response_header().error().code() == api::CommonError::CODE_OK;
    }
    return true;
  }

  bool DisconnectWifi() const {
    api::DisconnectWifiRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    api::DisconnectWifiResponse res;
    grpc::ClientContext context;
    grpc::Status status = system_service_->DisconnectWifi(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.response_header().has_error()) {
      return res.response_header().error().code() == api::CommonError::CODE_OK;
    }
    return true;
  }

  std::optional<WifiStatus> GetWifiStatus() const {
    api::GetWifiStatusRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    api::GetWifiStatusResponse res;
    grpc::ClientContext context;
    grpc::Status status = system_service_->GetWifiStatus(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.response_header().has_error()) {
      if (res.response_header().error().code() != api::CommonError::CODE_OK) {
        return std::nullopt;
      }
    }

    WifiStatus wifi_status;
    wifi_status.ssid = res.ssid();
    wifi_status.ip_address = res.ip_address();
    wifi_status.gateway = res.gateway();
    for (const auto& d : res.dns()) {
      wifi_status.dns.push_back(d);
    }
    wifi_status.connected = res.connected();

    return wifi_status;
  }

  std::vector<SerialDevice> GetSerialDeviceList() const {
    api::GetSerialDeviceListRequest req;
    InitializeRequestHeader(req.mutable_request_header());

    api::GetSerialDeviceListResponse res;
    grpc::ClientContext context;
    grpc::Status status = serial_service_->GetSerialDeviceList(&context, req, &res);
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    if (res.response_header().has_error()) {
      if (res.response_header().error().code() != api::CommonError::CODE_OK) {
        std::cerr << "Error: " << res.response_header().error().message() << std::endl;
        return {};
      }
    }

    std::vector<SerialDevice> serial_devices;

    for (const auto& d : res.devices()) {
      SerialDevice device;
      device.path = d.path();
      device.description = d.description();
      serial_devices.push_back(device);
    }

    return serial_devices;
  }

  std::unique_ptr<SerialStream> OpenSerialStream(const std::string& device_path, int buadrate, int bytesize,
                                                 char parity, int stopbits) const {
    return std::unique_ptr<SerialStream>(new SerialStream(
        std::make_unique<SerialStreamImpl>(serial_service_.get(), device_path, buadrate, bytesize, parity, stopbits)));
  }

  bool DownloadFile(const std::string& path, std::ostream& output) const {
    return DownloadFileToCallback(path, [&output](const char* data, size_t size) { output.write(data, size); });
  }

  bool DownloadFileToCallback(const std::string& path, std::function<void(const char*, size_t)> on_chunk) const {
    api::DownloadFileRequest req;
    req.set_file_path(path);

    grpc::ClientContext context;
    InitializeRequestHeader(req.mutable_request_header());

    std::unique_ptr<grpc::ClientReader<api::DownloadFileResponse>> reader(file_service_->DownloadFile(&context, req));

    api::DownloadFileResponse chunk;
    while (reader->Read(&chunk)) {
      if (chunk.has_response_header()) {
        if (chunk.response_header().has_error()) {
          if (chunk.response_header().error().code() != api::CommonError::CODE_OK) {
            std::cerr << "Error: " << chunk.response_header().error().message() << std::endl;
            return false;
          }
        }
      }

      if (on_chunk) {
        on_chunk(chunk.file_content().data(), chunk.file_content().size());
      }
    }

    grpc::Status status = reader->Finish();
    if (!status.ok()) {
      std::stringstream ss;
      ss << "gRPC call failed. Code: " << static_cast<int>(status.error_code()) << "("
         << gRPCStatusToString(status.error_code()) << ")";
      if (!status.error_message().empty()) {
        ss << ", Message: " << status.error_message();
      }
      throw std::runtime_error(ss.str());
    }

    return true;
  }

  class StateReader : public grpc::ClientReadReactor<api::GetRobotStateStreamResponse> {
   public:
    explicit StateReader(std::shared_ptr<RobotImpl<T>> robot, api::RobotStateService::Stub* stub,
                         const std::function<void(const RobotState<T>&, const ControlManagerState&)>& cb, double rate)
        : robot_(std::move(robot)) {
      cb_ = cb;

      api::GetRobotStateStreamRequest req;
      InitializeRequestHeader(req.mutable_request_header());
      req.set_update_rate(rate);
      stub->async()->GetRobotStateStream(&context_, &req, this);
      StartRead(&res_);
      StartCall();
    }

    ~StateReader() override {
      Cancel();
      WaitForDone();
    }

    grpc::Status WaitForDone() {
      std::unique_lock<std::mutex> l(mtx_);
      cv_.wait(l, [this] { return done_; });
      return status_;
    }

    void Cancel() { context_.TryCancel(); }

   private:
    void OnReadDone(bool ok) override {
      if (ok) {
        if (res_.has_robot_state() && cb_) {
          cb_(robot_->ProtoToRobotState(res_.robot_state()),
              api::ProtoToControlManagerState(res_.control_manager_state()));
        }
        StartRead(&res_);
      }
    }

    void OnDone(const grpc::Status& s) override {
      std::unique_lock<std::mutex> l(mtx_);
      status_ = s;
      done_ = true;
      cv_.notify_all();
    }

   private:
    std::shared_ptr<RobotImpl<T>> robot_;
    grpc::ClientContext context_;
    std::function<void(const RobotState<T>&, const ControlManagerState&)> cb_;
    api::GetRobotStateStreamResponse res_;
    std::mutex mtx_;
    std::condition_variable cv_;
    grpc::Status status_;
    bool done_ = false;
  };

  class LogReader : public grpc::ClientReadReactor<api::GetLogStreamResponse> {
   public:
    explicit LogReader(std::shared_ptr<RobotImpl<T>> robot, api::LogService::Stub* stub,
                       const std::function<void(const std::vector<Log>&)>& cb, double rate)
        : robot_(std::move(robot)) {
      cb_ = cb;

      api::GetLogStreamRequest req;
      InitializeRequestHeader(req.mutable_request_header());
      req.set_update_rate(rate);
      stub->async()->GetLogStream(&context_, &req, this);
      StartRead(&res_);
      StartCall();
    }

    ~LogReader() override {
      Cancel();
      WaitForDone();
    }

    grpc::Status WaitForDone() {
      std::unique_lock<std::mutex> l(mtx_);
      cv_.wait(l, [this] { return done_; });
      return status_;
    }

    void Cancel() { context_.TryCancel(); }

   private:
    void OnReadDone(bool ok) override {
      if (ok) {
        if (cb_) {
          std::vector<Log> logs;
          for (const auto& log : res_.logs()) {
            logs.push_back(robot_->ProtoToLog(log));
          }
          cb_(logs);
        }
        StartRead(&res_);
      }
    }

    void OnDone(const grpc::Status& s) override {
      std::unique_lock<std::mutex> l(mtx_);
      status_ = s;
      done_ = true;
      cv_.notify_all();
    }

   private:
    std::shared_ptr<RobotImpl<T>> robot_;
    grpc::ClientContext context_;
    std::function<void(const std::vector<Log>&)> cb_;
    api::GetLogStreamResponse res_;
    std::mutex mtx_;
    std::condition_variable cv_;
    grpc::Status status_;
    bool done_ = false;
  };

  rb::RobotState<T> ProtoToRobotState(const api::RobotState& msg) const {
    rb::RobotState<T> rs;

    if (msg.has_timestamp()) {
      if (time_sync_established_) {
        int64_t rs_ts = TimestampInNs(msg.timestamp());
        rs_ts += time_sync_estimate_;
        rs.timestamp.tv_sec = rs_ts / kNanosecondsInSecond;
        rs.timestamp.tv_nsec = rs_ts % kNanosecondsInSecond;
      } else {
        const auto& rs_ts = msg.timestamp();
        rs.timestamp.tv_sec = rs_ts.seconds();
        rs.timestamp.tv_nsec = rs_ts.nanos();
      }
    }

    if (msg.has_system_stat()) {
      const auto& rs_s = msg.system_stat();
      rs.system_stat.cpu_usage = rs_s.cpu_usage();
      rs.system_stat.memory_usage = rs_s.memory_usage();
      rs.system_stat.uptime = rs_s.uptime();
      rs.system_stat.program_uptime = rs_s.program_uptime();
    }

    if (msg.has_battery_state()) {
      const auto& rs_bs = msg.battery_state();
      rs.battery_state.voltage = rs_bs.voltage();
      rs.battery_state.current = rs_bs.current();
      rs.battery_state.level_percent = rs_bs.level_percent();
    }

    for (const auto& s : msg.power_states()) {
      rb::PowerState ps;
      ps.voltage = s.voltage();
      ps.state = static_cast<rb::PowerState::State>(s.state());
      rs.power_states.push_back(ps);
    }

    for (const auto& s : msg.emo_states()) {
      rb::EMOState es;
      es.state = static_cast<rb::EMOState::State>(s.state());
      rs.emo_states.push_back(es);
    }

    const auto& proto_to_tool_flange = [](rb::ToolFlangeState& tf, const api::ToolFlangeState& tf_proto) {
      if (tf_proto.has_time_since_last_update()) {
        tf.time_since_last_update = DurationToTimespec(tf_proto.time_since_last_update());
      } else {
        tf.time_since_last_update.tv_sec = 12 * 60 * 60 /* 12 hour */;
      }

      if (tf_proto.has_gyro()) {
        tf.gyro(0) = tf_proto.gyro().x();
        tf.gyro(1) = tf_proto.gyro().y();
        tf.gyro(2) = tf_proto.gyro().z();
      }

      if (tf_proto.has_acceleration()) {
        tf.acceleration(0) = tf_proto.acceleration().x();
        tf.acceleration(1) = tf_proto.acceleration().y();
        tf.acceleration(2) = tf_proto.acceleration().z();
      }

      tf.switch_A = tf_proto.switch_a();
      tf.output_voltage = tf_proto.output_voltage();
      tf.digital_input_A = tf_proto.digital_input_a();
      tf.digital_input_B = tf_proto.digital_input_b();
      tf.digital_output_A = tf_proto.digital_output_a();
      tf.digital_output_B = tf_proto.digital_output_b();
    };
    if (msg.has_tool_flange_right()) {
      proto_to_tool_flange(rs.tool_flange_right, msg.tool_flange_right());
    }
    if (msg.has_tool_flange_left()) {
      proto_to_tool_flange(rs.tool_flange_left, msg.tool_flange_left());
    }

    const auto& proto_to_ftsensor = [](rb::FTSensorData& ft, const api::FTSensorData& ft_proto) {
      if (ft_proto.has_time_since_last_update()) {
        ft.time_since_last_update = DurationToTimespec(ft_proto.time_since_last_update());
      } else {
        ft.time_since_last_update.tv_sec = 999;
      }

      if (ft_proto.has_force()) {
        ft.force(0) = ft_proto.force().x();
        ft.force(1) = ft_proto.force().y();
        ft.force(2) = ft_proto.force().z();
      }

      if (ft_proto.has_torque()) {
        ft.torque(0) = ft_proto.torque().x();
        ft.torque(1) = ft_proto.torque().y();
        ft.torque(2) = ft_proto.torque().z();
      }
    };
    if (msg.has_ft_sensor_right()) {
      proto_to_ftsensor(rs.ft_sensor_right, msg.ft_sensor_right());
    }
    if (msg.has_ft_sensor_left()) {
      proto_to_ftsensor(rs.ft_sensor_left, msg.ft_sensor_left());
    }

    if (msg.joint_states_size() != T::kRobotDOF) {
      throw std::runtime_error("The size of the joint state does not match the DOF.");
    }

    for (int i = 0; i < msg.joint_states_size(); i++) {
      const auto& s = msg.joint_states(i);
      auto& js = rs.joint_states[i];

      js.is_ready = s.is_ready();
      js.fet_state = static_cast<rb::JointState::FETState>(s.fet_state());
      js.run_state = static_cast<rb::JointState::RunState>(s.run_state());
      js.init_state = static_cast<rb::JointState::InitializationState>(s.init_state());

      js.motor_type = s.motor_type();
      js.motor_state = s.motor_state();

      if (s.has_time_since_last_update()) {
        js.time_since_last_update = DurationToTimespec(s.time_since_last_update());
      } else {
        js.time_since_last_update.tv_sec = 999;
      }
      js.power_on = s.power_on();
      js.position = s.position();
      js.velocity = s.velocity();
      js.current = s.current();
      js.torque = s.torque();
      js.target_position = s.target_position();
      js.target_velocity = s.target_velocity();
      js.target_feedback_gain = s.target_feedback_gain();
      js.target_feedforward_torque = s.target_feedforward_torque();
      js.temperature = s.temperature();
    }

    if (msg.is_ready_size() != T::kRobotDOF) {
      throw std::runtime_error("The size of 'is_ready' vector does not match the DOF.");
    }

    for (int i = 0; i < T::kRobotDOF; i++) {
      rs.is_ready[i] = msg.is_ready(i);
    }

    if (msg.position_size() != T::kRobotDOF) {
      throw std::runtime_error("The size of 'position' vector does not match the DOF.");
    }

    for (int i = 0; i < T::kRobotDOF; i++) {
      rs.position[i] = msg.position(i);
    }

    if (msg.velocity_size() != T::kRobotDOF) {
      throw std::runtime_error("The size of 'velocity' vector does not match the DOF.");
    }

    for (int i = 0; i < T::kRobotDOF; i++) {
      rs.velocity[i] = msg.velocity(i);
    }

    if (msg.current_size() != T::kRobotDOF) {
      throw std::runtime_error("The size of 'current' vector does not match the DOF.");
    }

    for (int i = 0; i < T::kRobotDOF; i++) {
      rs.current[i] = msg.current(i);
    }

    if (msg.torque_size() != T::kRobotDOF) {
      throw std::runtime_error("The size of 'torque' vector does not match the DOF.");
    }

    for (int i = 0; i < T::kRobotDOF; i++) {
      rs.torque[i] = msg.torque(i);
    }

    if (msg.target_position_size() != T::kRobotDOF) {
      throw std::runtime_error("The size of 'target_position' vector does not match the DOF.");
    }

    for (int i = 0; i < T::kRobotDOF; i++) {
      rs.target_position[i] = msg.target_position(i);
    }

    if (msg.target_velocity_size() != T::kRobotDOF) {
      throw std::runtime_error("The size of 'target_velocity' vector does not match the DOF.");
    }

    for (int i = 0; i < T::kRobotDOF; i++) {
      rs.target_velocity[i] = msg.target_velocity(i);
    }

    if (msg.target_feedback_gain_size() != T::kRobotDOF) {
      throw std::runtime_error("The size of 'target_feedback_gain' vector does not match the DOF.");
    }

    for (int i = 0; i < T::kRobotDOF; i++) {
      rs.target_feedback_gain[i] = msg.target_feedback_gain(i);
    }

    if (msg.target_feedforward_torque_size() != T::kRobotDOF) {
      throw std::runtime_error("The size of 'target_feedforward_torque' vector does not match the DOF.");
    }

    for (int i = 0; i < T::kRobotDOF; i++) {
      rs.target_feedforward_torque[i] = msg.target_feedforward_torque(i);
    }

    if (msg.has_odometry()) {
      Eigen::Vector<double, 2> pos{Eigen::Vector<double, 2>::Zero()};
      if (msg.odometry().has_position()) {
        pos = Eigen::Vector<double, 2>{msg.odometry().position().x(), msg.odometry().position().y()};
      }
      rs.odometry = math::SE2::T(msg.odometry().angle(), pos);
    }

    if (msg.has_center_of_mass()) {
      rs.center_of_mass(0) = msg.center_of_mass().x();
      rs.center_of_mass(1) = msg.center_of_mass().y();
      rs.center_of_mass(2) = msg.center_of_mass().z();
    }

    rs.collisions.clear();
    for (const auto& col : msg.collisions()) {
      dyn::CollisionResult cs;
      cs.link1 = col.link1();
      cs.link2 = col.link2();
      if (col.has_position1()) {
        cs.position1(0) = col.position1().x();
        cs.position1(1) = col.position1().y();
        cs.position1(2) = col.position1().z();
      }
      if (col.has_position2()) {
        cs.position2(0) = col.position2().x();
        cs.position2(1) = col.position2().y();
        cs.position2(2) = col.position2().z();
      }
      cs.distance = col.distance();
      rs.collisions.push_back(cs);
    }

    if (msg.temperature_size() == 0) {
      // Backward-compatability issue
      for (int i = 0; i < T::kRobotDOF; i++) {
        rs.temperature[i] = 0;
      }
    } else if (msg.temperature_size() == T::kRobotDOF) {
      for (int i = 0; i < T::kRobotDOF; i++) {
        rs.temperature[i] = msg.temperature(i);
      }
    } else {
      throw std::runtime_error("The size of 'temperature' vector does not match the DOF.");
    }

    return rs;
  }

  Log ProtoToLog(const api::Log& msg) const {
    rb::Log log;

    if (msg.has_robot_system_timestamp()) {
      const auto& rs_ts = msg.robot_system_timestamp();
      log.robot_system_timestamp.tv_sec = rs_ts.seconds();
      log.robot_system_timestamp.tv_nsec = rs_ts.nanos();
    }
    if (msg.has_timestamp()) {
      if (time_sync_established_) {
        int64_t rs_ts = TimestampInNs(msg.timestamp());
        rs_ts += time_sync_estimate_;
        log.timestamp.tv_sec = rs_ts / kNanosecondsInSecond;
        log.timestamp.tv_nsec = rs_ts % kNanosecondsInSecond;
      } else {
        const auto& rs_ts = msg.timestamp();
        log.timestamp.tv_sec = rs_ts.seconds();
        log.timestamp.tv_nsec = rs_ts.nanos();
      }
    }

    log.level = static_cast<rb::Log::Level>(msg.level());
    log.message = msg.message();

    return log;
  }

 private:
  std::string address_;
  bool connected_;

  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<api::PingService::Stub> ping_service_;
  std::unique_ptr<api::PowerService::Stub> power_service_;
  std::unique_ptr<api::JointOperationService::Stub> joint_operation_service_;
  std::unique_ptr<api::ControlManagerService::Stub> control_manager_service_;
  std::unique_ptr<api::RobotInfoService::Stub> robot_info_service_;
  std::unique_ptr<api::RobotCommandService::Stub> robot_command_service_;
  std::unique_ptr<api::RobotStateService::Stub> robot_state_service_;
  std::unique_ptr<api::LogService::Stub> log_service_;
  std::unique_ptr<api::ParameterService::Stub> parameter_service_;
  std::unique_ptr<api::LEDService::Stub> led_service_;
  std::unique_ptr<api::SystemService::Stub> system_service_;
  std::unique_ptr<api::SerialService::Stub> serial_service_;
  std::unique_ptr<api::ToolFlangeService::Stub> tool_flange_service_;
  std::unique_ptr<api::FileService::Stub> file_service_;

  std::unique_ptr<StateReader> state_reader_;
  std::unique_ptr<LogReader> log_reader_;

  bool time_sync_established_;
  int64_t time_sync_estimate_;

  std::unique_ptr<EventLoop> time_sync_;

  friend class RobotCommandHandlerImpl<T>;
};

template <typename T>
std::shared_ptr<Robot<T>> Robot<T>::Create(std::string address) {
  return std::shared_ptr<Robot<T>>(new Robot<T>(std::move(address)));
}

template <typename T>
Robot<T>::Robot(std::string address) {
  impl_ = std::make_shared<RobotImpl<T>>(std::move(address));
}

template <typename T>
Robot<T>::~Robot() = default;

template <typename T>
std::string Robot<T>::GetAddress() {
  return impl_->GetAddress();
}

template <typename T>
bool Robot<T>::Connect(int max_retries, int timeout_ms, std::function<bool()> signal_check) {
  return impl_->Connect(max_retries, timeout_ms, signal_check);
}

template <typename T>
void Robot<T>::Disconnect() {
  impl_->Disconnect();
}

template <typename T>
bool Robot<T>::IsConnected() const {
  return impl_->IsConnected();
}

template <typename T>
RobotInfo Robot<T>::GetRobotInfo() const {
  return ProtoToRobotInfo(impl_->GetRobotInfo());
}

template <typename T>
double Robot<T>::GetTimeScale() const {
  return impl_->GetTimeScale();
}

template <typename T>
double Robot<T>::SetTimeScale(double time_scale) const {
  return impl_->SetTimeScale(time_scale);
}

template <typename T>
bool Robot<T>::PowerOn(const std::string& dev_name) const {
  return impl_->PowerOn(dev_name);
}

template <typename T>
bool Robot<T>::PowerOff(const std::string& dev_name) const {
  return impl_->PowerOff(dev_name);
}

template <typename T>
bool Robot<T>::IsPowerOn(const std::string& dev_name) const {
  return impl_->IsPowerOn(dev_name);
}

template <typename T>
bool Robot<T>::ServoOn(const std::string& dev_name) const {
  return impl_->ServoOn(dev_name);
}

template <typename T>
bool Robot<T>::IsServoOn(const std::string& dev_name) const {
  return impl_->IsServoOn(dev_name);
}

template <typename T>
bool Robot<T>::ServoOff(const std::string& dev_name) const {
  return impl_->ServoOff(dev_name);
}

template <typename T>
bool Robot<T>::SetPositionPGain(const std::string& dev_name, uint16_t p_gain) const {
  return impl_->SetPositionGain(dev_name, p_gain, std::nullopt, std::nullopt);
}

template <typename T>
bool Robot<T>::SetPositionIGain(const std::string& dev_name, uint16_t i_gain) const {
  return impl_->SetPositionGain(dev_name, std::nullopt, i_gain, std::nullopt);
}

template <typename T>
bool Robot<T>::SetPositionDGain(const std::string& dev_name, uint16_t d_gain) const {
  return impl_->SetPositionGain(dev_name, std::nullopt, std::nullopt, d_gain);
}

template <typename T>
bool Robot<T>::SetPositionPIDGain(const std::string& dev_name, uint16_t p_gain, uint16_t i_gain,
                                  uint16_t d_gain) const {
  return impl_->SetPositionGain(dev_name, p_gain, i_gain, d_gain);
}

template <typename T>
bool Robot<T>::SetPositionPIDGain(const std::string& dev_name, const rb::PIDGain& pid_gain) const {
  return impl_->SetPositionGain(dev_name, pid_gain.p_gain, pid_gain.i_gain, pid_gain.d_gain);
}

template <typename T>
std::vector<rb::PIDGain> Robot<T>::GetTorsoPositionPIDGains() const {
  return impl_->GetTorsoPositionPIDGains();
}

template <typename T>
std::vector<rb::PIDGain> Robot<T>::GetRightArmPositionPIDGains() const {
  return impl_->GetRightArmPositionPIDGains();
}

template <typename T>
std::vector<rb::PIDGain> Robot<T>::GetLeftArmPositionPIDGains() const {
  return impl_->GetLeftArmPositionPIDGains();
}

template <typename T>
std::vector<rb::PIDGain> Robot<T>::GetHeadPositionPIDGains() const {
  return impl_->GetHeadPositionPIDGains();
}

template <typename T>
rb::PIDGain Robot<T>::GetPositionPIDGain(const std::string& dev_name) const {
  return impl_->GetPositionPIDGain(dev_name);
}

template <typename T>
bool Robot<T>::BreakEngage(const std::string& dev_name) const {
  return impl_->BreakEngage(dev_name);
}

template <typename T>
bool Robot<T>::BreakRelease(const std::string& dev_name) const {
  return impl_->BreakRelease(dev_name);
}

template <typename T>
bool Robot<T>::HomeOffsetReset(const std::string& dev_name) const {
  return impl_->HomeOffsetReset(dev_name);
}

template <typename T>
bool Robot<T>::SetPresetPosition(const std::string& joint_name) const {
  return impl_->SetPresetPosition(joint_name);
}

template <typename T>
bool Robot<T>::EnableControlManager(bool unlimited_mode_enabled) const {
  return impl_->EnableControlManager(unlimited_mode_enabled);
}

template <typename T>
bool Robot<T>::DisableControlManager() const {
  return impl_->DisableControlManager();
}

template <typename T>
bool Robot<T>::ResetFaultControlManager() const {
  return impl_->ResetFaultControlManager();
}

template <typename T>
bool Robot<T>::CancelControl() const {
  return impl_->CancelControl();
}

template <typename T>
bool Robot<T>::SetToolFlangeOutputVoltage(const std::string& name, int voltage) const {
  return impl_->SetToolFlangeOutputVoltage(name, voltage);
}

template <typename T>
bool Robot<T>::SetToolFlangeDigitalOutput(const std::string& name, unsigned int channel, bool state) const {
  return impl_->SetToolFlangeDigitalOutput(name, channel, state);
}

template <typename T>
bool Robot<T>::SetToolFlangeDigitalOutputDual(const std::string& name, bool state_0, bool state_1) const {
  return impl_->SetToolFlangeDigitalOutputDual(name, state_0, state_1);
}

template <typename T>
void Robot<T>::StartStateUpdate(const std::function<void(const RobotState<T>&)>& cb, double rate) {
  StartStateUpdate([cb](const RobotState<T>& rs, const ControlManagerState&) { cb(rs); }, rate);
}

template <typename T>
void Robot<T>::StartStateUpdate(const std::function<void(const RobotState<T>&, const ControlManagerState&)>& cb,
                                double rate) {
  impl_->StartStateUpdate(cb, rate);
}

template <typename T>
void Robot<T>::StopStateUpdate() {
  impl_->StopStateUpdate();
}

template <typename T>
void Robot<T>::StartLogStream(const std::function<void(const std::vector<Log>&)>& cb, double rate) {
  impl_->StartLogStream(cb, rate);
}

template <typename T>
void Robot<T>::StopLogStream() {
  impl_->StopLogStream();
}

template <typename T>
RobotState<T> Robot<T>::GetState() const {
  return impl_->GetState();
}

template <typename T>
std::vector<Log> Robot<T>::GetLastLog(unsigned int count) const {
  return impl_->GetLastLog(count);
}

template <typename T>
std::vector<std::string> Robot<T>::GetFaultLogList() const {
  return impl_->GetFaultLogList();
}

template <typename T>
ControlManagerState Robot<T>::GetControlManagerState() const {
  return impl_->GetControlManagerState();
}

template <typename T>
std::unique_ptr<RobotCommandHandler<T>> Robot<T>::SendCommand(const RobotCommandBuilder& builder, int priority) {
  return impl_->SendCommand(builder, priority);
}

template <typename T>
std::unique_ptr<RobotCommandStreamHandler<T>> Robot<T>::CreateCommandStream(int priority) {
  return impl_->CreateCommandStream(priority);
}

template <typename T>
bool Robot<T>::Control(std::function<ControlInput<T>(const ControlState<T>&)> control, int port, int priority) {
  return impl_->Control(std::move(control), port, priority);
}

template <typename T>
bool Robot<T>::ResetOdometry(double angle, const Eigen::Vector<double, 2>& position) {
  return impl_->ResetOdometry(angle, position);
}

template <typename T>
std::vector<std::pair<std::string, int>> Robot<T>::GetParameterList() const {
  return impl_->GetParameterList();
}

template <typename T>
bool Robot<T>::SetParameter(const std::string& name, const std::string& value, bool write_db) {
  return impl_->SetParameter(name, value, write_db);
}

template <typename T>
std::string Robot<T>::GetParameter(const std::string& name) const {
  return impl_->GetParameter(name);
}

template <typename T>
bool Robot<T>::FactoryResetParameter(const std::string& name) const {
  return impl_->FactoryResetParameter(name);
}

template <typename T>
void Robot<T>::FactoryResetAllParameters() const {
  impl_->FactoryResetAllParameters();
}

template <typename T>
bool Robot<T>::ResetParameter(const std::string& name) const {
  return impl_->ResetParameter(name);
}

template <typename T>
void Robot<T>::ResetAllParameters() const {
  impl_->ResetAllParameters();
}

template <typename T>
bool Robot<T>::ResetParameterToDefault(const std::string& name) const {
  return impl_->ResetParameterToDefault(name);
}

template <typename T>
void Robot<T>::ResetAllParametersToDefault() const {
  impl_->ResetAllParametersToDefault();
}

template <typename T>
std::string Robot<T>::GetRobotModel() const {
  return impl_->GetRobotModel();
}

template <typename T>
bool Robot<T>::ImportRobotModel(const std::string& name, const std::string& model) const {
  return impl_->ImportRobotModel(name, model);
}

template <typename T>
bool Robot<T>::SyncTime() {
  return impl_->SyncTime();
}

template <typename T>
bool Robot<T>::HasEstablishedTimeSync() {
  return impl_->HasEstablishedTimeSync();
}

template <typename T>
bool Robot<T>::StartTimeSync(long period_sec) {
  return impl_->StartTimeSync(period_sec);
}

template <typename T>
bool Robot<T>::StopTimeSync() {
  return impl_->StopTimeSync();
}

template <typename T>
std::shared_ptr<dyn::Robot<T::kRobotDOF>> Robot<T>::GetDynamics(const std::string& urdf_model) {
  return impl_->GetDynamics(urdf_model);
}

template <typename T>
bool Robot<T>::SetLEDColor(const rb::Color& color, double duration, double transition_time, bool blinking,
                           double blinking_freq) {
  return impl_->SetLEDColor(color, duration, transition_time, blinking, blinking_freq);
}

template <typename T>
std::tuple<struct timespec, std::string, std::string> Robot<T>::GetSystemTime() const {
  return impl_->GetSystemTime();
}

template <typename T>
bool Robot<T>::SetSystemTime(struct timespec utc_time, std::optional<std::string> time_zone) const {
  return impl_->SetSystemTime(utc_time, time_zone);
}

template <typename T>
bool Robot<T>::SetBatteryLevel(const double level) const {
  return impl_->SetBatteryLevel(level);
}

template <typename T>
bool Robot<T>::SetBatteryConfig(double cutoff_voltage, double fully_charged_voltage,
                                const std::array<double, 4>& coefficients) {
  return impl_->SetBatteryConfig(cutoff_voltage, fully_charged_voltage, coefficients);
}

template <typename T>
bool Robot<T>::ResetBatteryConfig() const {
  return impl_->ResetBatteryConfig();
}

template <typename T>
bool Robot<T>::WaitForControlReady(long timeout_ms) const {
  return impl_->WaitForControlReady(timeout_ms);
}

template <typename T>
bool Robot<T>::ResetNetworkSetting() const {
  return impl_->ResetNetworkSetting();
}

template <typename T>
std::vector<WifiNetwork> Robot<T>::ScanWifi() const {
  return impl_->ScanWifi();
}

template <typename T>
bool Robot<T>::ConnectWifi(const std::string& ssid, const std::string& password, bool use_dhcp,
                           const std::string& ip_address, const std::string& gateway,
                           const std::vector<std::string>& dns) const {
  return impl_->ConnectWifi(ssid, password, use_dhcp, ip_address, gateway, dns);
}

template <typename T>
bool Robot<T>::DisconnectWifi() const {
  return impl_->DisconnectWifi();
}

template <typename T>
std::optional<WifiStatus> Robot<T>::GetWifiStatus() const {
  return impl_->GetWifiStatus();
}

template <typename T>
std::vector<SerialDevice> Robot<T>::GetSerialDeviceList() const {
  return impl_->GetSerialDeviceList();
}

template <typename T>
std::unique_ptr<SerialStream> Robot<T>::OpenSerialStream(const std::string& device_path, int buadrate, int bytesize,
                                                         char parity, int stopbits) const {
  return impl_->OpenSerialStream(device_path, buadrate, bytesize, parity, stopbits);
}

template <typename T>
bool Robot<T>::DownloadFile(const std::string& path, std::ostream& output) const {
  return impl_->DownloadFile(path, output);
}

template <typename T>
bool Robot<T>::DownloadFileToCallback(const std::string& path,
                                      std::function<void(const char*, size_t)> on_chunk) const {
  return impl_->DownloadFileToCallback(path, on_chunk);
}

template <typename T>
RobotCommandHandler<T>::RobotCommandHandler(std::unique_ptr<RobotCommandHandlerImpl<T>> impl)
    : impl_(std::move(impl)) {}

template <typename T>
RobotCommandHandler<T>::~RobotCommandHandler() {
  Cancel();
}

template <typename T>
bool RobotCommandHandler<T>::IsDone() const {
  return impl_->IsDone();
}

template <typename T>
void RobotCommandHandler<T>::Wait() {
  impl_->Wait();
}

template <typename T>
bool RobotCommandHandler<T>::WaitFor(int timeout_ms) {
  return impl_->WaitFor(timeout_ms);
}

template <typename T>
void RobotCommandHandler<T>::Cancel() {
  impl_->Cancel();
}

template <typename T>
RobotCommandFeedback RobotCommandHandler<T>::Get() {
  const auto& msg = impl_->Get();

  RobotCommandFeedback feedback;

  RobotCommandFeedbackParser parser;
  parser.Parse(feedback, (void*)&msg);

  return feedback;
}

template <typename T>
bool RobotCommandHandler<T>::GetStatus() const {
  return impl_->GetStatus().ok();
}

template <typename T>
RobotCommandStreamHandler<T>::RobotCommandStreamHandler(std::unique_ptr<RobotCommandStreamHandlerImpl<T>> impl)
    : impl_(std::move(impl)) {}

template <typename T>
RobotCommandStreamHandler<T>::~RobotCommandStreamHandler() = default;

template <typename T>
bool RobotCommandStreamHandler<T>::IsDone() const {
  return impl_->IsDone();
}

template <typename T>
void RobotCommandStreamHandler<T>::Wait() {
  impl_->Wait();
}

template <typename T>
bool RobotCommandStreamHandler<T>::WaitFor(int timeout_ms) {
  return impl_->WaitFor(timeout_ms);
}

template <typename T>
void RobotCommandStreamHandler<T>::Cancel() {
  impl_->Cancel();
}

template <typename T>
RobotCommandFeedback RobotCommandStreamHandler<T>::SendCommand(const RobotCommandBuilder& builder, int timeout_ms) {
  const auto& msg = impl_->SendCommand(builder, timeout_ms);

  RobotCommandFeedback feedback;

  RobotCommandFeedbackParser parser;
  parser.Parse(feedback, (void*)&msg);

  return feedback;
}

template <typename T>
RobotCommandFeedback RobotCommandStreamHandler<T>::RequestFeedback(int timeout_ms) {
  const auto& msg = impl_->RequestFeedback(timeout_ms);

  RobotCommandFeedback feedback;

  RobotCommandFeedbackParser parser;
  parser.Parse(feedback, (void*)&msg);

  return feedback;
}

SerialStream::SerialStream(std::unique_ptr<SerialStreamImpl> impl) : impl_(std::move(impl)) {}

SerialStream::~SerialStream() = default;

bool SerialStream::Connect(bool verbose) const {
  return impl_->Connect(verbose);
}

void SerialStream::Disconnect() const {
  impl_->Disconnect();
}

void SerialStream::Wait() const {
  impl_->Wait();
}

bool SerialStream::WaitFor(int timeout_ms) const {
  return impl_->WaitFor(timeout_ms);
}

bool SerialStream::IsOpened() const {
  return impl_->IsOpened();
}

bool SerialStream::IsCancelled() const {
  return impl_->IsCancelled();
}

bool SerialStream::IsDone() const {
  return impl_->IsDone();
}

void SerialStream::SetReadCallback(const std::function<void(const std::string&)>& cb) {
  impl_->SetReadCallback(cb);
}

bool SerialStream::Write(const std::string& data) {
  return impl_->WriteData(data);
}

bool SerialStream::Write(const char* data) {
  return impl_->WriteData(data);
}

bool SerialStream::Write(const char* data, int n) {
  return impl_->WriteData(data, n);
}

bool SerialStream::WriteByte(char ch) {
  return impl_->WriteByteData(ch);
}

}  // namespace rb

template class rb::Robot<rb::y1_model::A>;
template class rb::RobotCommandHandler<rb::y1_model::A>;
template class rb::RobotCommandStreamHandler<rb::y1_model::A>;
template class rb::ControlInput<rb::y1_model::A>;
template class rb::ControlState<rb::y1_model::A>;

template class rb::Robot<rb::y1_model::T5>;
template class rb::RobotCommandHandler<rb::y1_model::T5>;
template class rb::RobotCommandStreamHandler<rb::y1_model::T5>;
template class rb::ControlInput<rb::y1_model::T5>;
template class rb::ControlState<rb::y1_model::T5>;

template class rb::Robot<rb::y1_model::M>;
template class rb::RobotCommandHandler<rb::y1_model::M>;
template class rb::RobotCommandStreamHandler<rb::y1_model::M>;
template class rb::ControlInput<rb::y1_model::M>;
template class rb::ControlState<rb::y1_model::M>;

template class rb::Robot<rb::y1_model::UB>;
template class rb::RobotCommandHandler<rb::y1_model::UB>;
template class rb::RobotCommandStreamHandler<rb::y1_model::UB>;
template class rb::ControlInput<rb::y1_model::UB>;
template class rb::ControlState<rb::y1_model::UB>;