#include "helpers.h"

#include <doctest/doctest.h>
#include <prometheus/registry.h>
#include <SQLiteCpp/Database.h>

#include <boost/outcome.hpp>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "application.h"
#include "common/clock_functions.h"
#include "common/logging/application_log.h"
#include "common/messages/scanner.h"
#include "common/zevs/zevs_test_support.h"
#include "configuration/config_manager.h"
#include "test_utils/testlog.h"
#include "weld_control/weld_control_types.h"

using configuration::ConfigManager;

const uint32_t TIMER_INSTANCE = 1;

ApplicationWrapper::ApplicationWrapper(SQLite::Database* database,
                                       clock_functions::SystemClockNowFunc system_clock_now_func,
                                       clock_functions::SteadyClockNowFunc steady_clock_now_func,
                                       std::filesystem::path test_config_file)

    : system_clock_now_func_(system_clock_now_func), steady_clock_now_func_(steady_clock_now_func) {
  configuration_ = std::make_unique<ConfigManager>("");
  std::optional<std::filesystem::path> config_file;

  // Resolve config path robustly by searching upwards for tests/configs/sil/configuration.yaml
  auto resolve_config = []() -> std::filesystem::path {
    namespace fs = std::filesystem;
    fs::path rel{"tests/configs/sil/configuration.yaml"};
    fs::path dir = fs::current_path();
    for (int i = 0; i < 6; ++i) {
      fs::path candidate = dir / rel;
      if (fs::exists(candidate)) {
        return candidate;
      }
      if (!dir.has_parent_path()) {
        break;
      }
      dir = dir.parent_path();
    }
    return fs::current_path() / rel;  // fallback
  };
  auto default_config = resolve_config();
  if (!test_config_file.empty()) {
    // if relative, resolve relative to the directory we found default_config in
    auto base = default_config.parent_path();
    auto candidate = base / test_config_file;
    config_file = candidate;
  }

  if (configuration_->Init(default_config, config_file, default_config.parent_path()) != boost::outcome_v2::success()) {
    LOG_ERROR("Init of configuration failed");
    return;
  }

  registry_    = std::make_shared<prometheus::Registry>();
  application_ = std::make_unique<Application>(configuration_.get(), std::filesystem::path("assets/events/events.yaml"),
                                               database, "/var/log/adaptio/", system_clock_now_func_,
                                               steady_clock_now_func_, registry_.get(), -1);
}

void ApplicationWrapper::Start() {
  if (!application_) {
    LOG_ERROR("Application not initialized - configuration init likely failed");
    return;
  }
  application_->Run("Application", "adaptio");
}

auto ApplicationWrapper::InShutdown() const -> bool { return application_->InShutdown(); }

auto ApplicationWrapper::Registry() -> prometheus::Registry* { return registry_.get(); }

auto ApplicationWrapper::GetWeldControlConfig() -> weld_control::Configuration {
  return configuration_->GetWeldControlConfiguration();
}

ClockNowFuncWrapper::ClockNowFuncWrapper()
    : system_clock_latest_(std::chrono::system_clock::now().time_since_epoch()),
      steady_clock_latest_(std::chrono::steady_clock::now().time_since_epoch()) {};
auto ClockNowFuncWrapper::GetSystemClock() -> std::chrono::time_point<std::chrono::system_clock> {
  return std::chrono::time_point<std::chrono::system_clock>(system_clock_latest_);
};
auto ClockNowFuncWrapper::GetSteadyClock() -> std::chrono::time_point<std::chrono::steady_clock> {
  return std::chrono::time_point<std::chrono::steady_clock>(steady_clock_latest_);
};

TimerWrapper::TimerWrapper(clock_functions::SteadyClockNowFunc steady_clock_now_func)
    : steady_clock_now_func_(steady_clock_now_func) {};

void TimerWrapper::RequestTimer(uint32_t duration_ms, bool periodic, const std::string& task_name) {
  timer_tasks_.insert(
      {std::chrono::milliseconds(duration_ms), periodic, task_name, steady_clock_now_func_().time_since_epoch()});
}

void TimerWrapper::CancelTimer(const std::string& task_name) {
  std::erase_if(timer_tasks_, [task_name](const auto& task) { return task.name == task_name; });
}

void TimerWrapper::DispatchAllExpired() {
  std::set<std::string> expired_tasks;
  if (!dispatch_handler_) {
    return;  // Not yet configured
  }
  // Finds expired timeouts and indicate to be removed if non periodic
  auto remove = [this, &expired_tasks](TimerTask task) {
    if (steady_clock_now_func_ != nullptr &&
        (steady_clock_now_func_().time_since_epoch() > (task.request_time + task.duration_ms))) {
      expired_tasks.insert({task.name});
      if (task.periodic) {
        task.request_time = steady_clock_now_func_().time_since_epoch();
      }
      return !task.periodic;
    }
    return false;
  };
  std::erase_if(timer_tasks_, remove);
  // Dispatch expired tasks
  for (const auto& task : expired_tasks) {
    dispatch_handler_(task);
  }
}

void TimerWrapper::SetDispatchHandler(DispatchHandler dispatch_handler) {
  dispatch_handler_ = std::move(dispatch_handler);
}

inline auto TimerWrapper::TimerTask::operator<(const TimerTask& other) const -> bool {
  return (name < other.name) || ((name == other.name));
}

void TestFixture::SetupTimerWrapper() {
  timer_mocket_->SetRequestObserver([this](uint32_t duration_ms, bool periodic, const std::string& task_name) {
    GetTimerWrapper()->RequestTimer(duration_ms, periodic, task_name);
  });
  timer_mocket_->SetCancelObserver([this](const std::string& task_name) { GetTimerWrapper()->CancelTimer(task_name); });
  timer_wrapper_->SetDispatchHandler([this](const std::string& task_name) { Timer()->Dispatch(task_name); });
}

void TestFixture::SetupMockets() {
  // LOG_DEBUG("factory contains: {}", factory.Describe());
  // bind_endpoints: {inproc://adaptio/WebHmiIn, inproc://adaptio/WebHmiOut}
  // connect_endpoints: {inproc://adaptio/control, inproc://adaptio/kinematics,inproc://adaptio/scanner}

  web_hmi_in_mocket_  = factory_.GetMocket(zevs::Endpoint::BIND, "tcp://0.0.0.0:5555");
  web_hmi_out_mocket_ = factory_.GetMocket(zevs::Endpoint::BIND, "tcp://0.0.0.0:5556");
  management_mocket_  = factory_.GetMocket(zevs::Endpoint::CONNECT, "inproc://adaptio/management");
  kinematics_mocket_  = factory_.GetMocket(zevs::Endpoint::CONNECT, "inproc://adaptio/kinematics");
  scanner_mocket_     = factory_.GetMocket(zevs::Endpoint::CONNECT, "inproc://adaptio/scanner");
  weld_system_mocket_ = factory_.GetMocket(zevs::Endpoint::CONNECT, "inproc://adaptio/weld-system");
  timer_mocket_       = factory_.GetMocketTimer(TIMER_INSTANCE);
}

auto TestFixture::MocketsFound() const -> bool {
  return web_hmi_in_mocket_ && web_hmi_out_mocket_ && management_mocket_ && kinematics_mocket_ && scanner_mocket_ &&
         weld_system_mocket_;
}

auto TestFixture::StartedOK() const -> bool { return MocketsFound(); }

auto TestFixture::GetClockNowFuncWrapper() -> ClockNowFuncWrapper* { return clock_now_func_wrapper_.get(); }
auto TestFixture::GetTimerWrapper() -> TimerWrapper* { return timer_wrapper_.get(); }

TestFixture::TestFixture() : TestFixture(nullptr) {}
TestFixture::TestFixture(std::filesystem::path test_config_file) : TestFixture(nullptr, test_config_file) {}
TestFixture::TestFixture(SQLite::Database* database, std::filesystem::path test_config_file)
    : ptr_database_(database != nullptr ? database : &database_),
      // NOLINTNEXTLINE(hicpp-signed-bitwise)
      database_(SQLite::Database(":memory:", SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE)) {
  ptr_database_->exec("PRAGMA foreign_keys=on");

  clock_now_func_wrapper_            = std::make_shared<ClockNowFuncWrapper>();
  auto clock_system_now_func_wrapper = [this]() -> std::chrono::time_point<std::chrono::system_clock> {
    return clock_now_func_wrapper_->GetSystemClock();
  };
  auto clock_steady_now_func_wrapper = [this]() -> std::chrono::time_point<std::chrono::steady_clock> {
    return clock_now_func_wrapper_->GetSteadyClock();
  };
  timer_wrapper_   = std::make_shared<TimerWrapper>(clock_steady_now_func_wrapper);
  application_sut_ = std::make_unique<ApplicationWrapper>(ptr_database_, clock_system_now_func_wrapper,
                                                          clock_steady_now_func_wrapper, test_config_file);
  application_sut_->Start();
  SetupMockets();
  assert(StartedOK());

  TESTLOG_NOHDR("  --== fixture setup done - start test ==--")
}

auto TestFixture::Factory() -> zevs::MocketFactory* { return &factory_; }
auto TestFixture::WebHmiIn() -> zevs::Mocket* { return web_hmi_in_mocket_.get(); }
auto TestFixture::WebHmiOut() -> zevs::Mocket* { return web_hmi_out_mocket_.get(); }
auto TestFixture::Management() -> zevs::Mocket* { return management_mocket_.get(); }
auto TestFixture::Kinematics() -> zevs::Mocket* { return kinematics_mocket_.get(); }
auto TestFixture::Scanner() -> zevs::Mocket* { return scanner_mocket_.get(); }
auto TestFixture::WeldSystem() -> zevs::Mocket* { return weld_system_mocket_.get(); }
auto TestFixture::Timer() -> zevs::MocketTimer* { return timer_mocket_.get(); }

auto TestFixture::DescribeQueue() const -> std::string {
  std::string description = "{WebHmiIn:" + std::to_string(web_hmi_in_mocket_->Queued()) +
                            ", WebHmiOut:" + std::to_string(web_hmi_out_mocket_->Queued()) +
                            ", Management:" + std::to_string(management_mocket_->Queued()) +
                            ", Kinematics:" + std::to_string(kinematics_mocket_->Queued()) +
                            ", Scanner:" + std::to_string(scanner_mocket_->Queued()) +
                            ", WeldSystem:" + std::to_string(weld_system_mocket_->Queued()) + "}";

  return description;
}

auto TestFixture::ScannerData() -> ScannerDataWrapper* { return &scanner_data_; }

auto TestFixture::Sut() -> ApplicationWrapper* { return application_sut_.get(); }

const double TOP_LEVEL    = 0.0;
const double BOTTOM_LEVEL = -30.0;

ScannerDataWrapper::ScannerDataWrapper() {
  data_.groove[0] = {-22, TOP_LEVEL};
  data_.groove[1] = {-15, BOTTOM_LEVEL};
  data_.groove[2] = {-10, BOTTOM_LEVEL};
  data_.groove[3] = {0, BOTTOM_LEVEL};
  data_.groove[4] = {10, BOTTOM_LEVEL};
  data_.groove[5] = {15, BOTTOM_LEVEL};
  data_.groove[6] = {22, TOP_LEVEL};

  data_.confidence = common::msg::scanner::SliceConfidence::HIGH;

  data_.line[0] = {-22, TOP_LEVEL};
  data_.line[1] = {-15, BOTTOM_LEVEL};
  data_.line[2] = {-10, BOTTOM_LEVEL};
  data_.line[3] = {0, BOTTOM_LEVEL};
  data_.line[4] = {10, BOTTOM_LEVEL};
  data_.line[5] = {15, BOTTOM_LEVEL};
  data_.line[6] = {22, TOP_LEVEL};
}

auto ScannerDataWrapper::Get() const -> common::msg::scanner::SliceData { return data_; }
auto ScannerDataWrapper::GetWithConfidence(common::msg::scanner::SliceConfidence confidence) const
    -> common::msg::scanner::SliceData {
  auto data       = data_;
  data.confidence = confidence;

  return data;
}

auto ScannerDataWrapper::ShiftHorizontal(double value) -> ScannerDataWrapper& {
  for (auto& coord : data_.groove) {
    coord.x += value;
  }

  for (auto& coord : data_.line) {
    coord.x += value;
  }

  return *this;
}

auto ScannerDataWrapper::FillUp(double value) -> ScannerDataWrapper& {
  for (auto& coord : data_.groove) {
    if (coord.y < TOP_LEVEL) coord.y += value;
  }

  for (auto& coord : data_.line) {
    if (coord.y < TOP_LEVEL) coord.y += value;
  }

  return *this;
}
