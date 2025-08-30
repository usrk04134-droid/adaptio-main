#pragma once

#include <prometheus/registry.h>
#include <SQLiteCpp/Database.h>

#include <boost/log/detail/event.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <optional>

#include "bead_control/bead_control.h"
#include "common/clock_functions.h"
#include "common/filters/gaussian_filter.h"
#include "common/logging/component_logger.h"
#include "common/zevs/zevs_socket.h"
#include "delay_buffer.h"
#include "event_handler/event_codes.h"
#include "event_handler/event_handler.h"
#include "image_logging/image_logging_manager.h"
#include "joint_geometry/joint_geometry.h"
#include "kinematics/kinematics_client.h"
#include "lpcs/lpcs_slice.h"
#include "macs/macs_point.h"
#include "macs/macs_slice.h"
#include "performance_metrics.h"
#include "scanner_client/scanner_client.h"
#include "slice_translator/slice_observer.h"
#include "slice_translator/slice_translator_service_v2.h"
#include "tracking/tracking_manager.h"
#include "web_hmi/web_hmi.h"
#include "weld_control/src/confident_slice_buffer.h"
#include "weld_control/src/settings.h"
#include "weld_control/src/settings_provider.h"
#include "weld_control/weld_control.h"
#include "weld_control/weld_control_types.h"
#include "weld_control/weld_state_observer.h"
#include "weld_metrics.h"
#include "weld_process_parameters.h"
#include "weld_sequence_config.h"
#include "weld_system_client/weld_system_client.h"
#include "weld_system_client/weld_system_types.h"

namespace weld_control {

enum class State {
  IDLE,
  WELDING,
};

class WeldControlImpl : public WeldControl,
                        public slice_translator::SliceObserver,
                        public scanner_client::ScannerObserver {
 public:
  explicit WeldControlImpl(const Configuration& config, WeldSequenceConfig* weld_sequence_config,
                           SettingsProvider* settings_provider, web_hmi::WebHmi* web_hmi,
                           kinematics::KinematicsClient* kinematics_client, std::filesystem::path const& path_logs,
                           weld_system::WeldSystemClient* weld_system_client,
                           tracking::TrackingManager* tracking_manager, scanner_client::ScannerClient* scanner_client,
                           zevs::Timer* timer, event::EventHandler* event_handler,
                           bead_control::BeadControl* bead_control, DelayBuffer* delay_buffer,
                           clock_functions::SystemClockNowFunc system_clock_now_func,
                           clock_functions::SteadyClockNowFunc steady_clock_now_func, prometheus::Registry* registry,
                           image_logging::ImageLoggingManager* image_logging_manager,
                           slice_translator::SliceTranslatorServiceV2* slice_translator_v2, SQLite::Database* db);

  /* WeldControl */
  void JointTrackingStart(const joint_geometry::JointGeometry& joint_geometry,
                          tracking::TrackingMode joint_tracking_mode, double horizontal_offset,
                          double vertical_offset) override;
  void JointTrackingUpdate(tracking::TrackingMode joint_tracking_mode, double horizontal_offset,
                           double vertical_offset) override;
  void AutoBeadPlacementStart(LayerType layer_type) override;
  void AutoBeadPlacementStop() override;
  void Stop() override;
  void SetObserver(WeldControlObserver* observer) override;
  auto GetObserver() const -> WeldControlObserver* override;
  void SubscribeReady(std::function<void(const std::vector<std::pair<Mode, LayerType>>&)> on_ready_update) override;
  void ResetGrooveData() override;
  void AddWeldStateObserver(WeldStateObserver* observer) override;

  /* ScannerObserver */
  void OnScannerStarted(bool success) override;
  void OnScannerStopped(bool success) override;
  void OnScannerDataUpdate(const lpcs::Slice& data, const macs::Point& axis_position) override {};

  /* SliceObserver */
  void Receive(const macs::Slice& machine_data, const lpcs::Slice& scanner_data, const macs::Point& slides_actual,
               double angle_from_torch_to_scanner) override;

  auto GetMode() const -> Mode { return mode_; }
  auto GetState() const -> State { return state_; }
  auto GetTrackingMode() const -> tracking::TrackingMode { return tracking_mode_; }

 private:
  Configuration config_;
  WeldSequenceConfig* weld_sequence_config_;
  SettingsProvider* settings_provider_;
  Settings settings_;
  web_hmi::WebHmi* web_hmi_;
  kinematics::KinematicsClient* kinematics_client_;
  weld_system::WeldSystemClient* weld_system_client_;
  tracking::TrackingManager* tracking_manager_;
  scanner_client::ScannerClient* scanner_client_;
  zevs::Timer* timer_;
  event::EventHandler* event_handler_;
  image_logging::ImageLoggingManager* image_logging_manager_;
  slice_translator::SliceTranslatorServiceV2* slice_translator_v2_;
  bead_control::BeadControl* bead_control_;
  DelayBuffer* delay_buffer_;
  ConfidentSliceBuffer confident_slice_buffer_;
  common::logging::ComponentLogger weld_logger_;
  WeldControlObserver* observer_{nullptr};
  std::chrono::time_point<std::chrono::system_clock> last_log_;
  Mode mode_{Mode::IDLE};
  State state_{State::IDLE};
  LayerType abp_layer_type_{LayerType::FILL};
  std::vector<WeldStateObserver*> weld_state_observers_;

  double cached_weld_axis_position_{0.0};
  double cached_weld_axis_ang_velocity_{0.0};
  double cached_weld_object_radius_{0.0};
  double cached_edge_position_{0.0};

  std::optional<double> last_weld_axis_position_;
  bool ready_for_auto_cap_{false};

  macs::Slice cached_mcs_{};
  lpcs::Slice cached_lpcs_{};
  std::optional<macs::Slice> last_confident_mcs_;
  std::optional<lpcs::Slice> last_confident_lpcs_;
  double cached_torch_to_scanner_angle_{0.0};

  struct WeldSystem {
    weld_system::WeldSystemState state{weld_system::WeldSystemState::INIT};
    weld_system::WeldSystemData data{};
    weld_system::WeldSystemSettings settings{};
    std::optional<std::chrono::steady_clock::time_point> arcing_lost_timestamp;
  };
  std::map<weld_system::WeldSystemId, WeldSystem> weld_systems_;

  std::optional<std::chrono::steady_clock::time_point> scanner_no_confidence_timestamp_;
  std::optional<std::chrono::steady_clock::time_point> scanner_low_confidence_timestamp_;
  std::optional<std::chrono::steady_clock::time_point> handover_to_jt_timestamp_;
  std::optional<std::chrono::steady_clock::time_point> handover_to_abp_cap_timestamp_;
  std::optional<std::chrono::steady_clock::time_point> handover_to_manual_timestamp_;
  double cached_groove_area_{0.0};
  std::optional<macs::Groove> cached_delayed_mcs_;
  std::optional<tracking::TrackingManager::Output> slides_desired_;
  macs::Point slides_actual_;
  double horizontal_offset_{0.0};
  double vertical_offset_{0.0};
  tracking::TrackingMode tracking_mode_{tracking::TrackingMode::TRACKING_LEFT_HEIGHT};
  double bead_slice_area_ratio_{1.0};
  double groove_area_ratio_{1.0};
  double bead_control_horizontal_offset_{0.0};
  double weld_axis_velocity_desired_{0.0};

  bool pending_get_weld_axis_data_{false};
  bool pending_get_weld_system1_data_{false};
  bool pending_get_weld_system2_data_{false};
  bool pending_get_edge_position_{false};
  bool pending_scanner_stop_{false};

  std::function<void(std::vector<std::pair<Mode, LayerType>>)> on_ready_update_;

  clock_functions::SystemClockNowFunc system_clock_now_func_;
  clock_functions::SteadyClockNowFunc steady_clock_now_func_;
  kinematics::State weld_axis_state_{kinematics::State::INIT};
  kinematics::EdgeState edge_state_{kinematics::EdgeState::NOT_AVAILABLE};

  /* Metrics */
  std::unique_ptr<PerformanceMetrics> perf_metrics_;
  std::unique_ptr<WeldMetrics> weld_metrics_;

  common::filters::GaussianFilter smooth_weld_speed_;
  common::filters::GaussianFilter smooth_ws2_current_;

  struct {
    std::map<lpcs::SliceConfidence, prometheus::Counter*> slice_confidence;
    struct {
      prometheus::Counter* ok;
      prometheus::Counter* no_data;
      prometheus::Counter* translation_failed;
    } confident_slice;
    prometheus::Gauge* confident_slice_buffer_fill_ratio;
  } metrics_;

  /* WebHMI */
  void GetWeldControlStatus() const;

  void WeldSystemStateChange(weld_system::WeldSystemId id, weld_system::WeldSystemState state);
  void LogData(std::optional<std::string> annotation);
  void LogDataRateLimited();
  void LogModeChange();
  void UpdateConfidentSlice();
  void UpdateReadyForABPCap();
  void ProcessInput();
  auto GetDelayedGrooveMCS(double delay) -> macs::Groove;
  auto StoreGrooveInDelayBuffer() -> bool;
  auto GetHybridGrooveMCS() const -> macs::Groove;
  auto GetSampleToTorchDistRad(uint64_t ts_sample, double ang_velocity, double torch_to_scanner_angle) -> double;
  auto GetSmoothMCS(double smooth_ang_distance) -> std::optional<macs::Groove>;
  void UpdateTrackingPosition();
  void SetupMetrics(prometheus::Registry* registry);
  void UpdateBeadControlParameters();
  void CheckReady();
  auto JTReady() const -> bool;
  auto ABPReady() const -> bool;
  void UpdateReady();
  void UpdateOutput(double bead_slice_area_ratio, double area_ratio);
  auto ValidateInput() -> std::optional<event::Code>;
  auto CheckSupervision() -> std::optional<event::Code>;
  void ChangeMode(Mode new_mode);
  void ChangeState(State new_state);
  auto CheckHandover() -> bool;
  auto UpdateSliceConfidence() -> bool;
};

}  // namespace weld_control
