#include "calibration/calibration_manager_impl.h"

#include <prometheus/registry.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>

#include "calibration/calibration_manager.h"
#include "calibration/calibration_metrics.h"
#include "calibration/calibration_types.h"
#include "common/logging/application_log.h"
#include "common/zevs/zevs_socket.h"
#include "joint_geometry/joint_geometry.h"
#include "lpcs/lpcs_slice.h"
#include "macs/macs_point.h"
#include "scanner_client/scanner_client.h"
#include "slice_translator/slice_translator.h"

using calibration::CalibrationManagerImpl;

const uint32_t MAX_CALIBRATION_DURATION_MS = 5000;

CalibrationManagerImpl::CalibrationManagerImpl(zevs::Timer* timer, scanner_client::ScannerClient* scanner_client,
                                               slice_translator::SliceTranslator* slice_translator,
                                               prometheus::Registry* registry)
    : timer_(timer),
      scanner_client_(scanner_client),
      slice_translator_(slice_translator),
      cal_metrics_(std::make_unique<CalibrationMetrics>(registry)) {
  scanner_client_->AddObserver(this);

  auto const ltc = slice_translator_->GetLaserToTorchCalibration();
  auto const woc = slice_translator_->GetWeldObjectCalibration();

  LOG_INFO("LaserTorchCalibration - {}",
           ltc.has_value() ? LaserTorchCalibrationToString(ltc.value()) : "not-calibrated");
  LOG_INFO("WeldObjectCalibration - {}",
           woc.has_value() ? WeldObjectCalibrationToString(woc.value()) : "not-calibrated");
}

void CalibrationManagerImpl::OnScannerDataUpdate(const lpcs::Slice& data, const macs::Point& axis_position) {
  switch (status_) {
    case CalibrationActivity::LASER_TORCH_CALIBRATION_ONGOING:
      TryCalibrateLaserToTorch(data);
      break;
    case CalibrationActivity::WELD_OBJECT_CALIBRATION_ONGOING:
      TryCalibrateWeldObject(data, axis_position);
    default:
      break;
  }
}

void CalibrationManagerImpl::SetObserver(CalibrationObserver* observer) { observer_ = observer; }

auto CalibrationManagerImpl::GetObserver() const -> CalibrationObserver* { return observer_; }

void CalibrationManagerImpl::Stop() {
  scanner_client_->Stop();
  status_ = CalibrationActivity::IDLE;

  if (timer_task_id_.has_value()) {
    timer_->Cancel(timer_task_id_.value());
    timer_task_id_ = {};
  }
}

auto CalibrationManagerImpl::GetLaserToTorchCalibration() const -> std::optional<LaserTorchCalibration> {
  return slice_translator_->GetLaserToTorchCalibration();
}

auto CalibrationManagerImpl::SetLaserToTorchCalibration(const LaserTorchCalibration& data) -> bool {
  if (slice_translator_->SetLaserToTorchCalibration(data) != boost::outcome_v2::success()) {
    LOG_ERROR("Failed to store laser to torch calibration");
    return false;
  }

  on_update_();
  return true;
}

auto CalibrationManagerImpl::GetWeldObjectCalibration() const -> std::optional<WeldObjectCalibration> {
  return slice_translator_->GetWeldObjectCalibration();
}

auto CalibrationManagerImpl::SetWeldObjectCalibration(const WeldObjectCalibration& data) -> bool {
  // Conditions for applying and storing a new weld object calibration:
  // 1. Not arcing
  // 2. If tracking, it must be tracking from center
  //
  // The above conditions are checked with a call to the wo_cal_coordinator function.
  // If conditions are fulfilled, this call also resets the delay buffer in order to
  // immediately have an impact on tracking position.
  //
  // A more straightforward coordination between calibration and welding functions
  // will be developed when moving weld function responsibility from PLC to Adaptio.
  if (wo_cal_coordinator_ && !wo_cal_coordinator_()) {
    return false;
  }

  if (slice_translator_->SetWeldObjectCalibration(data) != boost::outcome_v2::success()) {
    LOG_ERROR("Failed to store weld object calibration");
    return false;
  }

  on_update_();
  return true;
}

void CalibrationManagerImpl::StartCalibrateLaserToTorch(const joint_geometry::JointGeometry& calibration_joint_geometry,
                                                        double offset, double angle, double stickout) {
  offset_   = offset;
  angle_    = angle;
  stickout_ = stickout;
  StartCalibrateLaserToTorch(calibration_joint_geometry);
  cal_metrics_->IncrementLaserTorchCalCount();
}

void CalibrationManagerImpl::StartCalibrateLaserToTorch(
    const joint_geometry::JointGeometry& calibration_joint_geometry) {
  // TODO: Add 10 sec timeout supervision for this to be completed?
  LOG_INFO("Start LaserToTorch calibration with offset={}, angle={}, stickout={}, calibration_joint_geometry={}",
           offset_, angle_, stickout_, calibration_joint_geometry.ToString());

  scanner_client_->Start({.sensitivity = scanner_client::ScannerSensitivity::HIGH}, calibration_joint_geometry);
  status_ = CalibrationActivity::LASER_TORCH_CALIBRATION_ONGOING;

  timer_task_id_ =
      timer_->Request(&CalibrationManagerImpl::OnTimeout, this, MAX_CALIBRATION_DURATION_MS, "calibration_supervision");
}

void CalibrationManagerImpl::StartCalibrateWeldObject(const joint_geometry::JointGeometry& joint_geometry,
                                                      double radius, double stickout) {
  radius_   = radius;
  stickout_ = stickout;
  StartCalibrateWeldObject(joint_geometry);
  cal_metrics_->IncrementWeldObjectCalCount();
}

void CalibrationManagerImpl::StartCalibrateWeldObject(const joint_geometry::JointGeometry& joint_geometry) {
  // TODO: Add 10 sec timeout supervision for this to be completed?
  LOG_INFO("Start Weld Object calibration with radius={}, stickout={}, calibration_joint_geometry={}", radius_,
           stickout_, joint_geometry.ToString());

  scanner_client_->Start({}, joint_geometry);
  status_ = CalibrationActivity::WELD_OBJECT_CALIBRATION_ONGOING;

  timer_task_id_ =
      timer_->Request(&CalibrationManagerImpl::OnTimeout, this, MAX_CALIBRATION_DURATION_MS, "calibration_supervision");
}

void CalibrationManagerImpl::TryCalibrateLaserToTorch(const lpcs::Slice& data) {
  auto result = slice_translator_->CalibrateLaserToTorch(angle_, offset_, stickout_, data);

  if (!result) {
    // will try again on next scanner data
    LOG_DEBUG("Failed to calibrate LaserOrigin");
    return;
  }

  LOG_INFO("LaserTorchCalibration completed - {}", LaserTorchCalibrationToString(result.value()));

  observer_->LaserTorchCalibrationCompleted(*result);
  Stop();
}

void CalibrationManagerImpl::TryCalibrateWeldObject(const lpcs::Slice& data, const macs::Point& axis_position) {
  auto result = slice_translator_->CalibrateWeldObject(radius_, stickout_, data, axis_position);

  if (!result) {
    LOG_DEBUG("Failed to calibrate work object, keep trying until timeout");
    // will try again on next scanner data
    return;
  }

  if (result.value().z < 0) {
    LOG_INFO("WeldObjectCalibration completed - {}", WeldObjectCalibrationToString(result.value()));
    observer_->WeldObjectCalibrationCompleted(*result);
  } else {
    LOG_ERROR("Failed to calibrate weld object with z={} (should be < 0). Check clock position!", result.value().z);
    observer_->WeldObjectCalibrationFailed();
  }

  Stop();
}

void CalibrationManagerImpl::OnTimeout() {
  LOG_ERROR("Failed to complete calibration within {} ms", MAX_CALIBRATION_DURATION_MS);

  switch (status_) {
    case CalibrationActivity::LASER_TORCH_CALIBRATION_ONGOING:
      observer_->LaserTorchCalibrationFailed();
      break;
    case CalibrationActivity::WELD_OBJECT_CALIBRATION_ONGOING:
      observer_->WeldObjectCalibrationFailed();
      break;
    default:
      break;
  }

  Stop();
}

auto CalibrationManagerImpl::LaserToTorchCalibrationValid() const -> bool {
  return GetLaserToTorchCalibration().has_value();
}

auto CalibrationManagerImpl::WeldObjectCalibrationValid() const -> bool {
  return GetWeldObjectCalibration().has_value();
}

void CalibrationManagerImpl::Subscribe(std::function<void()> on_update) { on_update_ = on_update; }

void CalibrationManagerImpl::SetWOCalCoordinator(std::function<bool()> wo_cal_coordinator) {
  wo_cal_coordinator_ = wo_cal_coordinator;
}
