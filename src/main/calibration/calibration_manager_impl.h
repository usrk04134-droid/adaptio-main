#pragma once

#include <prometheus/registry.h>
#include <SQLiteCpp/Database.h>

#include <memory>
#include <nlohmann/json.hpp>

#include "calibration_manager.h"
#include "calibration_metrics.h"
#include "common/zevs/zevs_socket.h"
#include "coordination/calibration_status.h"
#include "joint_geometry/joint_geometry.h"
#include "scanner_client/scanner_client.h"
#include "slice_translator/slice_translator.h"

namespace calibration {

enum class CalibrationActivity : uint32_t {
  IDLE = 0,
  LASER_TORCH_CALIBRATION_ONGOING,
  WELD_OBJECT_CALIBRATION_ONGOING,
};

class CalibrationManagerImpl : public CalibrationManager,
                               public scanner_client::ScannerObserver,
                               public coordination::CalibrationStatus {
 public:
  explicit CalibrationManagerImpl(zevs::Timer* timer, scanner_client::ScannerClient* scanner,
                                  slice_translator::SliceTranslator* slice_translator, prometheus::Registry* registry);

  CalibrationManagerImpl(CalibrationManagerImpl&)                     = delete;
  auto operator=(CalibrationManagerImpl&) -> CalibrationManagerImpl&  = delete;
  CalibrationManagerImpl(CalibrationManagerImpl&&)                    = delete;
  auto operator=(CalibrationManagerImpl&&) -> CalibrationManagerImpl& = delete;

  ~CalibrationManagerImpl() override = default;

  void SetObserver(CalibrationObserver* observer) override;
  auto GetObserver() const -> CalibrationObserver* override;
  void StartCalibrateLaserToTorch(const joint_geometry::JointGeometry& calibration_joint_geometry, double offset,
                                  double angle, double stickout) override;
  void StartCalibrateLaserToTorch(const joint_geometry::JointGeometry& calibration_joint_geometry) override;
  void StartCalibrateWeldObject(const joint_geometry::JointGeometry& joint_geometry, double radius,
                                double stickout) override;
  void StartCalibrateWeldObject(const joint_geometry::JointGeometry& joint_geometry) override;
  void Stop() override;
  auto GetLaserToTorchCalibration() const -> std::optional<LaserTorchCalibration> override;
  auto SetLaserToTorchCalibration(const LaserTorchCalibration& data) -> bool override;
  auto GetWeldObjectCalibration() const -> std::optional<WeldObjectCalibration> override;
  auto SetWeldObjectCalibration(const WeldObjectCalibration& data) -> bool override;

  // ScannerObserver
  void OnScannerStarted(bool success) override {};
  void OnScannerStopped(bool success) override {};
  void OnScannerDataUpdate(const lpcs::Slice& data, const macs::Point& axis_position) override;

  // coordination::CalibrationStatus
  auto LaserToTorchCalibrationValid() const -> bool override;
  auto WeldObjectCalibrationValid() const -> bool override;
  void Subscribe(std::function<void()> on_update) override;
  void SetWOCalCoordinator(std::function<bool()> wo_cal_coordinator);

 private:
  friend class CalibrationScannerObserver;
  void TryCalibrateLaserToTorch(const lpcs::Slice& data);
  void TryCalibrateWeldObject(const lpcs::Slice& data, const macs::Point& axis_position);
  void OnTimeout();

  zevs::Timer* timer_;
  scanner_client::ScannerClient* scanner_client_;
  slice_translator::SliceTranslator* slice_translator_;
  CalibrationActivity status_ = CalibrationActivity::IDLE;
  CalibrationObserver* observer_;
  std::optional<uint32_t> timer_task_id_;
  double offset_   = 50;
  double angle_    = 20;
  double stickout_ = 20;
  double radius_   = 4000;
  std::function<void()> on_update_;
  std::unique_ptr<CalibrationMetrics> cal_metrics_;
  std::function<bool()> wo_cal_coordinator_;
};

}  // namespace calibration
