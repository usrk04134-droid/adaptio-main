

#pragma once

#include <boost/outcome.hpp>
#include <Eigen/Eigen>
#include <optional>
#include <vector>

#include "calibration/calibration_types.h"
#include "configuration/converter.h"
#include "lpcs/lpcs_slice.h"
#include "macs/macs_point.h"
#include "slice_translator/slice_translator.h"
#include "slice_translator_service.h"

namespace slice_translator {

class SliceTranslatorImpl : public SliceTranslator, public SliceTranslatorService {
 public:
  SliceTranslatorImpl(std::optional<calibration::LaserTorchCalibration> laser_torch_calibration,
                      configuration::ConfigurationHandle* laser_torch_config_handle,
                      std::optional<calibration::WeldObjectCalibration> weld_object_calibration,
                      configuration::ConfigurationHandle* weld_object_config_handle)
      : opt_laser_torch_calibration_(laser_torch_calibration),
        laser_torch_config_handle_(laser_torch_config_handle),
        opt_weld_object_calibration_(weld_object_calibration),
        weld_object_config_handle_(weld_object_config_handle) {}
  ~SliceTranslatorImpl() override = default;

  auto LPCSToMCS(const std::vector<lpcs::Point>& lpcs_points, const macs::Point& axis_position)
      -> boost::outcome_v2::result<std::vector<macs::Point>> override;
  auto CalibrateLaserToTorch(double angle, double offset, double stickout, const lpcs::Slice& scanner_data)
      -> std::optional<calibration::LaserTorchCalibration> override;
  auto CalibrateWeldObject(double radius, double stickout, const lpcs::Slice& scanner_data,
                           const macs::Point& axis_position)
      -> std::optional<calibration::WeldObjectCalibration> override;

  auto GetLaserToTorchCalibration() const -> std::optional<calibration::LaserTorchCalibration> override;
  auto SetLaserToTorchCalibration(const calibration::LaserTorchCalibration& data)
      -> boost::outcome_v2::result<void> override;
  auto GetWeldObjectCalibration() const -> std::optional<calibration::WeldObjectCalibration> override;
  auto SetWeldObjectCalibration(const calibration::WeldObjectCalibration& data)
      -> boost::outcome_v2::result<void> override;

  auto AngleFromTorchToScanner(const std::vector<lpcs::Point>& lpcs_points, const macs::Point& axis_position)
      -> boost::outcome_v2::result<double> override;

 private:
  std::optional<calibration::LaserTorchCalibration> opt_laser_torch_calibration_;
  configuration::ConfigurationHandle* laser_torch_config_handle_;
  std::optional<calibration::WeldObjectCalibration> opt_weld_object_calibration_;
  configuration::ConfigurationHandle* weld_object_config_handle_;
  static auto GetRotMatrix(double angle) -> Eigen::MatrixXd;
  auto FromLaserPlane(const Eigen::MatrixXd& slice, const macs::Point& axis_position)
      -> boost::outcome_v2::result<Eigen::MatrixXd>;
  auto RotateToTool(const Eigen::MatrixXd& slice) -> boost::outcome_v2::result<Eigen::MatrixXd>;
  static auto VectorToMatrix(const std::vector<lpcs::Point>& coord_vec) -> Eigen::MatrixXd;
  static auto MatrixToVector(Eigen::MatrixXd) -> std::vector<macs::Point>;
};
}  // namespace slice_translator
