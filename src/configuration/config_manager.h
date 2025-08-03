#pragma once

#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <utility>

#include "common/data/data_value.h"
#include "common/tolerances/tolerances_configuration.h"
#include "conf_file_handler.h"
#include "controller/controller_configuration.h"
#include "converter.h"
#include "main/calibration/calibration_configuration.h"
#include "main/calibration/calibration_types.h"
#include "main/image_logging/image_logging_configuration.h"
#include "main/joint_geometry/joint_geometry.h"
#include "main/weld_control/weld_control_types.h"
#include "scanner/image_provider/image_provider_configuration.h"
#include "scanner/scanner_calibration_configuration.h"
#include "scanner/scanner_configuration.h"

namespace configuration {
namespace fs = std::filesystem;

const std::string TAG_CONF = "configuration";
const std::string TAG_SC   = "scanner_calibration";
const std::string TAG_CWOC = "circular_weld_object_calibration";
const std::string TAG_LTC  = "laser_torch_calibration";

class ConfigManager {
 public:
  explicit ConfigManager(const fs::path& path_scanner_calibration);
  virtual ~ConfigManager() = default;

  auto Init(const fs::path& default_config, std::optional<std::filesystem::path> cmd_line_config,
            fs::path const& path_data) -> boost::outcome_v2::result<void>;

  auto GetController() -> controller::ControllerConfigurationData { return controller_config_; }
  auto GetImageProvider() -> scanner::image_provider::ImageProviderConfigData { return image_provider_config_; }
  auto GetScannerCalibration(const std::string& scanner_serial_number)
      -> std::optional<scanner::ScannerCalibrationData>;
  auto GetScanner() -> scanner::ScannerConfigurationData { return scanner_config_; }

  auto GetCalibrationFixtureJointGeometry() -> joint_geometry::JointGeometry {
    return calibration_joint_geometry_config_;
  }
  auto GetCircWeldObjectCalib() -> std::pair<std::optional<calibration::WeldObjectCalibration>, ConfigurationHandle*>;
  auto GetLaserTorchCalib() -> std::pair<std::optional<calibration::LaserTorchCalibration>, ConfigurationHandle*>;
  auto GetWeldControlConfiguration() -> weld_control::Configuration { return weld_control_config_; }
  auto GetTolerancesConfiguration() -> tolerances::Configuration { return tolerances_config_; }
  auto GetCalibrationConfiguration() -> calibration::Configuration { return calibration_config_; }
  auto GetImageLoggingConfiguration() -> image_logging::Configuration { return image_logging_config_; };

 private:
  auto ReadConfigFiles() -> boost::outcome_v2::result<void>;

  auto ReadConfigurationFile(const fs::path& config_file, bool must_exist = false) -> boost::outcome_v2::result<void>;

  auto CheckConfigFiles() -> boost::outcome_v2::result<void>;
  void TryCopyConfigFiles(const fs::path& default_config, const fs::path& path_data);

  // key = tag, value = (yaml_file, ConverterPtr)
  std::map<std::string, std::pair<fs::path, ConverterPtr>> converters_;

  FileHandlerPtr fh_;
  controller::ControllerConfigurationData controller_config_{};
  scanner::ScannerConfigurationData scanner_config_{};
  scanner::image_provider::ImageProviderConfigData image_provider_config_{};
  joint_geometry::JointGeometry calibration_joint_geometry_config_{};
  weld_control::Configuration weld_control_config_{};
  tolerances::Configuration tolerances_config_{};
  calibration::Configuration calibration_config_{};
  image_logging::Configuration image_logging_config_{};
  fs::path path_scanner_calibration_;
};
}  // namespace configuration
