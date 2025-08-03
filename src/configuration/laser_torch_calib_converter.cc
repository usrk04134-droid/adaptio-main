#include "configuration/laser_torch_calib_converter.h"

#include <any>
#include <exception>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

#include "common/data/data_value.h"
#include "common/file/yaml.h"
#include "common/logging/application_log.h"
#include "conf_factory.h"
#include "configuration_error.h"
#include "main/calibration/calibration_types.h"

using configuration::LaserTorchCalibConverter;

LaserTorchCalibConverter::LaserTorchCalibConverter(const std::string& tag, const std::filesystem::path& yaml_file)
    : tag_(tag), yaml_file_(yaml_file) {
  fh_ = GetFactory()->CreateFileHandler();
}

auto LaserTorchCalibConverter::ReadPersistentData() -> boost::outcome_v2::result<void> {
  LOG_DEBUG("ReadPersistentData");
  auto yaml_string              = fh_->ReadFile(yaml_file_);
  auto maybe_configuration_yaml = common::file::Yaml::FromString(yaml_string, tag_);
  if (maybe_configuration_yaml.has_error()) {
    LOG_ERROR("Could not parse yaml string with error code: {}", maybe_configuration_yaml.error().to_string());
    return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR));
  }

  yaml_    = std::move(maybe_configuration_yaml.value());
  auto map = yaml_->AsUnorderedMap();

  try {
    calibration::LaserTorchCalibration laser_torch_calib = {};
    auto valid                                           = map.at(tag_ + "/valid").Value<bool>().value();

    laser_torch_calib.x     = map.at(tag_ + "/x").Value<double>().value();
    laser_torch_calib.y     = map.at(tag_ + "/y").Value<double>().value();
    laser_torch_calib.z     = map.at(tag_ + "/z").Value<double>().value();
    laser_torch_calib.angle = map.at(tag_ + "/angle").Value<double>().value();

    if (valid) {
      laser_torch_calib_ = laser_torch_calib;
    }
  } catch (const std::exception& e) {
    LOG_ERROR("Format of configuration file is not supported: file: {} error: {}", yaml_file_.string(), e.what());
    return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR));
  }

  return boost::outcome_v2::success();
}

auto LaserTorchCalibConverter::WritePersistentData(std::any config_struct) -> boost::outcome_v2::result<void> {
  LOG_DEBUG("WritePersistentData");
  try {
    laser_torch_calib_ = std::any_cast<calibration::LaserTorchCalibration>(config_struct);
  } catch (const std::bad_any_cast& e) {
    // This should happen. Should return error
    LOG_ERROR("Unexpected fault. Cast failed");
    return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_WRITE_FILE_ERROR));
  }

  std::unordered_map<std::string, common::data::DataValue> config_map;

  config_map.emplace(tag_ + "/valid", true);
  config_map.emplace(tag_ + "/x", laser_torch_calib_.value().x);
  config_map.emplace(tag_ + "/y", laser_torch_calib_.value().y);
  config_map.emplace(tag_ + "/z", laser_torch_calib_.value().z);
  config_map.emplace(tag_ + "/angle", laser_torch_calib_.value().angle);

  auto maybe_yaml_string = yaml_->ToString(config_map);

  if (maybe_yaml_string.has_error()) {
    LOG_ERROR("Could not parse to yaml string with error code: {}", maybe_yaml_string.error().to_string());
    return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_WRITE_FILE_ERROR));
  }

  if (!fh_->WriteFile(yaml_file_, maybe_yaml_string.value())) {
    LOG_ERROR("Could not store configuration: {}", yaml_file_.string());
    return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_WRITE_FILE_ERROR));
  }

  return boost::outcome_v2::success();
}
