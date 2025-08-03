#include "configuration/circ_weld_object_calib_converter.h"

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

using configuration::CircWeldObjectCalibConverter;

CircWeldObjectCalibConverter::CircWeldObjectCalibConverter(const std::string& tag,
                                                           const std::filesystem::path& yaml_file)
    : tag_(tag), yaml_file_(yaml_file) {
  fh_ = GetFactory()->CreateFileHandler();
}

auto CircWeldObjectCalibConverter::ReadPersistentData() -> boost::outcome_v2::result<void> {
  LOG_DEBUG("ReadPersistentData");
  auto yaml_string              = fh_->ReadFile(yaml_file_);
  auto maybe_configuration_yaml = common::file::Yaml::FromString(yaml_string, tag_);
  if (maybe_configuration_yaml.has_error()) {
    LOG_ERROR("Could not parse yaml string with error code: {}", maybe_configuration_yaml.error().to_string());
    return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR));
  }

  yaml_ = std::move(maybe_configuration_yaml.value());

  auto map = yaml_->AsUnorderedMap();
  try {
    calibration::WeldObjectCalibration circ_weld_object_calib = {};
    auto valid                                                = map.at(tag_ + "/valid").Value<bool>().value();
    circ_weld_object_calib.y                                  = map.at(tag_ + "/y").Value<double>().value();
    circ_weld_object_calib.z                                  = map.at(tag_ + "/z").Value<double>().value();
    circ_weld_object_calib.x_adjustment                       = map.at(tag_ + "/x_adjustment").Value<double>().value();

    if (valid) {
      circ_weld_object_calib_ = circ_weld_object_calib;
    }

  } catch (const std::exception& e) {
    LOG_ERROR("Format of configuration file is not supported: file: {} error: {}", yaml_file_.string(), e.what());
    return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR));
  }

  return boost::outcome_v2::success();
}

auto CircWeldObjectCalibConverter::WritePersistentData(std::any config_struct) -> boost::outcome_v2::result<void> {
  LOG_DEBUG("WritePersistentData");
  try {
    circ_weld_object_calib_ = std::any_cast<calibration::WeldObjectCalibration>(config_struct);
  } catch (const std::bad_any_cast& e) {
    // This should not happen
    return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_WRITE_FILE_ERROR));
  }

  std::unordered_map<std::string, common::data::DataValue> config_map;

  config_map.emplace(tag_ + "/valid", true);
  config_map.emplace(tag_ + "/y", circ_weld_object_calib_.value().y);
  config_map.emplace(tag_ + "/z", circ_weld_object_calib_.value().z);
  config_map.emplace(tag_ + "/x_adjustment", circ_weld_object_calib_.value().x_adjustment);

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
