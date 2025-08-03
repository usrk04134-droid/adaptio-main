#pragma once

#include <any>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

#include "common/file/yaml.h"
#include "conf_file_handler.h"
#include "configuration/converter.h"
#include "main/calibration/calibration_types.h"
#include "main/slice_translator/slice_translator.h"

namespace configuration {

class CircWeldObjectCalibConverter : public Converter {
 public:
  CircWeldObjectCalibConverter(const std::string& tag, const std::filesystem::path& yaml_file);
  ~CircWeldObjectCalibConverter() override = default;

  auto ReadPersistentData() -> boost::outcome_v2::result<void> override;
  auto WritePersistentData(std::any config_struct) -> boost::outcome_v2::result<void> override;
  auto GetConfig() -> std::any override { return circ_weld_object_calib_; };

 private:
  std::string tag_;
  std::filesystem::path yaml_file_;
  std::unique_ptr<common::file::Yaml> yaml_;
  FileHandlerPtr fh_;

  // The struct that is mapped to a yaml file
  // It is optional since it gets values when calibration is done i.e. no default values
  std::optional<calibration::WeldObjectCalibration> circ_weld_object_calib_;
};
}  // namespace configuration
