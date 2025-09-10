#include <filesystem>
#include <cmath>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#ifndef DOCTEST_CONFIG_DISABLE
#include <doctest/doctest.h>

#include <yaml-cpp/yaml.h>

#include "common/file/yaml.h"
#include "scanner/image/image_builder.h"
#include "scanner/image/tilted_perspective_camera.h"
#include "scanner/joint_model/big_snake.h"

using common::file::Yaml;

using scanner::image::TiltedPerspectiveCamera;
using scanner::image::TiltedPerspectiveCameraProperties;
using scanner::joint_model::BigSnake;
using scanner::joint_model::JointModel;
using scanner::joint_model::JointProperties;
using scanner::joint_model::Point;

namespace fs = std::filesystem;

static auto ReadJointPropertiesFromYaml(const std::string &dataset_yaml_path) -> JointProperties {
  auto maybe_joint_yaml = Yaml::FromFile(dataset_yaml_path, "joint");
  CHECK_EQ(maybe_joint_yaml.has_error(), false);
  auto joint_map = maybe_joint_yaml.value()->AsUnorderedMap();

  JointProperties props{
      .upper_joint_width = joint_map.at("joint/upper_joint_width").Value<double>().value(),
      .left_max_surface_angle = joint_map.at("joint/left_max_surface_angle").Value<double>().value(),
      .right_max_surface_angle = joint_map.at("joint/right_max_surface_angle").Value<double>().value(),
      .left_joint_angle = joint_map.at("joint/left_joint_angle").Value<double>().value(),
      .right_joint_angle = joint_map.at("joint/right_joint_angle").Value<double>().value(),
      .groove_depth = joint_map.at("joint/groove_depth").Value<double>().value(),
      .upper_joint_width_tolerance = joint_map.at("joint/upper_joint_width_tolerance").Value<double>().value(),
      .surface_angle_tolerance = joint_map.at("joint/surface_angle_tolerance").Value<double>().value(),
      .groove_angle_tolerance = joint_map.at("joint/groove_angle_tolerance").Value<double>().value(),
      .offset_distance = joint_map.at("joint/offset_distance").Value<double>().value(),
  };
  return props;
}

static auto ReadScannerConfigFromYaml(const std::string &dataset_yaml_path) -> scanner::ScannerConfigurationData {
  auto maybe_scanner_yaml = Yaml::FromFile(dataset_yaml_path, "scanner");
  CHECK_EQ(maybe_scanner_yaml.has_error(), false);
  auto scanner_map = maybe_scanner_yaml.value()->AsUnorderedMap();

  scanner::ScannerConfigurationData cfg{
      .gray_minimum_top = static_cast<int64_t>(scanner_map.at("scanner/filtering/gray_minimum_top").Value<int64_t>().value()),
      .gray_minimum_wall = static_cast<int64_t>(scanner_map.at("scanner/filtering/gray_minimum_wall").Value<int64_t>().value()),
      .gray_minimum_bottom = static_cast<int64_t>(scanner_map.at("scanner/filtering/gray_minimum_bottom").Value<int64_t>().value()),
  };
  return cfg;
}

static auto ReadCameraFromYaml(const std::string &dataset_yaml_path) -> TiltedPerspectiveCameraProperties {
  auto maybe_camera_yaml = Yaml::FromFile(dataset_yaml_path, "camera");
  CHECK_EQ(maybe_camera_yaml.has_error(), false);
  auto camera_map = maybe_camera_yaml.value()->AsUnorderedMap();
  auto properties = TiltedPerspectiveCameraProperties::FromUnorderedMap(camera_map);

  // Optional FOV config under dataset file
  try {
    auto maybe_cfg = Yaml::FromFile(dataset_yaml_path, "image_provider");
    if (!maybe_cfg.has_error()) {
      auto map = maybe_cfg.value()->AsUnorderedMap();
      properties.config_fov.width = static_cast<int>(map.at("image_provider/fov/width").Value<int64_t>().value());
      properties.config_fov.height = static_cast<int>(map.at("image_provider/fov/height").Value<int64_t>().value());
      properties.config_fov.offset_x = static_cast<int>(map.at("image_provider/fov/offset_x").Value<int64_t>().value());
      properties.config_fov.offset_y = static_cast<int>(map.at("image_provider/fov/offset_y").Value<int64_t>().value());
    }
  } catch (...) {
    // Keep defaults if not provided
  }

  return properties;
}

static auto ReadImagesBaseDir(const YAML::Node &root) -> fs::path {
  if (root["images_dir"]) {
    return fs::path(root["images_dir"].as<std::string>());
  }
  // Default to images subfolder under test_data
  return fs::path("./src/scanner/joint_model/test/test_data/images");
}

static auto NodeHasExpectedAbw(const YAML::Node &node) -> bool {
  return node["expected"] && node["expected"]["abw"] && node["expected"]["abw"].IsSequence() &&
         node["expected"]["abw"].size() == 7;
}

TEST_SUITE("BigSnake") {
  TEST_CASE("Parse dataset images and optionally validate ABW points") {
    const std::string dataset_yaml_path = "./src/scanner/joint_model/test/test_data/data_set.yaml";
    CHECK_MESSAGE(fs::exists(dataset_yaml_path), "Dataset YAML not found at " << dataset_yaml_path);

    auto camera_properties = ReadCameraFromYaml(dataset_yaml_path);
    auto scanner_cfg = ReadScannerConfigFromYaml(dataset_yaml_path);
    auto joint_properties = ReadJointPropertiesFromYaml(dataset_yaml_path);

    YAML::Node doc = YAML::LoadFile(dataset_yaml_path);
    REQUIRE_MESSAGE(doc.IsDefined(), "Failed to parse dataset YAML");
    auto base_dir = ReadImagesBaseDir(doc);

    REQUIRE_MESSAGE(doc["dataset"].IsDefined(), "Missing 'dataset' in dataset YAML");
    REQUIRE_MESSAGE(doc["dataset"].IsSequence(), "'dataset' must be a sequence in dataset YAML");
    auto dataset = doc["dataset"];

    for (std::size_t i = 0; i < dataset.size(); ++i) {
      auto item = dataset[i];
      REQUIRE_MESSAGE(item["image"], "Dataset entry missing 'image' field at index " << i);
      auto image_file = item["image"].as<std::string>();
      fs::path image_path = base_dir / image_file;
      CHECK_MESSAGE(fs::exists(image_path), "Image not found: " << image_path.string());

      auto camera_model = std::make_unique<TiltedPerspectiveCamera>(camera_properties);
      BigSnake big_snake(joint_properties, scanner_cfg, std::move(camera_model));

      auto image_builder = scanner::image::ImageBuilder::From(image_path);
      auto maybe_image = image_builder.Finalize();
      REQUIRE_MESSAGE(maybe_image.has_error() == false, "Failed to open image: " << image_path.string());
      auto image = std::move(maybe_image.value());

      auto result = big_snake.Parse(*image, std::nullopt, std::nullopt, false, std::nullopt);
      REQUIRE_MESSAGE(result.has_value(), "BigSnake parsing failed for image: " << image_path.string());

      if (NodeHasExpectedAbw(item)) {
        const auto& result_tuple = result.value();
        const auto& profile = std::get<0>(result_tuple);
        auto expected_abw = item["expected"]["abw"];
        // Allow small numeric tolerance (meters)
        const double tol_x = (item["expected"]["tolerance_x"]) ? item["expected"]["tolerance_x"].as<double>() : 1e-3;
        const double tol_y = (item["expected"]["tolerance_y"]) ? item["expected"]["tolerance_y"].as<double>() : 1e-3;

        for (std::size_t p = 0; p < 7; ++p) {
          auto exp_point = expected_abw[p];
          double ex = exp_point["x"].as<double>();
          double ey = exp_point["y"].as<double>();
          CHECK_LE(std::fabs(profile.points[p].x - ex), tol_x);
          CHECK_LE(std::fabs(profile.points[p].y - ey), tol_y);
        }
      }
    }
  }
}
#endif

