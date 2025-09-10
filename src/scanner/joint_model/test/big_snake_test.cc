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
  try {
    auto maybe_joint_yaml = Yaml::FromFile(dataset_yaml_path, "joint");
    if (maybe_joint_yaml.has_error()) {
      throw std::runtime_error("joint yaml missing");
    }
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
  } catch (...) {
    // Fallback defaults
    JointProperties props{
        .upper_joint_width = 30.0,
        .left_max_surface_angle = 0.34906585,
        .right_max_surface_angle = 0.34906585,
        .left_joint_angle = 0.5235987755982988,
        .right_joint_angle = 0.5235987755982988,
        .groove_depth = 12.0,
        .upper_joint_width_tolerance = 7.0,
        .surface_angle_tolerance = 10.0 * 2.0 * M_PI / 360.0,
        .groove_angle_tolerance = 10.0 * 2.0 * M_PI / 360.0,
        .offset_distance = 3.0,
    };
    return props;
  }
}

static auto ReadScannerConfigFromYaml(const std::string &dataset_yaml_path) -> scanner::ScannerConfigurationData {
  try {
    auto maybe_scanner_yaml = Yaml::FromFile(dataset_yaml_path, "scanner");
    if (maybe_scanner_yaml.has_error()) {
      throw std::runtime_error("scanner yaml missing");
    }
    auto scanner_map = maybe_scanner_yaml.value()->AsUnorderedMap();

    scanner::ScannerConfigurationData cfg{
        .gray_minimum_top = static_cast<int64_t>(scanner_map.at("scanner/filtering/gray_minimum_top").Value<int64_t>().value()),
        .gray_minimum_wall = static_cast<int64_t>(scanner_map.at("scanner/filtering/gray_minimum_wall").Value<int64_t>().value()),
        .gray_minimum_bottom = static_cast<int64_t>(scanner_map.at("scanner/filtering/gray_minimum_bottom").Value<int64_t>().value()),
    };
    return cfg;
  } catch (...) {
    return scanner::ScannerConfigurationData{.gray_minimum_top = 32, .gray_minimum_wall = 16, .gray_minimum_bottom = 32};
  }
}

static auto ReadCameraFromYaml(const std::string &dataset_yaml_path) -> TiltedPerspectiveCameraProperties {
  try {
    auto maybe_camera_yaml = Yaml::FromFile(dataset_yaml_path, "camera");
    if (maybe_camera_yaml.has_error()) {
      throw std::runtime_error("camera yaml missing");
    }
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
  } catch (...) {
    // Fallback to reasonable defaults used in other tests
    TiltedPerspectiveCameraProperties p{};
    p.config_calib.intrinsic.rho               = 3.141447305679321289e+00;
    p.config_calib.intrinsic.tau               = 1.645262539386749268e-01;
    p.config_calib.intrinsic.scaling_factors.m = 0.1;
    p.config_calib.intrinsic.scaling_factors.w = 0.007093;
    p.config_calib.intrinsic.d                 = 5.489320311280183606e-01;
    p.config_calib.intrinsic.focus_distance    = 3.744565963745117188e+00;
    p.config_calib.intrinsic.principal_point.x = 9.869480729103088379e-01;
    p.config_calib.intrinsic.principal_point.y = 7.230033874511718750e-01;
    p.config_calib.intrinsic.pixel_pitch.x     = 2.74e-06;
    p.config_calib.intrinsic.pixel_pitch.y     = 2.74e-06;
    p.config_calib.intrinsic.K1                = 3.780014812946319580e-03;
    p.config_calib.intrinsic.K2                = -1.993117621168494225e-03;
    p.config_calib.intrinsic.K3                = 5.228068857832113281e-07;
    p.config_calib.intrinsic.P1                = -1.876385213108733296e-04;
    p.config_calib.intrinsic.P2                = -5.847600405104458332e-04;
    p.config_calib.extrinsic.rotation.setZero();
    // Reasonable rotation/translation (identity-like)
    p.config_calib.extrinsic.rotation(0, 0) = 9.999974673412257431e-01;
    p.config_calib.extrinsic.rotation(0, 1) = 2.039705193809659024e-03;
    p.config_calib.extrinsic.rotation(0, 2) = 9.512696023625968975e-04;
    p.config_calib.extrinsic.rotation(1, 0) = 0.0;
    p.config_calib.extrinsic.rotation(1, 1) = 4.226691551490259768e-01;
    p.config_calib.extrinsic.rotation(1, 2) = -9.062840533108859065e-01;
    p.config_calib.extrinsic.rotation(2, 0) = -2.250624609754632317e-03;
    p.config_calib.extrinsic.rotation(2, 1) = 9.062817580026263364e-01;
    p.config_calib.extrinsic.rotation(2, 2) = 4.226680846722816187e-01;
    p.config_calib.extrinsic.translation << 0.0, 0.0, 4.087606157143235386e-01;
    p.config_fov.width   = 3500;
    p.config_fov.height  = 2500;
    p.config_fov.offset_x = 312;
    p.config_fov.offset_y = 0;
    return p;
  }
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
      // Allow per-image override of gray_minimum_wall
      auto scanner_cfg_local = scanner_cfg;
      try {
        if (item["scanner"] && item["scanner"]["filtering"] && item["scanner"]["filtering"]["gray_minimum_wall"]) {
          scanner_cfg_local.gray_minimum_wall = item["scanner"]["filtering"]["gray_minimum_wall"].as<int64_t>();
        }
      } catch (...) {
        // keep defaults
      }
      BigSnake big_snake(joint_properties, scanner_cfg_local, std::move(camera_model));

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

