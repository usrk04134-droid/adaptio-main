#include "scanner/joint_model/big_snake.h"

#include <doctest/doctest.h>

#include <filesystem>
#include <cstdlib>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "common/file/yaml.h"
#include "common/logging/application_log.h"
#include "scanner/image/image_builder.h"
#include "scanner/image/tilted_perspective_camera.h"
#include "scanner/scanner_configuration.h"

using scanner::image::CameraModelPtr;
using scanner::image::TiltedPerspectiveCamera;
using scanner::image::TiltedPerspectiveCameraProperties;
using scanner::joint_model::ABWPoints;
using scanner::joint_model::BigSnake;
using scanner::joint_model::JointModelErrorCode;
using scanner::joint_model::JointProfile;
using scanner::joint_model::JointProperties;

namespace {

struct DataSetEntry {
  std::string image_path;
  std::optional<ABWPoints> expected_points;
};

struct DataSetHeader {
  std::string camera_calibration_path;
  scanner::image_provider::Fov fov{.width = 3500, .height = 2500, .offset_x = 312, .offset_y = 0};
  JointProperties joint_properties{};
  scanner::ScannerConfigurationData scanner_filtering{48, 16, 48};
  double abw_xy_tolerance{1e-3};
};

auto LoadCameraProperties(const std::string& calibration_yaml_path, const scanner::image_provider::Fov& fov)
    -> TiltedPerspectiveCameraProperties {
  auto maybe_yaml = common::file::Yaml::FromFile(calibration_yaml_path, "camera");
  REQUIRE_MESSAGE(maybe_yaml.has_value(), "Failed to read calibration: " << calibration_yaml_path);
  auto map = maybe_yaml.value()->AsUnorderedMap();
  auto props = TiltedPerspectiveCameraProperties::FromUnorderedMap(map);
  auto properties_with_fov = props;
  properties_with_fov.config_fov = fov;
  return properties_with_fov;
}

auto ParseJointPropertiesFromYaml(const YAML::Node& node) -> JointProperties {
  JointProperties p{};
  p.upper_joint_width           = node["upper_joint_width"].as<double>();
  p.left_max_surface_angle      = node["left_max_surface_angle"].as<double>();
  p.right_max_surface_angle     = node["right_max_surface_angle"].as<double>();
  p.left_joint_angle            = node["left_joint_angle"].as<double>();
  p.right_joint_angle           = node["right_joint_angle"].as<double>();
  p.groove_depth                = node["groove_depth"].as<double>();
  p.upper_joint_width_tolerance = node["upper_joint_width_tolerance"].as<double>();
  p.surface_angle_tolerance     = node["surface_angle_tolerance"].as<double>();
  p.groove_angle_tolerance      = node["groove_angle_tolerance"].as<double>();
  p.offset_distance             = node["offset_distance"].as<double>();
  return p;
}

auto TryParseABWPoints(const YAML::Node& node) -> std::optional<ABWPoints> {
  if (!node || !node.IsSequence() || node.size() != 7) {
    return std::nullopt;
  }
  ABWPoints points{};
  for (size_t i = 0; i < 7; i++) {
    points[i].x = node[i]["x"].as<double>();
    points[i].y = node[i]["y"].as<double>();
  }
  return points;
}

auto LoadDataSet(const std::string& dataset_yaml_path)
    -> std::pair<DataSetHeader, std::vector<DataSetEntry>> {
  YAML::Node root = YAML::LoadFile(dataset_yaml_path);

  DataSetHeader header{};
  if (root["header"]) {
    const auto h = root["header"];
    if (h["camera_calibration"]) {
      header.camera_calibration_path = h["camera_calibration"].as<std::string>();
    }
    if (h["fov"]) {
      const auto f = h["fov"];
      header.fov.width    = f["width"].as<int>();
      header.fov.height   = f["height"].as<int>();
      header.fov.offset_x = f["offset_x"].as<int>();
      header.fov.offset_y = f["offset_y"].as<int>();
    }
    if (h["joint"]) {
      header.joint_properties = ParseJointPropertiesFromYaml(h["joint"]);
    }
    if (h["scanner_filtering"]) {
      const auto s = h["scanner_filtering"];
      header.scanner_filtering.gray_minimum_top    = s["gray_minimum_top"].as<int64_t>();
      header.scanner_filtering.gray_minimum_wall   = s["gray_minimum_wall"].as<int64_t>();
      header.scanner_filtering.gray_minimum_bottom = s["gray_minimum_bottom"].as<int64_t>();
    }
    if (h["tolerance"]) {
      if (h["tolerance"]["abw_xy"]) {
        header.abw_xy_tolerance = h["tolerance"]["abw_xy"].as<double>();
      }
    }
  }

  std::vector<DataSetEntry> entries;
  if (root["data"]) {
    for (const auto& n : root["data"]) {
      DataSetEntry e;
      e.image_path = n["image"].as<std::string>();
      e.expected_points = TryParseABWPoints(n["expected"]["ABWPoints"]);
      entries.push_back(e);
    }
  }

  return {header, entries};
}

}  // namespace

TEST_SUITE("BigSnake Parse - Data Set") {
  TEST_CASE("Parse images and optionally validate ABW points against annotations") {
    // Default path can be overridden by env var ADAPTIO_BIGSNAKE_DATASET
    const char* env_path = std::getenv("ADAPTIO_BIGSNAKE_DATASET");
    const std::string dataset_path = env_path ? std::string(env_path)
                                              : std::string("./src/scanner/joint_model/test/test_data/big_snake_data_set.yaml");

    if (!std::filesystem::exists(dataset_path)) {
      MESSAGE("Dataset YAML not found - skipping dataset-driven BigSnake::Parse test. Path: " << dataset_path);
      return;  // gracefully skip when dataset isn't available in the checkout
    }

    auto [header, entries] = LoadDataSet(dataset_path);

    REQUIRE_MESSAGE(!header.camera_calibration_path.empty(), "Camera calibration path missing in dataset header");
    REQUIRE_MESSAGE(std::filesystem::exists(header.camera_calibration_path),
                    "Camera calibration YAML not found: " << header.camera_calibration_path);

    auto camera_props = LoadCameraProperties(header.camera_calibration_path, header.fov);

    // Build camera model and joint model for tests
    auto camera_model = std::make_unique<TiltedPerspectiveCamera>(camera_props);
    BigSnake model(header.joint_properties, header.scanner_filtering, std::move(camera_model));

    for (const auto& entry : entries) {
      CAPTURE(entry.image_path);
      REQUIRE_MESSAGE(std::filesystem::exists(entry.image_path), "Image not found: " << entry.image_path);

      auto gray = cv::imread(entry.image_path, cv::IMREAD_GRAYSCALE);
      REQUIRE_MESSAGE(!gray.empty(), "Failed to load image: " << entry.image_path);

      auto maybe_image = scanner::image::ImageBuilder::From(gray, entry.image_path, 0).Finalize();
      REQUIRE_MESSAGE(maybe_image.has_value(), "Failed to build Image object from: " << entry.image_path);
      auto* image = maybe_image.value().get();

      auto result = model.Parse(*image, std::nullopt, std::nullopt, false, std::nullopt);
      REQUIRE_MESSAGE(result.has_value(), "BigSnake::Parse failed with error code: "
                                             << static_cast<uint32_t>(result.error()));

      const auto& profile = std::get<0>(result.value());

      // Basic sanity checks
      CHECK(profile.points[0].x < profile.points[1].x);
      CHECK(profile.points[1].x < profile.points[2].x);
      CHECK(profile.points[2].x < profile.points[3].x);
      CHECK(profile.points[3].x < profile.points[4].x);
      CHECK(profile.points[4].x < profile.points[5].x);
      CHECK(profile.points[5].x < profile.points[6].x);

      if (entry.expected_points.has_value()) {
        const auto& exp = entry.expected_points.value();
        for (int i = 0; i < 7; i++) {
          CAPTURE(i);
          CHECK(profile.points[i].x == doctest::Approx(exp[i].x).epsilon(0).scale(header.abw_xy_tolerance));
          CHECK(profile.points[i].y == doctest::Approx(exp[i].y).epsilon(0).scale(header.abw_xy_tolerance));
        }
      }
    }
  }
}

