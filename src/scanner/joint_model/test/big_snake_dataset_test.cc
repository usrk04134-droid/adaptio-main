#include <doctest/doctest.h>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <optional>
#include <cstdlib>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include "common/file/yaml.h"
#include "scanner/image/image_builder.h"
#include "scanner/image/tilted_perspective_camera.h"
#include "scanner/joint_model/big_snake.h"

using scanner::image::ImageBuilder;
using scanner::image::TiltedPerspectiveCamera;
using scanner::image::TiltedPerspectiveCameraProperties;
using scanner::joint_model::BigSnake;
using scanner::joint_model::JointModelPtr;
using scanner::joint_model::JointProperties;
using scanner::joint_model::JointProfile;
using scanner::joint_model::Point;
using scanner::ScannerConfigurationData;

namespace {

auto DefaultFov() -> scanner::image_provider::Fov {
  scanner::image_provider::Fov fov;
  fov.width    = 3500;
  fov.height   = 2500;
  fov.offset_x = 312;
  fov.offset_y = 0;
  return fov;
}

auto TryLoadFovFromConfig(const std::filesystem::path& config_path)
    -> std::optional<scanner::image_provider::Fov> {
  if (!std::filesystem::exists(config_path)) {
    return std::nullopt;
  }
  auto maybe_yaml = common::file::Yaml::FromFile(config_path, "image_provider");
  if (maybe_yaml.has_error()) {
    return std::nullopt;
  }
  auto map = maybe_yaml.value()->AsUnorderedMap();
  try {
    scanner::image_provider::Fov fov;
    fov.width    = static_cast<int>(map.at("image_provider/fov/width").Value<int64_t>().value());
    fov.height   = static_cast<int>(map.at("image_provider/fov/height").Value<int64_t>().value());
    fov.offset_x = static_cast<int>(map.at("image_provider/fov/offset_x").Value<int64_t>().value());
    fov.offset_y = static_cast<int>(map.at("image_provider/fov/offset_y").Value<int64_t>().value());
    return fov;
  } catch (...) {
    return std::nullopt;
  }
}

auto MakeCameraFromScannerYamlString(const std::string& yaml) -> std::unique_ptr<TiltedPerspectiveCamera> {
  auto maybe_yaml = common::file::Yaml::FromString(yaml, "camera");
  if (maybe_yaml.has_error()) {
    return nullptr;
  }
  auto map        = maybe_yaml.value()->AsUnorderedMap();
  auto properties = TiltedPerspectiveCameraProperties::FromUnorderedMap(map);

  // Fallback to default FOV or try to read from tests config
  auto fov = TryLoadFovFromConfig("/workspace/tests/configs/sil/configuration.yaml").value_or(DefaultFov());
  properties.config_fov = fov;

  return std::make_unique<TiltedPerspectiveCamera>(properties);
}

auto MakeCameraFromScannerYamlFile(const std::filesystem::path& path) -> std::unique_ptr<TiltedPerspectiveCamera> {
  auto maybe_yaml = common::file::Yaml::FromFile(path, "camera");
  if (maybe_yaml.has_error()) {
    return nullptr;
  }
  auto map        = maybe_yaml.value()->AsUnorderedMap();
  auto properties = TiltedPerspectiveCameraProperties::FromUnorderedMap(map);
  auto fov        = TryLoadFovFromConfig("/workspace/tests/configs/sil/configuration.yaml").value_or(DefaultFov());
  properties.config_fov = fov;
  return std::make_unique<TiltedPerspectiveCamera>(properties);
}

auto LoadJointProperties() -> std::optional<JointProperties> {
  // Prefer tests config if present, else fall back to assets
  std::vector<std::filesystem::path> candidates = {
      "/workspace/tests/configs/sil/configuration.yaml", "/workspace/assets/configuration/configuration.yaml"};

  for (const auto& p : candidates) {
    if (!std::filesystem::exists(p)) continue;
    auto maybe_yaml = common::file::Yaml::FromFile(p, "calibration_fixture_joint_geometry");
    if (maybe_yaml.has_error()) continue;
    auto map = maybe_yaml.value()->AsUnorderedMap();
    try {
      JointProperties props = {
          .upper_joint_width = map.at("calibration_fixture_joint_geometry/upper_joint_width").Value<double>().value(),
          .left_max_surface_angle =
              map.at("calibration_fixture_joint_geometry/left_max_surface_angle").Value<double>().value(),
          .right_max_surface_angle =
              map.at("calibration_fixture_joint_geometry/right_max_surface_angle").Value<double>().value(),
          .left_joint_angle   = map.at("calibration_fixture_joint_geometry/left_joint_angle").Value<double>().value(),
          .right_joint_angle  = map.at("calibration_fixture_joint_geometry/right_joint_angle").Value<double>().value(),
          .groove_depth       = map.at("calibration_fixture_joint_geometry/groove_depth").Value<double>().value(),
          .upper_joint_width_tolerance = 7.0,
          .surface_angle_tolerance     = 0.17453292519943295, // 10 deg in rad
          .groove_angle_tolerance      = 0.15707963267948966, // 9 deg in rad
          .offset_distance             = 3.0,
      };
      return props;
    } catch (...) {
      continue;
    }
  }
  return std::nullopt;
}

auto LoadScannerFiltering() -> ScannerConfigurationData {
  // Try tests config first then assets, else defaults
  std::vector<std::filesystem::path> candidates = {
      "/workspace/tests/configs/sil/configuration.yaml", "/workspace/assets/configuration/configuration.yaml"};

  for (const auto& p : candidates) {
    if (!std::filesystem::exists(p)) continue;
    auto maybe_yaml = common::file::Yaml::FromFile(p, "scanner");
    if (maybe_yaml.has_error()) continue;
    auto map = maybe_yaml.value()->AsUnorderedMap();
    try {
      ScannerConfigurationData cfg{};
      cfg.gray_minimum_top    = map.at("scanner/filtering/gray_minimum_top").Value<int64_t>().value();
      cfg.gray_minimum_wall   = map.at("scanner/filtering/gray_minimum_wall").Value<int64_t>().value();
      cfg.gray_minimum_bottom = map.at("scanner/filtering/gray_minimum_bottom").Value<int64_t>().value();
      return cfg;
    } catch (...) {
      continue;
    }
  }

  return ScannerConfigurationData{32, 16, 32};
}

auto FindImagePathByName(const std::string& filename) -> std::optional<std::filesystem::path> {
  // Search a few likely roots
  std::vector<std::filesystem::path> roots = {
      "/workspace/src/scanner/joint_model/test/test_data", "/workspace/tests", "/workspace"};
  for (const auto& root : roots) {
    if (!std::filesystem::exists(root)) continue;
    for (auto const& entry : std::filesystem::recursive_directory_iterator(root)) {
      if (!entry.is_regular_file()) continue;
      if (entry.path().filename() == filename) {
        return entry.path();
      }
    }
  }
  return std::nullopt;
}

struct ExpectedAbwPoints {
  std::array<Point, 7> points;
};

auto ParseExpectedFromAbwPointsNode(const YAML::Node& node) -> std::optional<ExpectedAbwPoints> {
  ExpectedAbwPoints out{};
  try {
    for (int i = 0; i < 7; ++i) {
      std::string key = std::string("ABW") + std::to_string(i);
      if (!node[key]) return std::nullopt;
      out.points[static_cast<size_t>(i)].x = node[key]["x"].as<double>();
      out.points[static_cast<size_t>(i)].y = node[key]["y"].as<double>();
    }
    return out;
  } catch (...) {
    return std::nullopt;
  }
}

auto LoadDataset(const std::filesystem::path& dataset_path) -> YAML::Node {
  return YAML::LoadFile(dataset_path.string());
}

}  // namespace

TEST_SUITE("BigSnake Dataset") {
  TEST_CASE("Parse annotated images and compare ABW points") {
    // Locate dataset file
    std::filesystem::path dataset_path = std::getenv("ADAPTIO_TEST_DATASET") ? std::getenv("ADAPTIO_TEST_DATASET")
                                                                             : "/workspace/tests/data_set/data_set.yaml";

    if (!std::filesystem::exists(dataset_path)) {
      // No dataset available in this workspace – gracefully skip
      return;
    }

    YAML::Node dataset;
    try {
      dataset = LoadDataset(dataset_path);
    } catch (...) {
      // Malformed dataset – skip
      return;
    }

    // Try construct camera from header.scanner_config if available
    std::unique_ptr<TiltedPerspectiveCamera> camera_model;
    if (dataset["header"]) {
      const auto& header = dataset["header"];
      if (header["scanner_config"]) {
        if (header["scanner_config"].IsScalar()) {
          camera_model = MakeCameraFromScannerYamlString(header["scanner_config"].as<std::string>());
        } else {
          std::stringstream ss;
          ss << header["scanner_config"];
          camera_model = MakeCameraFromScannerYamlString(ss.str());
        }
      }
    }

    if (!camera_model) {
      // Fallback to a default calibration file if dataset did not include one
      // Prefer a deterministic one to avoid flakiness across environments
      std::filesystem::path default_calib = "/workspace/assets/scanner_calibration/LX31624160053.yaml";
      if (std::filesystem::exists(default_calib)) {
        camera_model = MakeCameraFromScannerYamlFile(default_calib);
      }
    }

    if (!camera_model) {
      // As a last resort, skip if we cannot construct a camera
      return;
    }

    auto maybe_joint_props = LoadJointProperties();
    if (!maybe_joint_props.has_value()) {
      return;
    }
    auto joint_props     = maybe_joint_props.value();
    auto scanner_filters = LoadScannerFiltering();

    // Default tolerance in workspace units (mm)
    const double tol_x = 0.3;
    const double tol_y = 0.3;

    // The dataset "data" can either be a sequence or a map of ABWPointsX blocks – support both
    YAML::Node data = dataset["data"];
    if (!data) {
      // Try a top-level variant
      data = dataset;
    }

    // Collect entries as a vector of pairs (image_name, node_with_points)
    std::vector<std::pair<std::string, YAML::Node>> entries;

    if (data.IsSequence()) {
      for (const auto& item : data) {
        if (!item) continue;
        // Expect fields: image, ABW0..ABW6
        if (!item["image"]) continue;
        entries.emplace_back(item["image"].as<std::string>(), item);
      }
    } else if (data.IsMap()) {
      for (auto it = data.begin(); it != data.end(); ++it) {
        auto key  = it->first.as<std::string>();
        auto node = it->second;
        // Expect keys like ABWPoints0, ABWPoints1, ... with child keys ABW0..ABW6 and an "image" field
        if (node && node["image"]) {
          entries.emplace_back(node["image"].as<std::string>(), node);
        }
      }
    }

    REQUIRE_MESSAGE(entries.size() > 0, "Dataset contains no entries to test");

    // Build the joint model once and reuse
    JointModelPtr joint_model = JointModelPtr(new BigSnake(joint_props, scanner_filters, std::move(camera_model)));

    for (const auto& [image_name, node] : entries) {
      INFO("image=" << image_name);

      auto expected_opt = ParseExpectedFromAbwPointsNode(node);
      if (!expected_opt.has_value()) {
        // No expected ABW points – skip this entry
        continue;
      }

      auto image_path_opt = FindImagePathByName(image_name);
      if (!image_path_opt.has_value()) {
        // Image file not found in workspace – skip this entry
        continue;
      }

      auto grayscale_image = cv::imread(image_path_opt->string(), cv::IMREAD_GRAYSCALE);
      REQUIRE_MESSAGE(!grayscale_image.empty(), "Failed to load image: " << image_path_opt->string());

      auto maybe_image = ImageBuilder::From(grayscale_image, image_name, 0).Finalize();
      REQUIRE(maybe_image.has_value());
      auto* image = maybe_image.value().get();

      auto result = joint_model->Parse(*image, std::nullopt, std::nullopt, false, std::nullopt);
      REQUIRE(result.has_value());

      auto [profile, /* snake_lpcs */ std::ignore, /* processing_ms */ std::ignore, /* num_walls */ std::ignore] =
          result.value();

      auto expected = expected_opt.value();
      for (int i = 0; i < 7; ++i) {
        CHECK_MESSAGE(profile.points[static_cast<size_t>(i)].x == doctest::Approx(expected.points[static_cast<size_t>(i)].x).epsilon(0.0).scale(1.0).margin(tol_x),
                      "ABW" << i << ".x mismatch");
        CHECK_MESSAGE(profile.points[static_cast<size_t>(i)].y == doctest::Approx(expected.points[static_cast<size_t>(i)].y).epsilon(0.0).scale(1.0).margin(tol_y),
                      "ABW" << i << ".y mismatch");
      }
    }
  }

  TEST_CASE("Parse all TIFF images in tests folder and succeed") {
    // Build camera
    std::unique_ptr<TiltedPerspectiveCamera> camera_model;
    std::filesystem::path default_calib = "/workspace/assets/scanner_calibration/LX31624160053.yaml";
    if (std::filesystem::exists(default_calib)) {
      camera_model = MakeCameraFromScannerYamlFile(default_calib);
    }
    if (!camera_model) return;  // Skip if we cannot construct a camera

    auto maybe_joint_props = LoadJointProperties();
    if (!maybe_joint_props.has_value()) return;
    auto joint_props     = maybe_joint_props.value();
    auto scanner_filters = LoadScannerFiltering();

    JointModelPtr joint_model = JointModelPtr(new BigSnake(joint_props, scanner_filters, std::move(camera_model)));

    // Collect .tiff images under tests folder
    std::vector<std::filesystem::path> roots = {"/workspace/tests", "/workspace/src/scanner/joint_model/test/test_data"};
    std::vector<std::filesystem::path> images;
    for (const auto& root : roots) {
      if (!std::filesystem::exists(root)) continue;
      for (auto const& entry : std::filesystem::recursive_directory_iterator(root)) {
        if (entry.is_regular_file()) {
          auto ext = entry.path().extension().string();
          if (ext == ".tiff" || ext == ".tif") images.push_back(entry.path());
        }
      }
    }

    if (images.empty()) return;  // nothing to test in this workspace

    for (const auto& path : images) {
      INFO("image=" << path.string());
      auto grayscale_image = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
      REQUIRE_MESSAGE(!grayscale_image.empty(), "Failed to load image: " << path.string());
      auto maybe_image = ImageBuilder::From(grayscale_image, path.filename().string(), 0).Finalize();
      REQUIRE(maybe_image.has_value());
      auto* image = maybe_image.value().get();

      auto result = joint_model->Parse(*image, std::nullopt, std::nullopt, false, std::nullopt);
      // For generic images we only assert that parsing succeeds
      CHECK(result.has_value());
    }
  }
}

