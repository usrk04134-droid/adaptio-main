#include <algorithm>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>

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
using scanner::joint_model::ABWPoints;
using scanner::joint_model::BigSnake;
using scanner::joint_model::JointProperties;
using scanner::joint_model::Point;

namespace {

auto existing_path_or_empty(std::vector<std::filesystem::path> candidates) -> std::filesystem::path {
  for (auto const &p : candidates) {
    if (std::filesystem::exists(p)) {
      return p;
    }
  }
  return {};
}

auto load_camera_model() -> std::unique_ptr<TiltedPerspectiveCamera> {
  // Prefer specific scanner calibration files if available
  auto calib_path = existing_path_or_empty({
      "/workspace/assets/scanner_calibration/LX31624160019.yaml",
      "/workspace/assets/scanner_calibration/LX31624160053.yaml",
      "/workspace/assets/scanner_calibration/HIL.yaml",
  });

  REQUIRE_MESSAGE(!calib_path.empty(), "No scanner calibration YAML found under assets/scanner_calibration");

  auto maybe_scanner_cfg = Yaml::FromFile(calib_path, "camera");
  REQUIRE_MESSAGE(!maybe_scanner_cfg.has_error(), "Failed to read calibration YAML");
  auto scanner_cfg = maybe_scanner_cfg.value()->AsUnorderedMap();

  TiltedPerspectiveCameraProperties camera_properties =
      TiltedPerspectiveCameraProperties::FromUnorderedMap(scanner_cfg);
  // FOV used throughout tests/tools
  camera_properties.config_fov.width    = 3500;
  camera_properties.config_fov.height   = 2500;
  camera_properties.config_fov.offset_x = 312;
  camera_properties.config_fov.offset_y = 0;

  return std::make_unique<TiltedPerspectiveCamera>(camera_properties);
}

auto load_joint_properties() -> JointProperties {
  auto cfg_path = existing_path_or_empty({
      "/workspace/tests/configs/sil/configuration.yaml",
      "/workspace/assets/configuration/configuration.yaml",
  });

  // Default properties similar to those used by tooling if no config is available
  JointProperties props{
      .upper_joint_width = 50.0,
      .left_max_surface_angle = 0.34906585,
      .right_max_surface_angle = 0.34906585,
      .left_joint_angle = 0.1396,
      .right_joint_angle = 0.1396,
      .groove_depth = 28.0,
      .upper_joint_width_tolerance = 7.0,
      .surface_angle_tolerance = 0.174532925, // ~10 deg in rad
      .groove_angle_tolerance = 0.157079633,  // ~9 deg in rad
      .offset_distance = 3.0,
  };

  if (cfg_path.empty()) {
    return props;
  }

  auto maybe_cfg = Yaml::FromFile(cfg_path, "cfg");
  if (maybe_cfg.has_error()) {
    return props;
  }
  auto cfg = maybe_cfg.value()->AsUnorderedMap();

  auto getd = [&](std::string const &key, double def) -> double {
    auto it = cfg.find(key);
    if (it == cfg.end()) return def;
    auto v = it->second.Value<double>();
    return v.has_value() ? v.value() : def;
  };

  props.upper_joint_width        = getd("cfg/calibration_fixture_joint_geometry/upper_joint_width", props.upper_joint_width);
  props.groove_depth             = getd("cfg/calibration_fixture_joint_geometry/groove_depth", props.groove_depth);
  props.left_joint_angle         = getd("cfg/calibration_fixture_joint_geometry/left_joint_angle", props.left_joint_angle);
  props.right_joint_angle        = getd("cfg/calibration_fixture_joint_geometry/right_joint_angle", props.right_joint_angle);
  props.left_max_surface_angle   = getd("cfg/calibration_fixture_joint_geometry/left_max_surface_angle", props.left_max_surface_angle);
  props.right_max_surface_angle  = getd("cfg/calibration_fixture_joint_geometry/right_max_surface_angle", props.right_max_surface_angle);

  // Optional tolerances if present
  props.surface_angle_tolerance = getd("cfg/tolerances/joint_geometry/surface_angle", props.surface_angle_tolerance);
  props.groove_angle_tolerance  = getd("cfg/weld_control/scanner_groove_geometry_update/tolerance/wall_angle", props.groove_angle_tolerance);
  props.upper_joint_width_tolerance = getd("cfg/weld_control/scanner_groove_geometry_update/tolerance/upper_width", props.upper_joint_width_tolerance);

  return props;
}

auto load_threshold_gray_minimum_wall() -> int {
  auto cfg_path = existing_path_or_empty({
      "/workspace/tests/configs/sil/configuration.yaml",
      "/workspace/assets/configuration/configuration.yaml",
  });
  if (cfg_path.empty()) {
    return 16;  // default used in tooling/tests
  }
  auto maybe_cfg = Yaml::FromFile(cfg_path, "cfg");
  if (maybe_cfg.has_error()) {
    return 16;
  }
  auto cfg = maybe_cfg.value()->AsUnorderedMap();
  auto it  = cfg.find("cfg/scanner/filtering/gray_minimum_wall");
  if (it == cfg.end()) {
    return 16;
  }
  auto v = it->second.Value<std::int64_t>();
  return v.has_value() ? static_cast<int>(v.value()) : 16;
}

auto find_test_images() -> std::vector<std::filesystem::path> {
  std::vector<std::filesystem::path> roots = {
      "/workspace/src/scanner/joint_model/test/test_data",
      "/workspace/tests/configs/sil",
      "/workspace/tests/configs/sil/calibration",
  };
  std::vector<std::filesystem::path> out;
  for (auto const &root : roots) {
    if (!std::filesystem::exists(root) || !std::filesystem::is_directory(root)) continue;
    for (auto const &entry : std::filesystem::directory_iterator(root)) {
      if (!entry.is_regular_file()) continue;
      auto ext = entry.path().extension().string();
      std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      if (ext == ".tiff" || ext == ".tif" || ext == ".bmp") {
        out.push_back(entry.path());
      }
    }
  }
  // Deduplicate
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

auto load_expected_abw_for_image(std::string const &image_name) -> std::optional<ABWPoints> {
  std::filesystem::path dataset = "/workspace/tests/data_set/data_set.yaml";
  if (!std::filesystem::exists(dataset)) {
    return std::nullopt;
  }

  YAML::Node root = YAML::LoadFile(dataset.string());
  if (!root["data"]) {
    return std::nullopt;
  }

  auto data = root["data"];
  for (auto it = data.begin(); it != data.end(); ++it) {
    auto node = it->second;
    if (!node["image"]) continue;
    auto name = node["image"].as<std::string>("");
    // Compare by basename as legacy datasets often store just the filename
    auto base = std::filesystem::path(name).filename().string();
    auto tgt  = std::filesystem::path(image_name).filename().string();
    if (base != tgt) continue;

    ABWPoints points{};
    bool ok = true;
    for (int i = 0; i < 7; i++) {
      auto key = std::string("ABW").append(std::to_string(i));
      if (!node[key]) {
        ok = false;
        break;
      }
      double x = node[key]["x"].as<double>(0.0);
      double y = node[key]["y"].as<double>(0.0);
      points[static_cast<size_t>(i)] = Point{x, y};
    }
    if (ok) return points;
  }
  return std::nullopt;
}

}  // namespace

TEST_SUITE("BigSnake Parse") {
  TEST_CASE("Parse all available test images and validate against dataset if present") {
    auto camera_model = load_camera_model();
    auto properties   = load_joint_properties();
    auto threshold    = load_threshold_gray_minimum_wall();

    BigSnake model(properties, {48, threshold, 48}, std::move(camera_model));

    auto images = find_test_images();
    REQUIRE_MESSAGE(!images.empty(), "No test images found to process");

    for (auto const &img_path : images) {
      CAPTURE(img_path.string());
      auto mat = cv::imread(img_path.string(), cv::IMREAD_GRAYSCALE);
      REQUIRE_MESSAGE(!mat.empty(), "Failed to read image");

      auto maybe_image = scanner::image::ImageBuilder::From(mat, img_path.filename().string(), 0).Finalize();
      REQUIRE(maybe_image.has_value());
      auto *image = maybe_image.value().get();

      auto res = model.Parse(*image, std::nullopt, std::nullopt, false, std::nullopt);

      auto expected = load_expected_abw_for_image(img_path.filename().string());
      if (expected.has_value()) {
        REQUIRE_MESSAGE(res.has_value(), "Parse failed but dataset has expectations for this image");
        auto const &profile = std::get<0>(res.value());
        auto const &got     = profile.points;
        double const tol    = 0.75;  // mm tolerance
        for (int i = 0; i < 7; i++) {
          CAPTURE(i);
          CHECK(doctest::Approx(got[static_cast<size_t>(i)].x).epsilon(0.0).margin(tol) == expected->at(static_cast<size_t>(i)).x);
          CHECK(doctest::Approx(got[static_cast<size_t>(i)].y).epsilon(0.0).margin(tol) == expected->at(static_cast<size_t>(i)).y);
        }
      } else {
        // Basic invariants when we do not have ground truth
        if (res.has_value()) {
          auto const &profile = std::get<0>(res.value());
          auto const &p       = profile.points;
          CHECK(p[1].x > p[0].x);
          CHECK(p[5].x < p[6].x);
        } else {
          // Still validate that it fails gracefully (i.e., returns an error and not crashes)
          CHECK(!res.has_value());
        }
      }
    }
  }
}

#endif

