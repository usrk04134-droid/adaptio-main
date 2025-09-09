#include <optional>

#include "scanner/image/image_builder.h"
#include "scanner/image/tilted_perspective_camera.h"
#include "scanner/joint_model/big_snake.h"
#include "common/file/yaml.h"

#ifndef DOCTEST_CONFIG_DISABLE
#include <doctest/doctest.h>

using common::file::Yaml;
using scanner::image::TiltedPerspectiveCamera;
using scanner::image::TiltedPerspectiveCameraProperties;
using scanner::joint_model::BigSnake;
using scanner::joint_model::JointModelErrorCode;
using scanner::joint_model::JointProperties;

static auto LoadCameraFromYaml(const std::filesystem::path &calib_yaml_path,
                               const scanner::image_provider::Fov &fov)
    -> std::unique_ptr<TiltedPerspectiveCamera> {
  auto maybe_calib = Yaml::FromFile(calib_yaml_path, "camera");
  REQUIRE(maybe_calib);
  auto calib_map = maybe_calib.value()->AsUnorderedMap();

  TiltedPerspectiveCameraProperties props = TiltedPerspectiveCameraProperties::FromUnorderedMap(calib_map);
  props.config_fov = fov;
  return std::make_unique<TiltedPerspectiveCamera>(props);
}

static auto LoadJointPropertiesFromYaml(const std::filesystem::path &joint_yaml_path) -> JointProperties {
  auto maybe_joint = Yaml::FromFile(joint_yaml_path, "joint");
  REQUIRE(maybe_joint);
  auto map = maybe_joint.value()->AsUnorderedMap();

  JointProperties properties = {
      .upper_joint_width = map.at("joint/upper_joint_width").Value<double>().value(),
      .left_max_surface_angle = map.at("joint/left_max_surface_angle").Value<double>().value(),
      .right_max_surface_angle = map.at("joint/right_max_surface_angle").Value<double>().value(),
      .left_joint_angle = map.at("joint/left_joint_angle").Value<double>().value(),
      .right_joint_angle = map.at("joint/right_joint_angle").Value<double>().value(),
      .groove_depth = map.at("joint/groove_depth").Value<double>().value(),
      .upper_joint_width_tolerance = 7.0,
      .surface_angle_tolerance = 10.0 * static_cast<double>(EIGEN_PI) / 180.0,
      .groove_angle_tolerance = 9.0 * static_cast<double>(EIGEN_PI) / 180.0,
      .offset_distance = 3.0,
  };
  return properties;
}

// Dataset YAML structure expected:
// data:
//   - image: <path>
//     expected:
//       ABW:
//         - { x: <double>, y: <double> }  # ABW0
//         - { x: <double>, y: <double> }  # ABW1
//         ... total 7 points
// tolerance:
//   x_mm: <double>
//   y_mm: <double>

TEST_SUITE("BigSnake::Parse - dataset") {
  TEST_CASE("single image, compare against expected ABW from dataset if available") {
    // Paths within repo
    const std::filesystem::path calib_yaml = "/workspace/assets/scanner_calibration/HIL.yaml";
    const std::filesystem::path joint_yaml = "/workspace/tests/configs/sil/configuration.yaml"; // uses calibration_fixture_joint_geometry
    const std::filesystem::path dataset_yaml = "/workspace/src/scanner/joint_model/test/datasets/big_snake_dataset.yaml";

    // Build camera
    scanner::image_provider::Fov fov{.width = 3500, .height = 2500, .offset_x = 312, .offset_y = 0};
    auto camera = LoadCameraFromYaml(calib_yaml, fov);

    // Build joint properties
    // Use section "calibration_fixture_joint_geometry" from tests config if available; fallback to fixed joint yaml if provided.
    JointProperties properties{};
    {
      auto maybe = Yaml::FromFile(joint_yaml, "calibration_fixture_joint_geometry");
      REQUIRE(maybe);
      auto map = maybe.value()->AsUnorderedMap();
      properties.upper_joint_width = map.at("calibration_fixture_joint_geometry/upper_joint_width").Value<double>().value();
      properties.left_max_surface_angle = map.at("calibration_fixture_joint_geometry/left_max_surface_angle").Value<double>().value();
      properties.right_max_surface_angle = map.at("calibration_fixture_joint_geometry/right_max_surface_angle").Value<double>().value();
      properties.left_joint_angle = map.at("calibration_fixture_joint_geometry/left_joint_angle").Value<double>().value();
      properties.right_joint_angle = map.at("calibration_fixture_joint_geometry/right_joint_angle").Value<double>().value();
      properties.groove_depth = map.at("calibration_fixture_joint_geometry/groove_depth").Value<double>().value();
      properties.upper_joint_width_tolerance = 7.0;
      properties.surface_angle_tolerance = 10.0 * static_cast<double>(EIGEN_PI) / 180.0;
      properties.groove_angle_tolerance = 9.0 * static_cast<double>(EIGEN_PI) / 180.0;
      properties.offset_distance = 3.0;
    }

    // Scanner threshold config
    scanner::ScannerConfigurationData scanner_cfg{.gray_minimum_top = 48, .gray_minimum_wall = 16, .gray_minimum_bottom = 48};

    // Create joint model
    auto joint_model = std::make_unique<BigSnake>(properties, scanner_cfg, std::move(camera));

    // Load dataset
    auto maybe_ds = Yaml::FromFile(dataset_yaml, "data");
    REQUIRE(maybe_ds);
    auto map = maybe_ds.value()->AsUnorderedMap();

    // Only run first entry for now (expand later if multiple)
    // Expect key: data/0/image and data/0/expected/ABW/0/x etc.
    // Image path can be absolute or relative to workspace.
    auto image_path_val = map.at("data/0/image").Value<std::string>();
    REQUIRE(image_path_val.has_value());
    auto image_path = std::filesystem::path(image_path_val.value());

    // Read image
    auto cv_img = imread(image_path.string(), cv::IMREAD_GRAYSCALE);
    REQUIRE(cv_img.data != nullptr);
    auto maybe_image = scanner::image::ImageBuilder::From(cv_img, image_path.filename().string(), 0).Finalize();
    REQUIRE(maybe_image);
    auto &image = *maybe_image.value();

    // Parse
    auto result = joint_model->Parse(image, std::nullopt, std::nullopt, false, std::nullopt);
    REQUIRE(result.has_value());
    auto [profile, snake_lpcs, processing_time_ms, num_walls] = result.value();
    (void)snake_lpcs;
    (void)processing_time_ms;
    (void)num_walls;

    // Tolerance in mm from dataset
    double tol_x_mm = 0.5;  // default
    double tol_y_mm = 0.5;  // default
    if (auto v = map.find("tolerance/x_mm"); v != map.end()) {
      tol_x_mm = v->second.Value<double>().value();
    }
    if (auto v = map.find("tolerance/y_mm"); v != map.end()) {
      tol_y_mm = v->second.Value<double>().value();
    }
    const double mm_to_m = 1e-3;
    const double tol_x = tol_x_mm * mm_to_m;
    const double tol_y = tol_y_mm * mm_to_m;

    // Compare ABW points if provided in dataset
    for (int i = 0; i < 7; i++) {
      std::string keyx = fmt::format("data/0/expected/ABW/{}/x", i);
      std::string keyy = fmt::format("data/0/expected/ABW/{}/y", i);
      auto itx = map.find(keyx);
      auto ity = map.find(keyy);
      if (itx != map.end() && ity != map.end()) {
        auto exp_x = itx->second.Value<double>().value();
        auto exp_y = ity->second.Value<double>().value();
        CHECK(std::fabs(profile.points[static_cast<size_t>(i)].x - exp_x) <= tol_x);
        CHECK(std::fabs(profile.points[static_cast<size_t>(i)].y - exp_y) <= tol_y);
      }
    }
  }
}
#endif

