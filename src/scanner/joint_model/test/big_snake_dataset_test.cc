#include <doctest/doctest.h>

#include <filesystem>
#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "common/file/yaml.h"
#include "scanner/image/image_builder.h"
#include "scanner/image/tilted_perspective_camera.h"
#include "scanner/joint_model/big_snake.h"

namespace fs = std::filesystem;
using common::file::Yaml;
using scanner::ScannerConfigurationData;
using scanner::image::TiltedPerspectiveCamera;
using scanner::image::TiltedPerspectiveCameraProperties;
using scanner::joint_model::BigSnake;
using scanner::joint_model::JointProperties;

static auto load_camera_properties(const fs::path &calibration_yaml) -> std::optional<TiltedPerspectiveCameraProperties> {
  auto maybe_yaml = Yaml::FromFile(calibration_yaml, "camera");
  if (maybe_yaml.has_error()) {
    return std::nullopt;
  }
  auto map = maybe_yaml.value()->AsUnorderedMap();
  return TiltedPerspectiveCameraProperties::FromUnorderedMap(map);
}

static auto load_image(const fs::path &image_path) -> std::optional<std::unique_ptr<scanner::image::Image>> {
  auto gray = cv::imread(image_path.string(), cv::IMREAD_GRAYSCALE);
  if (gray.empty()) {
    return std::nullopt;
  }
  auto maybe_img = scanner::image::ImageBuilder::From(gray, image_path.filename().string(), 0).Finalize();
  if (!maybe_img.has_value()) {
    return std::nullopt;
  }
  return maybe_img.value();
}

static auto default_joint_properties() -> JointProperties {
  // Reasonable generic defaults; dataset can override via YAML when available
  return JointProperties{.upper_joint_width           = 50.0,
                         .left_max_surface_angle      = 0.34906585,
                         .right_max_surface_angle     = 0.34906585,
                         .left_joint_angle            = 0.1396,
                         .right_joint_angle           = 0.1396,
                         .groove_depth                = 28.0,
                         .upper_joint_width_tolerance = 7.0,
                         .surface_angle_tolerance     = 0.174532925,
                         .groove_angle_tolerance      = 0.13962634,
                         .offset_distance             = 3.0};
}

static auto default_scanner_filtering() -> ScannerConfigurationData { return {32, 16, 32}; }

// Dataset-driven test: verifies ABW coordinates if a dataset YAML is present
TEST_SUITE("BigSnake Dataset") {
  TEST_CASE("ABW points match dataset expectations (if dataset available)") {
    // Locate dataset YAML
    fs::path dataset_path;
    if (const char *env = std::getenv("ADAPTIO_BIGSNAKE_DATASET")) {
      dataset_path = fs::path(env);
    } else {
      dataset_path = fs::path("tests/data_set/data_set.yaml");
    }

    if (!fs::exists(dataset_path)) {
      MESSAGE("Dataset not found - set ADAPTIO_BIGSNAKE_DATASET or place tests/data_set/data_set.yaml. Skipping.");
      return;  // Skip test gracefully
    }

    // Parse dataset with yaml-cpp directly for flexibility
    YAML::Node root = YAML::LoadFile(dataset_path.string());

    auto epsilon = root["epsilon"] ? root["epsilon"].as<double>() : 1e-4;  // coordinate tolerance

    // Read joint configuration (optional)
    JointProperties joint_props = default_joint_properties();
    if (root["joint"]) {
      auto j = root["joint"];
      if (j["upper_joint_width"]) joint_props.upper_joint_width = j["upper_joint_width"].as<double>();
      if (j["left_max_surface_angle"]) joint_props.left_max_surface_angle = j["left_max_surface_angle"].as<double>();
      if (j["right_max_surface_angle"]) joint_props.right_max_surface_angle = j["right_max_surface_angle"].as<double>();
      if (j["left_joint_angle"]) joint_props.left_joint_angle = j["left_joint_angle"].as<double>();
      if (j["right_joint_angle"]) joint_props.right_joint_angle = j["right_joint_angle"].as<double>();
      if (j["groove_depth"]) joint_props.groove_depth = j["groove_depth"].as<double>();
      if (j["upper_joint_width_tolerance"]) joint_props.upper_joint_width_tolerance = j["upper_joint_width_tolerance"].as<double>();
      if (j["surface_angle_tolerance"]) joint_props.surface_angle_tolerance = j["surface_angle_tolerance"].as<double>();
      if (j["groove_angle_tolerance"]) joint_props.groove_angle_tolerance = j["groove_angle_tolerance"].as<double>();
      if (j["offset_distance"]) joint_props.offset_distance = j["offset_distance"].as<double>();
    }

    // Scanner filtering thresholds (optional)
    ScannerConfigurationData filtering = default_scanner_filtering();
    if (root["filtering"]) {
      auto f = root["filtering"];
      if (f["gray_minimum_top"]) filtering.gray_minimum_top = f["gray_minimum_top"].as<long long>();
      if (f["gray_minimum_wall"]) filtering.gray_minimum_wall = f["gray_minimum_wall"].as<long long>();
      if (f["gray_minimum_bottom"]) filtering.gray_minimum_bottom = f["gray_minimum_bottom"].as<long long>();
    }

    REQUIRE(root["scanners"]);
    auto scanners = root["scanners"];
    REQUIRE(scanners.IsSequence());

    for (std::size_t s = 0; s < scanners.size(); ++s) {
      auto scanner_node = scanners[s];

      // Per-scanner calibration file
      REQUIRE_MESSAGE(scanner_node["calibration"], "Missing 'calibration' for scanner index " << s);
      fs::path calib_path = scanner_node["calibration"].as<std::string>();
      if (calib_path.is_relative()) calib_path = dataset_path.parent_path() / calib_path;

      auto maybe_cam_props = load_camera_properties(calib_path);
      REQUIRE_MESSAGE(maybe_cam_props.has_value(), "Failed loading calibration: " << calib_path);
      auto camera_model = std::make_unique<TiltedPerspectiveCamera>(maybe_cam_props.value());

      // Per-scanner joint override (optional)
      JointProperties effective_joint = joint_props;
      if (scanner_node["joint"]) {
        auto j = scanner_node["joint"];
        if (j["upper_joint_width"]) effective_joint.upper_joint_width = j["upper_joint_width"].as<double>();
        if (j["left_max_surface_angle"]) effective_joint.left_max_surface_angle = j["left_max_surface_angle"].as<double>();
        if (j["right_max_surface_angle"]) effective_joint.right_max_surface_angle = j["right_max_surface_angle"].as<double>();
        if (j["left_joint_angle"]) effective_joint.left_joint_angle = j["left_joint_angle"].as<double>();
        if (j["right_joint_angle"]) effective_joint.right_joint_angle = j["right_joint_angle"].as<double>();
        if (j["groove_depth"]) effective_joint.groove_depth = j["groove_depth"].as<double>();
        if (j["upper_joint_width_tolerance"]) effective_joint.upper_joint_width_tolerance = j["upper_joint_width_tolerance"].as<double>();
        if (j["surface_angle_tolerance"]) effective_joint.surface_angle_tolerance = j["surface_angle_tolerance"].as<double>();
        if (j["groove_angle_tolerance"]) effective_joint.groove_angle_tolerance = j["groove_angle_tolerance"].as<double>();
        if (j["offset_distance"]) effective_joint.offset_distance = j["offset_distance"].as<double>();
      }

      BigSnake big_snake(effective_joint, filtering, std::move(camera_model));

      REQUIRE_MESSAGE(scanner_node["images"], "Missing 'images' for scanner index " << s);
      auto images = scanner_node["images"];
      REQUIRE(images.IsSequence());

      for (std::size_t i = 0; i < images.size(); ++i) {
        auto img_node = images[i];
        REQUIRE_MESSAGE(img_node["file"], "Missing 'file' for scanner index " << s << ", image index " << i);

        fs::path img_path = img_node["file"].as<std::string>();
        if (img_path.is_relative()) img_path = dataset_path.parent_path() / img_path;

        auto maybe_image = load_image(img_path);
        REQUIRE_MESSAGE(maybe_image.has_value(), "Failed loading image: " << img_path);
        auto &image_ptr = maybe_image.value();

        // Optional image-specific joint override
        auto local_joint = effective_joint;
        if (img_node["joint"]) {
          auto j = img_node["joint"];
          if (j["upper_joint_width"]) local_joint.upper_joint_width = j["upper_joint_width"].as<double>();
          if (j["left_max_surface_angle"]) local_joint.left_max_surface_angle = j["left_max_surface_angle"].as<double>();
          if (j["right_max_surface_angle"]) local_joint.right_max_surface_angle = j["right_max_surface_angle"].as<double>();
          if (j["left_joint_angle"]) local_joint.left_joint_angle = j["left_joint_angle"].as<double>();
          if (j["right_joint_angle"]) local_joint.right_joint_angle = j["right_joint_angle"].as<double>();
          if (j["groove_depth"]) local_joint.groove_depth = j["groove_depth"].as<double>();
          if (j["upper_joint_width_tolerance"]) local_joint.upper_joint_width_tolerance = j["upper_joint_width_tolerance"].as<double>();
          if (j["surface_angle_tolerance"]) local_joint.surface_angle_tolerance = j["surface_angle_tolerance"].as<double>();
          if (j["groove_angle_tolerance"]) local_joint.groove_angle_tolerance = j["groove_angle_tolerance"].as<double>();
          if (j["offset_distance"]) local_joint.offset_distance = j["offset_distance"].as<double>();
        }

        // Re-create BigSnake if joint parameters changed
        BigSnake local_big_snake(local_joint, filtering, std::make_unique<TiltedPerspectiveCamera>(maybe_cam_props.value()));

        auto res = local_big_snake.Parse(*image_ptr, std::nullopt, std::nullopt, false, std::nullopt);
        REQUIRE_MESSAGE(res.has_value(), "BigSnake::Parse failed for image: " << img_path);

        auto profile = std::get<0>(res.value());

        REQUIRE_MESSAGE(img_node["expected_abw"],
                        "Missing 'expected_abw' for scanner index " << s << ", image index " << i);
        auto expected = img_node["expected_abw"];
        REQUIRE_MESSAGE(expected.IsSequence() && expected.size() == 7, "'expected_abw' must be a list of 7 points");

        for (std::size_t p = 0; p < 7; ++p) {
          auto pt = expected[p];
          REQUIRE(pt.IsMap());
          auto ex = pt["x"].as<double>();
          auto ey = pt["y"].as<double>();
          CHECK_MESSAGE(doctest::Approx(profile.points[p].x).epsilon(epsilon) == ex,
                        "ABW" << p << " x mismatch (scanner " << s << ", image " << i << ")");
          CHECK_MESSAGE(doctest::Approx(profile.points[p].y).epsilon(epsilon) == ey,
                        "ABW" << p << " y mismatch (scanner " << s << ", image " << i << ")");
        }
      }
    }
  }
}

// Fallback smoke test: iterate all local test images against available calibrations
TEST_SUITE("BigSnake Smoke") {
  TEST_CASE("Parses all sample images with all available calibrations") {
    const fs::path images_dir = fs::path("src/scanner/joint_model/test/test_data");
    const fs::path calib_dir  = fs::path("assets/scanner_calibration");

    if (!fs::exists(images_dir) || !fs::exists(calib_dir)) {
      MESSAGE("No images or calibrations found - skipping smoke test");
      return;
    }

    std::vector<fs::path> images;
    for (auto &ent : fs::directory_iterator(images_dir)) {
      if (!ent.is_regular_file()) continue;
      auto ext = ent.path().extension().string();
      if (ext == ".tiff" || ext == ".tif" || ext == ".bmp") {
        images.push_back(ent.path());
      }
    }

    std::vector<TiltedPerspectiveCameraProperties> calibrations;
    for (auto &ent : fs::directory_iterator(calib_dir)) {
      if (!ent.is_regular_file()) continue;
      if (ent.path().extension() == ".yaml" || ent.path().extension() == ".yml") {
        auto props = load_camera_properties(ent.path());
        if (props) calibrations.push_back(props.value());
      }
    }

    REQUIRE(!images.empty());
    REQUIRE(!calibrations.empty());

    auto filtering = default_scanner_filtering();
    auto joint     = default_joint_properties();

    for (const auto &img : images) {
      auto maybe_image = load_image(img);
      REQUIRE_MESSAGE(maybe_image.has_value(), "Failed loading image: " << img);
      auto &image_ptr = maybe_image.value();
      for (const auto &cal : calibrations) {
        BigSnake big_snake(joint, filtering, std::make_unique<TiltedPerspectiveCamera>(cal));
        auto res = big_snake.Parse(*image_ptr, std::nullopt, std::nullopt, false, std::nullopt);
        CHECK_MESSAGE(res.has_value(), "Parse failed for image " << img.filename().string());
        if (res) {
          const auto &points = std::get<0>(res.value()).points;
          // Basic sanity: x strictly increasing from ABW0..ABW6
          for (int i = 0; i < 6; ++i) {
            CHECK(points[i].x < points[i + 1].x);
          }
        }
      }
    }
  }
}

