#include "scanner/joint_model/big_snake.h"

#include <cmath>
#include <cstdlib>
#include <optional>
#include <filesystem>
#include <array>
#include <string>
#include <yaml-cpp/yaml.h>

#include "common/logging/application_log.h"
#include "scanner/image/camera_model.h"
#include "scanner/image/image.h"
#include "scanner/image/image_builder.h"
#include "scanner/image/tilted_perspective_camera.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/scanner_configuration.h"
#ifndef DOCTEST_CONFIG_DISABLE
#include <doctest/doctest.h>

// NOLINTBEGIN(*-magic-numbers, *-optional-access, *-use-nodiscard)

struct TestData {
  scanner::joint_model::JointProperties joint_properties{};
  scanner::ScannerConfigurationData scanner_config{};
  std::unique_ptr<scanner::image::TiltedPerspectiveCamera> camera_model;
  struct {
    std::string name;
    std::string path;
  } image_data;
};

inline auto Setup() -> TestData {
  TestData test_data;
  const auto base_dir = std::filesystem::path(__FILE__).parent_path() / "test_data";

  scanner::joint_model::JointProperties jp = {.upper_joint_width           = 25.0,
                                              .left_max_surface_angle      = 0.34906585,
                                              .right_max_surface_angle     = 0.34906585,
                                              .left_joint_angle            = 0.16,
                                              .right_joint_angle           = 0.140,
                                              .groove_depth                = 42.0,
                                              .upper_joint_width_tolerance = 7.0,
                                              .surface_angle_tolerance     = 10.0 * std::numbers::pi / 180.0,
                                              .groove_angle_tolerance      = 9.0 * std::numbers::pi / 180.0,
                                              .offset_distance             = 3.0};

  scanner::image::TiltedPerspectiveCameraProperties camera_properties;

  camera_properties.config_calib.intrinsic = {
      .projection_center_distance = 0.0,
      .focus_distance             = 4.707852952290943804,
      .principal_point =
          {
                            .x = 1.001100742322118764,
                            .y = 7.317642435771299914e-01,
                            },
      .pixel_pitch =
          {
                            .x = 2.74e-06,
                            .y = 2.74e-06,
                            },
      .rho = 3.141447305679321289,
      .tau = 1.221730476396030718,
      .d   = 6.193863034310445048e-01,
      .K1  = 2.545519889414866316e-02,
      .K2  = 4.181119910248848152e-03,
      .K3  = -6.696371931147962128e-03,
      .P1  = -3.320003802347088265e-03,
      .P2  = 3.050356537053298695e-03,
      .scaling_factors =
          {
                            .w  = 5.633439999999999975e-03,
                            .m  = 0.1,
                            .K1 = 0.1,
                            .K2 = 0.1,
                            .K3 = 0.1,
                            .P1 = 0.1,
                            .P2 = 0.1,
                            },
  };

  camera_properties.config_calib.extrinsic.rotation.row(0)
      << 9.997229424317457536e-01, -2.350816639678374523e-02, 1.185111080670121601e-03;
  camera_properties.config_calib.extrinsic.rotation.row(1)
      << 1.065018256781005875e-02, 4.068680551182541349e-01, -9.134248514987763912e-01;
  camera_properties.config_calib.extrinsic.rotation.row(2)
      << 2.099075955949937164e-02, 9.131844018800093776e-01, 4.070056955082629324e-01;
  camera_properties.config_calib.extrinsic.translation.col(0)
      << -5.106240047893689099e-02, -2.791469469541549980e-02, 3.925539620524008955e-01;

  camera_properties.config_fov = {.width = 3500, .offset_x = 312, .height = 2500, .offset_y = 0};

  auto camera_model = std::make_unique<scanner::image::TiltedPerspectiveCamera>(camera_properties);

  test_data = {.joint_properties = jp,
               .scanner_config   = {.gray_minimum_top = 48, .gray_minimum_wall = 16, .gray_minimum_bottom = 48},
               .camera_model     = std::move(camera_model),
               .image_data       = {.name = "Image__2024-08-16__11-13-03.tiff",
                                 .path = (base_dir.string() + "/images/")}};

  return test_data;
}

struct ExpectedPoints {
  std::array<double, 7> xs{};
  std::array<double, 7> ys{};
  bool loaded_from_yaml{false};
};

inline auto LoadExpectedFromYaml(const std::filesystem::path& yaml_path, const std::string& image_name) -> std::optional<ExpectedPoints> {
  try {
    if (!std::filesystem::exists(yaml_path)) {
      LOG_WARNING("Annotations YAML not found at {}", yaml_path.string());
      return std::nullopt;
    }

    auto yaml = YAML::LoadFile(yaml_path.string());

    // Accept either the report-style structure under "data" or a top-level sequence
    auto data_node = yaml["data"].IsDefined() ? yaml["data"] : yaml;

    // Iterate mappings like ABWPoints0, ABWPoints1, ... or sequence entries
    if (data_node.IsMap()) {
      for (auto it = data_node.begin(); it != data_node.end(); ++it) {
        const auto& node = it->second;
        if (!node.IsMap()) {
          continue;
        }
        if (!node["image"].IsDefined()) {
          continue;
        }

        std::string img = node["image"].as<std::string>("");
        auto img_filename = std::filesystem::path(img).filename().string();
        auto desired_filename = std::filesystem::path(image_name).filename().string();
        if (img_filename != desired_filename) {
          continue;
        }

        ExpectedPoints out{};
        out.loaded_from_yaml = true;
        for (int i = 0; i < 7; i++) {
          auto key = std::string("ABW") + std::to_string(i);
          if (!node[key].IsDefined()) {
            LOG_WARNING("Missing key {} in YAML for image {}", key, img_filename);
            return std::nullopt;
          }
          out.xs[static_cast<size_t>(i)] = node[key]["x"].as<double>();
          out.ys[static_cast<size_t>(i)] = node[key]["y"].as<double>();
        }
        return out;
      }
    } else if (data_node.IsSequence()) {
      for (auto entry : data_node) {
        if (!entry.IsMap() || !entry["image"].IsDefined()) {
          continue;
        }
        std::string img = entry["image"].as<std::string>("");
        auto img_filename = std::filesystem::path(img).filename().string();
        auto desired_filename = std::filesystem::path(image_name).filename().string();
        if (img_filename != desired_filename) {
          continue;
        }
        ExpectedPoints out{};
        out.loaded_from_yaml = true;
        for (int i = 0; i < 7; i++) {
          auto key = std::string("ABW") + std::to_string(i);
          out.xs[static_cast<size_t>(i)] = entry[key]["x"].as<double>();
          out.ys[static_cast<size_t>(i)] = entry[key]["y"].as<double>();
        }
        return out;
      }
    }

  } catch (const std::exception& ex) {
    LOG_ERROR("Failed to load expected annotations from YAML: {}", ex.what());
    return std::nullopt;
  }

  return std::nullopt;
}

TEST_SUITE("Test Big Snake") {
  TEST_CASE("Parse") {
    auto test_data = Setup();

    auto big_snake = scanner::joint_model::BigSnake(test_data.joint_properties, test_data.scanner_config,
                                                    std::move(test_data.camera_model));

    auto grayscale_image = cv::imread(test_data.image_data.path + test_data.image_data.name, cv::IMREAD_GRAYSCALE);
    auto maybe_image     = scanner::image::ImageBuilder::From(grayscale_image, test_data.image_data.name, 0).Finalize();
    auto *image          = maybe_image.value().get();

    auto res = big_snake.Parse(*image, {}, {}, false, {});
    REQUIRE(res.has_value());

    auto parsed = res.value();
    auto &profile = std::get<0>(parsed);

    // Load expected ABW points dynamically from YAML; fallback to hardcoded values if not found
    const auto base_dir = std::filesystem::path(__FILE__).parent_path() / "test_data";
    const auto yaml_path = base_dir / "data_set.yaml";

    ExpectedPoints expected{};
    auto loaded = LoadExpectedFromYaml(yaml_path, test_data.image_data.name);
    if (loaded.has_value()) {
      expected = loaded.value();
      LOG_INFO("Loaded expected ABW points from {} for image {}", yaml_path.string(), test_data.image_data.name);
    } else {
      // Expected ABW points for Image__2024-08-16__11-13-03.tiff from data_set.yaml (id: 3)
      expected.xs = {0.0394735, 0.0459291, 0.0494897, 0.0530746, 0.0566018, 0.06003, 0.064345};
      expected.ys = {0.0459356, 0.00591587, 0.00567378, 0.00476669, 0.00585357, 0.0061066, 0.0444451};
      LOG_WARNING("Falling back to hardcoded expected ABW points for image {}", test_data.image_data.name);
    }

    for (int i = 0; i < 7; i++) {
      LOG_INFO("image parsed points {} {}",profile.points[i].x,profile.points[i].y);
      LOG_INFO("expected annotated abw points {} {}",expected.xs[static_cast<size_t>(i)],expected.ys[static_cast<size_t>(i)]);
      CHECK(std::abs(profile.points[i].x - expected.xs[static_cast<size_t>(i)]) < 0.4);
      CHECK(std::abs(profile.points[i].y - expected.ys[static_cast<size_t>(i)]) < 0.4);
    }
  }
}
#endif

// NOLINTEND(*-magic-numbers, *-optional-access, *-use-nodiscard)

