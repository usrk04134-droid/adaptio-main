#include "scanner/joint_model/big_snake.h"

#include <cmath>
#include <cstdlib>
#include <expected>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "common/file/yaml.h"
#include "common/logging/application_log.h"
#include "scanner/image/image.h"
#include "scanner/image/image_builder.h"
#include "scanner/image/tilted_perspective_camera.h"
#include "scanner/scanner_configuration.h"

#ifndef DOCTEST_CONFIG_DISABLE
#include <doctest/doctest.h>

// NOLINTBEGIN(*-magic-numbers, *-optional-access, *-use-nodiscard)

namespace {

using scanner::image::TiltedPerspectiveCamera;
using scanner::image::TiltedPerspectiveCameraProperties;
using scanner::joint_model::BigSnake;
using scanner::joint_model::JointProperties;

struct ImageCase {
	std::string name;
	std::filesystem::path path;
	// Expected ABW points if available
	std::optional<std::array<double, 7>> expected_xs;
	std::optional<std::array<double, 7>> expected_ys;
};

struct TestEnv {
	JointProperties joint_properties{};
	scanner::ScannerConfigurationData scanner_config{};
	TiltedPerspectiveCameraProperties default_camera_properties{};
	std::vector<ImageCase> images;
};

auto MakeDefaultCameraProperties() -> TiltedPerspectiveCameraProperties {
	TiltedPerspectiveCameraProperties camera_properties;
	// Default HIL-like calibration. Tests relying on exact ABW comparison should
	// override via YAML dataset to match the original calibration.
	camera_properties.config_calib.intrinsic = {
	    .projection_center_distance = 0.0,
	    .focus_distance             = 4.715378224147283,
	    .principal_point            = {.x = 0.9930139449441449, .y = 0.754875302126084},
	    .pixel_pitch                = {.x = 2.74e-06, .y = 2.74e-06},
	    .rho                        = 3.141592653589793,
	    .tau                        = 0.12217304763960307,
	    .d                          = 0.5597753718678573,
	    .K1                         = 0.007962795536909519,
	    .K2                         = 0.026772094795031075,
	    .K3                         = -0.014787649308636013,
	    .P1                         = -0.0028219237254003744,
	    .P2                         = 0.00417864043329311,
	    .scaling_factors            = {.w = 0.00565536, .m = 0.1, .K1 = 0.1, .K2 = 0.1, .K3 = 0.1, .P1 = 0.1, .P2 = 0.1},
	};
	camera_properties.config_calib.extrinsic.rotation.row(0)
	    << 0.9999900141669034, -0.0037822314392825704, -0.0023803974071929038;
	camera_properties.config_calib.extrinsic.rotation.row(1)
	    << -0.0006203294305648313, 0.41001800882613115, -0.9120772158264081;
	camera_properties.config_calib.extrinsic.rotation.row(2)
	    << 0.004425692925864075, 0.9120695846061277, 0.41001156822525336;
	camera_properties.config_calib.extrinsic.translation.col(0)
	    << -0.04886608988697602, -0.04393368249422651, 0.37231035253911315;
	camera_properties.config_fov = {.width = 3500, .offset_x = 312, .height = 2500, .offset_y = 0};
	return camera_properties;
}

auto SetupEnv() -> TestEnv {
	TestEnv env{};
	// Reasonable defaults; dataset may override expectations only.
	env.joint_properties = {.upper_joint_width           = 25.0,
	                        .left_max_surface_angle      = 0.34906585,
	                        .right_max_surface_angle     = 0.34906585,
	                        .left_joint_angle            = 0.16,
	                        .right_joint_angle           = 0.140,
	                        .groove_depth                = 42.0,
	                        .upper_joint_width_tolerance = 7.0,
	                        .surface_angle_tolerance     = 10.0 * M_PI / 180.0,
	                        .groove_angle_tolerance      = 9.0 * M_PI / 180.0,
	                        .offset_distance             = 3.0};
	env.scanner_config           = {.gray_minimum_top = 48, .gray_minimum_wall = 16, .gray_minimum_bottom = 48};
	env.default_camera_properties = MakeDefaultCameraProperties();

	// Discover bundled test images
	auto base_dir = std::filesystem::path("./src/scanner/joint_model/test/test_data");
	if (std::filesystem::exists(base_dir) && std::filesystem::is_directory(base_dir)) {
		for (const auto &entry : std::filesystem::directory_iterator(base_dir)) {
			const auto &p = entry.path();
			if (!entry.is_regular_file()) continue;
			const auto ext = p.extension().string();
			if (ext == ".tif" || ext == ".tiff" || ext == ".png" || ext == ".jpg" || ext == ".jpeg") {
				ImageCase ic{};
				ic.name = p.filename().string();
				ic.path = p;
				env.images.push_back(std::move(ic));
			}
		}
	}

	// Optional dataset YAML co-located next to tests or at repo root
	std::vector<std::filesystem::path> candidate_yaml = {
	    base_dir / "data_set.yaml",
	    std::filesystem::path("./tests/data_set/data_set.yaml"),
	    std::filesystem::path("./data_set.yaml"),
	};

	for (const auto &yaml_path : candidate_yaml) {
		if (!std::filesystem::exists(yaml_path)) continue;
		auto maybe_yaml = common::file::Yaml::FromFile(yaml_path, "data");
		if (maybe_yaml.has_error()) continue;
		auto map = maybe_yaml.value()->AsUnorderedMap();
		// Expect a structure like: data/images/<id>/name, xs[], ys[] or similar.
		// We will support keys of the form: data/images/<filename>/expected/xs[i], ys[i]
		for (auto &img : env.images) {
			std::array<double,7> xs{};
			std::array<double,7> ys{};
			bool found = true;
			for (int i = 0; i < 7; ++i) {
				const auto keyx = fmt::format("data/images/{}/expected/xs/{}", img.name, i);
				const auto keyy = fmt::format("data/images/{}/expected/ys/{}", img.name, i);
				auto vx = map.find(keyx);
				auto vy = map.find(keyy);
				if (vx == map.end() || vy == map.end()) {
					found = false;
					break;
				}
				xs[static_cast<size_t>(i)] = vx->second.Value<double>().value();
				ys[static_cast<size_t>(i)] = vy->second.Value<double>().value();
			}
			if (found) {
				img.expected_xs = xs;
				img.expected_ys = ys;
			}
		}
		break; // only one yaml consumed
	}

	return env;
}

} // namespace

TEST_SUITE("BigSnake dataset") {
	TEST_CASE("Parse all test images and optionally verify ABW") {
		auto env = SetupEnv();

		// Build list of camera calibrations to test: default + scanner_calibration folder
		std::vector<std::pair<std::string, TiltedPerspectiveCameraProperties>> camera_props;
		camera_props.emplace_back(std::make_pair(std::string("default"), env.default_camera_properties));

		const auto calib_dir = std::filesystem::path("./assets/scanner_calibration");
		if (std::filesystem::exists(calib_dir) && std::filesystem::is_directory(calib_dir)) {
			for (const auto &entry : std::filesystem::directory_iterator(calib_dir)) {
				if (!entry.is_regular_file()) continue;
				if (entry.path().extension() != ".yaml" && entry.path().extension() != ".yml") continue;
				auto maybe_yaml = common::file::Yaml::FromFile(entry.path(), "camera");
				if (maybe_yaml.has_error()) continue;
				auto map = maybe_yaml.value()->AsUnorderedMap();
				scanner::image::TiltedPerspectiveCameraProperties props = scanner::image::TiltedPerspectiveCameraProperties::FromUnorderedMap(map);
				// Use standard FOV
				props.config_fov = {.width = 3500, .offset_x = 312, .height = 2500, .offset_y = 0};
				camera_props.emplace_back(std::make_pair(entry.path().filename().string(), props));
			}
		}

		for (const auto &img_case : env.images) {
			SUBCASE(img_case.name.c_str()) {
				for (auto &named_cam : camera_props) {
					SUBCASE(named_cam.first.c_str()) {
						auto big_snake = BigSnake(env.joint_properties, env.scanner_config, std::make_unique<TiltedPerspectiveCamera>(named_cam.second));

						auto grayscale_image = cv::imread(img_case.path.string(), cv::IMREAD_GRAYSCALE);
						REQUIRE(grayscale_image.data != nullptr);
						auto maybe_image = scanner::image::ImageBuilder::From(grayscale_image, img_case.name, 0).Finalize();
						REQUIRE(maybe_image.has_value());
						auto *image = maybe_image.value().get();

						auto res = big_snake.Parse(*image, {}, {}, false, {});
						CHECK(res.has_value());
						if (!res.has_value()) continue;
						auto parsed = res.value();
						auto &profile = std::get<0>(parsed);

						if (img_case.expected_xs && img_case.expected_ys && named_cam.first == "default") {
							for (int i = 0; i < 7; i++) {
								CHECK(std::abs(profile.points[i].x - img_case.expected_xs->at(static_cast<size_t>(i))) < 2e-4);
								CHECK(std::abs(profile.points[i].y - img_case.expected_ys->at(static_cast<size_t>(i))) < 2e-4);
							}
						} else {
							// At least sanity check coordinate ranges
							for (int i = 0; i < 7; i++) {
								CHECK(std::isfinite(profile.points[i].x));
								CHECK(std::isfinite(profile.points[i].y));
							}
						}
					}
				}
			}
		}
	}
}

#endif

// NOLINTEND(*-magic-numbers, *-optional-access, *-use-nodiscard)