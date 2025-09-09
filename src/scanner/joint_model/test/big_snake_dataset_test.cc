#include <doctest/doctest.h>

#include <cmath>
#include <cstdlib>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "common/file/yaml.h"
#include "scanner/image/image_builder.h"
#include "scanner/image/tilted_perspective_camera.h"
#include "scanner/joint_model/big_snake.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/scanner_configuration.h"

using common::file::Yaml;
using scanner::image::TiltedPerspectiveCamera;
using scanner::image::TiltedPerspectiveCameraProperties;
using scanner::joint_model::BigSnake;
using scanner::joint_model::JointModelErrorCode;
using scanner::joint_model::JointProperties;

namespace {

struct ImageExpectation {
	int id;
	std::string name;
	std::array<double, 7> xs;
	std::array<double, 7> ys;
};

// Helper to read expectations from YAML produced by test_image_set (data/ABWPointsN with ABW0..ABW6 and image)
static auto ReadExpectations(const std::string &yaml_path)
	-> std::unordered_map<int, ImageExpectation> {
	auto map        = std::unordered_map<int, ImageExpectation>{};
	auto y          = Yaml::FromFile(yaml_path, "data").value();
	auto parameters = y->AsUnorderedMap();

	// Collect base groups like data/ABWPoints0, data/ABWPoints1, ...
	std::unordered_map<std::string, int> group_to_id;
	for (const auto &kv : parameters) {
		const auto &key = kv.first;
		auto pos        = key.find("data/ABWPoints");
		if (pos == std::string::npos) {
			continue;
		}
		auto slash_pos = key.find('/', pos + std::string("data/").size());
		if (slash_pos == std::string::npos) {
			continue;
		}
		std::string group = key.substr(0, slash_pos);
		auto abw_pos      = group.rfind("ABWPoints");
		if (abw_pos == std::string::npos) {
			continue;
		}
		auto id_str = group.substr(abw_pos + std::string("ABWPoints").size());
		try {
			int id_val = std::stoi(id_str);
			group_to_id.emplace(group, id_val);
		} catch (...) {
			// ignore
		}
	}

	for (const auto &entry : group_to_id) {
		const auto &group = entry.first;
		int id            = entry.second;
		ImageExpectation exp{};
		exp.id = id;

		// image name
		{
			auto it = parameters.find(group + "/image");
			REQUIRE(it != parameters.end());
			exp.name = it->second.Value<std::string>().value();
		}
		// ABW0..ABW6 x,y
		for (int i = 0; i < 7; i++) {
			std::string base = group + "/ABW" + std::to_string(i);
			auto itx         = parameters.find(base + "/x");
			auto ity         = parameters.find(base + "/y");
			REQUIRE(itx != parameters.end());
			REQUIRE(ity != parameters.end());
			exp.xs[static_cast<size_t>(i)] = itx->second.Value<double>().value();
			exp.ys[static_cast<size_t>(i)] = ity->second.Value<double>().value();
		}

		map.emplace(id, exp);
	}

	return map;
}

} // namespace

TEST_SUITE("Test Big Snake dataset") {
	TEST_CASE("Parse 17 images from dataset and compare ABW points") {
		// Inputs expected via two environment variables to stay independent of repo assets:
		//  DATASET_DIR: directory containing 17 .tiff images
		//  DATASET_YAML: yaml file with expected ABW points keyed by id
		const char *dataset_dir_env  = std::getenv("DATASET_DIR");
		const char *dataset_yaml_env = std::getenv("DATASET_YAML");
		REQUIRE_MESSAGE(dataset_dir_env != nullptr, "DATASET_DIR not set");
		REQUIRE_MESSAGE(dataset_yaml_env != nullptr, "DATASET_YAML not set");

		const std::string dataset_dir  = dataset_dir_env;
		const std::string dataset_yaml = dataset_yaml_env;

		auto expectations = ReadExpectations(dataset_yaml);
		REQUIRE_MESSAGE(expectations.size() >= 17, "Dataset must contain at least 17 entries");

		// Build camera model from an installed calibration file
		// Use assets/scanner_calibration/LX31624160053.yaml and tests/configs/sil/configuration.yaml FOV
		{
			auto calib_yaml = Yaml::FromFile("assets/scanner_calibration/LX31624160053.yaml", "camera").value();
			auto calib_map  = calib_yaml->AsUnorderedMap();
			auto cfg_yaml   = Yaml::FromFile("tests/configs/sil/configuration.yaml", "").value();
			auto cfg_map    = cfg_yaml->AsUnorderedMap();

			TiltedPerspectiveCameraProperties camera_properties = TiltedPerspectiveCameraProperties::FromUnorderedMap(calib_map);
			camera_properties.config_fov.width                  = static_cast<int>(cfg_map.at("image_provider/fov/width").Value<int64_t>().value());
			camera_properties.config_fov.height                 = static_cast<int>(cfg_map.at("image_provider/fov/height").Value<int64_t>().value());
			camera_properties.config_fov.offset_x               = static_cast<int>(cfg_map.at("image_provider/fov/offset_x").Value<int64_t>().value());
			camera_properties.config_fov.offset_y               = static_cast<int>(cfg_map.at("image_provider/fov/offset_y").Value<int64_t>().value());

			auto camera_model = std::make_unique<TiltedPerspectiveCamera>(camera_properties);

			// Joint properties: use calibration fixture defaults in tests/configs/sil/configuration.yaml
			JointProperties jp{};
			jp.upper_joint_width           = cfg_map.at("calibration_fixture_joint_geometry/upper_joint_width").Value<double>().value();
			jp.left_max_surface_angle      = cfg_map.at("calibration_fixture_joint_geometry/left_max_surface_angle").Value<double>().value();
			jp.right_max_surface_angle     = cfg_map.at("calibration_fixture_joint_geometry/right_max_surface_angle").Value<double>().value();
			jp.left_joint_angle            = cfg_map.at("calibration_fixture_joint_geometry/left_joint_angle").Value<double>().value();
			jp.right_joint_angle           = cfg_map.at("calibration_fixture_joint_geometry/right_joint_angle").Value<double>().value();
			jp.groove_depth                = cfg_map.at("calibration_fixture_joint_geometry/groove_depth").Value<double>().value();
			jp.upper_joint_width_tolerance = 7.0;
			jp.surface_angle_tolerance     = 10.0 * static_cast<double>(EIGEN_PI) / 180.0;
			jp.groove_angle_tolerance      = 9.0 * static_cast<double>(EIGEN_PI) / 180.0;
			jp.offset_distance             = 3.0;

			ScannerConfigurationData scanner_config{.gray_minimum_top = 48, .gray_minimum_wall = 16, .gray_minimum_bottom = 48};

			BigSnake big_snake(jp, scanner_config, std::move(camera_model));

			// Iterate over ids 1..17 (ids must exist in YAML). For each, read the image and compare points.
			for (int id = 1; id <= 17; ++id) {
				auto it = expectations.find(id);
				REQUIRE_MESSAGE(it != expectations.end(), "Missing expectation for id");
				const auto &exp = it->second;

				const std::string image_path = dataset_dir + "/" + exp.name; // name is image filename
				auto grayscale_image          = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
				REQUIRE_MESSAGE(!grayscale_image.empty(), "Failed to read image");

				auto maybe_image = scanner::image::ImageBuilder::From(grayscale_image, exp.name, 0).Finalize();
				auto *image       = maybe_image.value().get();

				auto res = big_snake.Parse(*image, {}, {}, false, {});
				REQUIRE(res.has_value());

				auto parsed            = res.value();
				auto &profile          = std::get<0>(parsed);
				const double tolerance = 0.0002;
				for (int i = 0; i < 7; i++) {
					CHECK(std::abs(profile.points[i].x - exp.xs[static_cast<size_t>(i)]) < tolerance);
					CHECK(std::abs(profile.points[i].y - exp.ys[static_cast<size_t>(i)]) < tolerance);
				}
			}
		}
	}
}

