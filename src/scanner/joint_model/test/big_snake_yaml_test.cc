#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <Eigen/Core>

#include "common/file/yaml.h"
#include "scanner/joint_model/big_snake.h"
#include "scanner/image/image_builder.h"
#include "scanner/image/tilted_perspective_camera.h"
#include "scanner/scanner_configuration.h"

using common::file::Yaml;
using scanner::image::ImageBuilder;
using scanner::image::TiltedPerspectiveCamera;
using scanner::image::TiltedPerspectiveCameraProperties;
using scanner::image::WorkspaceCoordinates;
using scanner::joint_model::ABWPoint;
using scanner::joint_model::ABWPoints;
using scanner::joint_model::BigSnake;
using scanner::joint_model::JointModelErrorCode;
using scanner::joint_model::JointProfile;
using scanner::joint_model::JointProperties;

namespace {

// Helper function to compare ABW points with tolerance
bool CompareABWPoints(const ABWPoints& actual, const ABWPoints& expected, double tolerance) {
  if (actual.size() != expected.size()) {
    MESSAGE("Size mismatch: actual=", actual.size(), " expected=", expected.size());
    return false;
  }
  
  bool all_match = true;
  for (size_t i = 0; i < actual.size(); ++i) {
    double dx = std::abs(actual[i].x - expected[i].x);
    double dy = std::abs(actual[i].y - expected[i].y);
    
    if (dx > tolerance || dy > tolerance) {
      MESSAGE("ABW", i, " mismatch: actual(", actual[i].x, ", ", actual[i].y, 
              ") vs expected(", expected[i].x, ", ", expected[i].y, 
              ") diff(", dx, ", ", dy, ")");
      all_match = false;
    }
  }
  return all_match;
}

// Helper function to create camera model from YAML config
std::unique_ptr<TiltedPerspectiveCamera> CreateCameraModelFromYaml(const Yaml::UnorderedMap& camera_config) {
  TiltedPerspectiveCameraProperties props;
  
  if (camera_config.count("focal_length")) {
    props.focal_length = camera_config.at("focal_length").Value<double>().value();
  }
  if (camera_config.count("pixel_size")) {
    props.pixel_size = camera_config.at("pixel_size").Value<double>().value();
  }
  if (camera_config.count("image_width")) {
    props.image_width = camera_config.at("image_width").Value<int>().value();
  }
  if (camera_config.count("image_height")) {
    props.image_height = camera_config.at("image_height").Value<int>().value();
  }
  if (camera_config.count("principal_point_x")) {
    props.principal_point_x = camera_config.at("principal_point_x").Value<int>().value();
  }
  if (camera_config.count("principal_point_y")) {
    props.principal_point_y = camera_config.at("principal_point_y").Value<int>().value();
  }
  if (camera_config.count("tilt_angle")) {
    props.tilt_angle = camera_config.at("tilt_angle").Value<double>().value();
  }
  if (camera_config.count("rotation_angle")) {
    props.rotation_angle = camera_config.at("rotation_angle").Value<double>().value();
  }
  
  // FOV configuration
  if (camera_config.count("fov")) {
    auto fov_config = camera_config.at("fov").AsUnorderedMap();
    if (fov_config.count("width")) {
      props.config_fov.width = fov_config.at("width").Value<int>().value();
    }
    if (fov_config.count("height")) {
      props.config_fov.height = fov_config.at("height").Value<int>().value();
    }
    if (fov_config.count("offset_x")) {
      props.config_fov.offset_x = fov_config.at("offset_x").Value<int>().value();
    }
    if (fov_config.count("offset_y")) {
      props.config_fov.offset_y = fov_config.at("offset_y").Value<int>().value();
    }
  }
  
  return std::make_unique<TiltedPerspectiveCamera>(props);
}

// Helper function to create joint properties from YAML config
JointProperties CreateJointPropertiesFromYaml(const Yaml::UnorderedMap& joint_config) {
  JointProperties props;
  
  if (joint_config.count("upper_joint_width")) {
    props.upper_joint_width = joint_config.at("upper_joint_width").Value<double>().value();
  }
  if (joint_config.count("left_max_surface_angle")) {
    props.left_max_surface_angle = joint_config.at("left_max_surface_angle").Value<double>().value() * M_PI / 180.0;
  }
  if (joint_config.count("right_max_surface_angle")) {
    props.right_max_surface_angle = joint_config.at("right_max_surface_angle").Value<double>().value() * M_PI / 180.0;
  }
  if (joint_config.count("left_joint_angle")) {
    props.left_joint_angle = joint_config.at("left_joint_angle").Value<double>().value() * M_PI / 180.0;
  }
  if (joint_config.count("right_joint_angle")) {
    props.right_joint_angle = joint_config.at("right_joint_angle").Value<double>().value() * M_PI / 180.0;
  }
  if (joint_config.count("groove_depth")) {
    props.groove_depth = joint_config.at("groove_depth").Value<double>().value();
  }
  if (joint_config.count("upper_joint_width_tolerance")) {
    props.upper_joint_width_tolerance = joint_config.at("upper_joint_width_tolerance").Value<double>().value();
  }
  if (joint_config.count("surface_angle_tolerance")) {
    props.surface_angle_tolerance = joint_config.at("surface_angle_tolerance").Value<double>().value() * M_PI / 180.0;
  }
  if (joint_config.count("groove_angle_tolerance")) {
    props.groove_angle_tolerance = joint_config.at("groove_angle_tolerance").Value<double>().value() * M_PI / 180.0;
  }
  if (joint_config.count("offset_distance")) {
    props.offset_distance = joint_config.at("offset_distance").Value<double>().value();
  }
  
  return props;
}

// Helper function to create scanner config from YAML
scanner::ScannerConfigurationData CreateScannerConfigFromYaml(const Yaml::UnorderedMap& scanner_config) {
  scanner::ScannerConfigurationData config;
  
  if (scanner_config.count("gray_minimum_wall")) {
    config.gray_minimum_wall = scanner_config.at("gray_minimum_wall").Value<int>().value();
  }
  if (scanner_config.count("gray_minimum_groove")) {
    config.gray_minimum_groove = scanner_config.at("gray_minimum_groove").Value<int>().value();
  }
  if (scanner_config.count("gray_minimum_surface")) {
    config.gray_minimum_surface = scanner_config.at("gray_minimum_surface").Value<int>().value();
  }
  
  return config;
}

// Helper function to parse ABW points from YAML
ABWPoints ParseABWPointsFromYaml(const Yaml::UnorderedMap& abw_config) {
  ABWPoints points;
  points.reserve(15);
  
  for (int i = 0; i < 15; ++i) {
    std::string key = "ABW" + std::to_string(i);
    if (abw_config.count(key)) {
      auto point_config = abw_config.at(key).AsUnorderedMap();
      ABWPoint point;
      point.x = point_config.at("x").Value<double>().value();
      point.y = point_config.at("y").Value<double>().value();
      points.push_back(point);
    } else {
      // Default to (0, 0) for missing points
      points.push_back({0.0, 0.0});
    }
  }
  
  return points;
}

} // anonymous namespace

TEST_SUITE("BigSnake::Parse with YAML data") {
  
  TEST_CASE("Parse test images from YAML dataset") {
    // Load the test data YAML file
    std::string yaml_path = "./src/scanner/joint_model/test/test_data/data_set.yaml";
    
    if (!std::filesystem::exists(yaml_path)) {
      MESSAGE("Warning: Test data YAML file not found: ", yaml_path);
      return;
    }
    
    auto maybe_yaml = Yaml::FromFile(yaml_path);
    if (maybe_yaml.has_error()) {
      MESSAGE("Failed to load YAML file: ", maybe_yaml.error().to_string());
      return;
    }
    
    auto yaml_data = maybe_yaml.value()->AsUnorderedMap();
    
    // Get camera calibration
    std::unique_ptr<TiltedPerspectiveCamera> camera_model;
    if (yaml_data.count("camera_calibration")) {
      auto camera_config = yaml_data.at("camera_calibration").AsUnorderedMap();
      camera_model = CreateCameraModelFromYaml(camera_config);
    } else {
      MESSAGE("Warning: No camera calibration found in YAML");
      return;
    }
    
    // Process each test case
    if (yaml_data.count("test_cases")) {
      auto test_cases = yaml_data.at("test_cases").AsVector();
      
      for (const auto& test_case_yaml : test_cases) {
        auto test_case = test_case_yaml.AsUnorderedMap();
        
        std::string test_name = test_case.at("name").Value<std::string>().value();
        std::string image_name = test_case.at("image").Value<std::string>().value();
        std::string description = test_case.at("description").Value<std::string>().value();
        
        SUBCASE(test_name.c_str()) {
          MESSAGE("Testing: ", description);
          MESSAGE("Image: ", image_name);
          
          // Build full image path
          std::string image_path = "./src/scanner/joint_model/test/test_data/" + image_name;
          
          if (!std::filesystem::exists(image_path)) {
            MESSAGE("Warning: Test image not found: ", image_path);
            continue;
          }
          
          // Load the test image
          cv::Mat grayscale_image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
          if (grayscale_image.empty()) {
            MESSAGE("Failed to load image: ", image_path);
            continue;
          }
          
          // Create image object
          auto maybe_image = ImageBuilder::From(grayscale_image, image_name, 0).Finalize();
          REQUIRE(maybe_image.has_value());
          auto image = maybe_image.value();
          
          // Get joint properties
          auto joint_props_config = test_case.at("joint_properties").AsUnorderedMap();
          auto joint_properties = CreateJointPropertiesFromYaml(joint_props_config);
          
          // Get scanner config
          auto scanner_config_yaml = test_case.at("scanner_config").AsUnorderedMap();
          auto scanner_config = CreateScannerConfigFromYaml(scanner_config_yaml);
          
          // Create BigSnake instance
          BigSnake big_snake(joint_properties, scanner_config, camera_model->Clone());
          
          // Call Parse method
          auto result = big_snake.Parse(*image, 
                                       std::nullopt,  // No median profile
                                       std::nullopt,  // No updated properties
                                       false,         // Don't use approximation
                                       std::nullopt); // No horizontal hints
          
          // Check expected result
          auto expected_result = test_case.at("expected_result").AsUnorderedMap();
          bool expected_success = expected_result.at("success").Value<bool>().value();
          
          if (expected_success) {
            REQUIRE(result.has_value());
            
            if (result.has_value()) {
              auto [joint_profile, workspace_coords, time1, time2] = result.value();
              
              // Check ABW points if provided
              if (expected_result.count("abw_points")) {
                auto abw_config = expected_result.at("abw_points").AsUnorderedMap();
                auto expected_abw_points = ParseABWPointsFromYaml(abw_config);
                double tolerance = expected_result.at("tolerance").Value<double>().value();
                
                CHECK(CompareABWPoints(joint_profile.points, expected_abw_points, tolerance));
              }
              
              // Additional checks
              CHECK(joint_profile.points.size() == 15);
              CHECK(time1 > 0);
              CHECK(time2 > 0);
              
              MESSAGE("Processing times: ", time1, " us, ", time2, " us");
            }
          } else {
            REQUIRE(!result.has_value());
            
            if (!result.has_value() && expected_result.count("error_code")) {
              std::string expected_error = expected_result.at("error_code").Value<std::string>().value();
              MESSAGE("Expected error: ", expected_error);
              
              // Map string error codes to enum values
              if (expected_error == "NO_LASER_STRIPE") {
                CHECK(result.error() == JointModelErrorCode::NO_LASER_STRIPE);
              } else if (expected_error == "INVALID_JOINT") {
                CHECK(result.error() == JointModelErrorCode::INVALID_JOINT);
              }
              // Add more error code mappings as needed
            }
          }
        }
      }
    }
  }
  
  TEST_CASE("Batch process all images in test_data folder") {
    std::string test_data_dir = "./src/scanner/joint_model/test/test_data";
    
    if (!std::filesystem::exists(test_data_dir)) {
      MESSAGE("Test data directory not found: ", test_data_dir);
      return;
    }
    
    // Create default configurations
    TiltedPerspectiveCameraProperties camera_props;
    camera_props.focal_length = 35.0;
    camera_props.pixel_size = 0.00345;
    camera_props.image_width = 4096;
    camera_props.image_height = 3000;
    camera_props.principal_point_x = 2048;
    camera_props.principal_point_y = 1500;
    camera_props.config_fov.width = 3500;
    camera_props.config_fov.height = 2500;
    camera_props.config_fov.offset_x = 312;
    camera_props.config_fov.offset_y = 0;
    
    auto camera_model = std::make_unique<TiltedPerspectiveCamera>(camera_props);
    
    JointProperties joint_properties = {
      .upper_joint_width = 30.0,
      .left_max_surface_angle = 10.0 * M_PI / 180.0,
      .right_max_surface_angle = 10.0 * M_PI / 180.0,
      .left_joint_angle = 45.0 * M_PI / 180.0,
      .right_joint_angle = 45.0 * M_PI / 180.0,
      .groove_depth = 15.0,
      .upper_joint_width_tolerance = 7.0,
      .surface_angle_tolerance = 10.0 * M_PI / 180.0,
      .groove_angle_tolerance = 9.0 * M_PI / 180.0,
      .offset_distance = 3.0
    };
    
    scanner::ScannerConfigurationData scanner_config = {
      .gray_minimum_wall = 48,
      .gray_minimum_groove = 16,
      .gray_minimum_surface = 48
    };
    
    // Process all TIFF images in the directory
    int processed_count = 0;
    int success_count = 0;
    
    for (const auto& entry : std::filesystem::directory_iterator(test_data_dir)) {
      if (entry.is_regular_file()) {
        auto path = entry.path();
        if (path.extension() == ".tiff" || path.extension() == ".tif") {
          std::string filename = path.filename().string();
          
          MESSAGE("Processing: ", filename);
          
          cv::Mat grayscale_image = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
          if (!grayscale_image.empty()) {
            auto maybe_image = ImageBuilder::From(grayscale_image, filename, 0).Finalize();
            if (maybe_image.has_value()) {
              auto image = maybe_image.value();
              
              BigSnake big_snake(joint_properties, scanner_config, camera_model->Clone());
              auto result = big_snake.Parse(*image, std::nullopt, std::nullopt, false, std::nullopt);
              
              processed_count++;
              if (result.has_value()) {
                success_count++;
                auto [joint_profile, workspace_coords, time1, time2] = result.value();
                MESSAGE("  Success - Found ", joint_profile.points.size(), " ABW points");
                MESSAGE("  Processing time: ", time1, " us");
              } else {
                MESSAGE("  Failed with error code: ", static_cast<int>(result.error()));
              }
            }
          }
        }
      }
    }
    
    MESSAGE("Processed ", processed_count, " images, ", success_count, " successful");
    CHECK(processed_count > 0);
  }
}