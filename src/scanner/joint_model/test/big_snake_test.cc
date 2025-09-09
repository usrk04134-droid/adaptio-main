#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <Eigen/Core>

#include "scanner/joint_model/big_snake.h"
#include "scanner/image/image_builder.h"
#include "scanner/image/tilted_perspective_camera.h"
#include "scanner/scanner_configuration.h"

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

// Test data structure for each test image
struct TestImageData {
  std::string image_path;
  std::string description;
  ABWPoints expected_abw_points;
  double tolerance;  // Tolerance for comparing ABW points
  bool should_succeed;
  std::optional<JointModelErrorCode> expected_error;
};

// Helper function to compare ABW points with tolerance
bool CompareABWPoints(const ABWPoints& actual, const ABWPoints& expected, double tolerance) {
  if (actual.size() != expected.size()) {
    return false;
  }
  
  for (size_t i = 0; i < actual.size(); ++i) {
    double dx = std::abs(actual[i].x - expected[i].x);
    double dy = std::abs(actual[i].y - expected[i].y);
    
    if (dx > tolerance || dy > tolerance) {
      MESSAGE("ABW point ", i, " mismatch: actual(", actual[i].x, ", ", actual[i].y, 
              ") vs expected(", expected[i].x, ", ", expected[i].y, ")");
      return false;
    }
  }
  return true;
}

// Helper function to create camera model for testing
std::unique_ptr<TiltedPerspectiveCamera> CreateTestCameraModel() {
  // Default camera properties for testing
  // These should match the configuration used in the original test setup
  TiltedPerspectiveCameraProperties props;
  
  // Camera calibration parameters (example values - adjust based on actual calibration)
  props.focal_length = 35.0;  // mm
  props.pixel_size = 0.00345;  // mm/pixel
  props.image_width = 4096;
  props.image_height = 3000;
  props.principal_point_x = 2048;
  props.principal_point_y = 1500;
  
  // Tilt and perspective parameters
  props.tilt_angle = 0.0;  // degrees
  props.rotation_angle = 0.0;  // degrees
  
  // Field of view configuration
  props.config_fov.width = 3500;
  props.config_fov.height = 2500;
  props.config_fov.offset_x = 312;
  props.config_fov.offset_y = 0;
  
  return std::make_unique<TiltedPerspectiveCamera>(props);
}

// Test data for different images
std::vector<TestImageData> GetTestData() {
  std::vector<TestImageData> test_data;
  
  // Test case 1: Normal V-joint image
  test_data.push_back({
    .image_path = "./src/scanner/joint_model/test/test_data/1755001276997.tiff",
    .description = "Normal V-joint with clear laser line",
    .expected_abw_points = {
      {-15.5, 8.2},   // ABW0 - left top edge
      {-14.8, 7.5},   // ABW1 - left wall upper
      {-13.2, 5.8},   // ABW2 - left wall lower
      {-12.0, 4.5},   // ABW3 - bottom left
      {0.0, 0.0},     // ABW4 - bottom center (root)
      {12.0, 4.5},    // ABW5 - bottom right
      {13.2, 5.8},    // ABW6 - right wall lower
      {14.8, 7.5},    // ABW7 - right wall upper
      {15.5, 8.2},    // ABW8 - right top edge
      {0.0, 0.0},     // ABW9 - reserved
      {0.0, 0.0},     // ABW10 - reserved
      {0.0, 0.0},     // ABW11 - reserved
      {0.0, 0.0},     // ABW12 - reserved
      {0.0, 0.0},     // ABW13 - reserved
      {0.0, 0.0}      // ABW14 - reserved
    },
    .tolerance = 0.5,  // 0.5mm tolerance
    .should_succeed = true,
    .expected_error = std::nullopt
  });
  
  // Test case 2: Black/dark image (should fail)
  test_data.push_back({
    .image_path = "./src/scanner/joint_model/test/test_data/1754561083373.tiff",
    .description = "Black image with no visible laser line",
    .expected_abw_points = {},  // Empty as it should fail
    .tolerance = 0.5,
    .should_succeed = false,
    .expected_error = JointModelErrorCode::NO_LASER_STRIPE
  });
  
  // Add more test cases as needed when more images are available
  
  return test_data;
}

// Default joint properties for testing
JointProperties GetDefaultJointProperties() {
  return {
    .upper_joint_width = 30.0,              // mm
    .left_max_surface_angle = 10.0 * M_PI / 180.0,   // radians
    .right_max_surface_angle = 10.0 * M_PI / 180.0,  // radians
    .left_joint_angle = 45.0 * M_PI / 180.0,         // radians
    .right_joint_angle = 45.0 * M_PI / 180.0,        // radians
    .groove_depth = 15.0,                    // mm
    .upper_joint_width_tolerance = 7.0,     // mm
    .surface_angle_tolerance = 10.0 * M_PI / 180.0,  // radians
    .groove_angle_tolerance = 9.0 * M_PI / 180.0,    // radians
    .offset_distance = 3.0                  // mm
  };
}

// Scanner configuration data for testing
scanner::ScannerConfigurationData GetDefaultScannerConfig() {
  return {
    .gray_minimum_wall = 48,
    .gray_minimum_groove = 16,
    .gray_minimum_surface = 48
  };
}

} // anonymous namespace

TEST_SUITE("BigSnake::Parse") {
  
  TEST_CASE("Parse multiple test images and verify ABW points") {
    auto test_data = GetTestData();
    auto camera_model = CreateTestCameraModel();
    auto joint_properties = GetDefaultJointProperties();
    auto scanner_config = GetDefaultScannerConfig();
    
    for (const auto& test_case : test_data) {
      SUBCASE(test_case.description.c_str()) {
        MESSAGE("Testing image: ", test_case.image_path);
        
        // Check if image file exists
        if (!std::filesystem::exists(test_case.image_path)) {
          MESSAGE("Warning: Test image not found: ", test_case.image_path);
          continue;
        }
        
        // Load the test image
        cv::Mat grayscale_image = cv::imread(test_case.image_path, cv::IMREAD_GRAYSCALE);
        REQUIRE(!grayscale_image.empty());
        
        // Create image object
        auto maybe_image = ImageBuilder::From(grayscale_image, 
                                             std::filesystem::path(test_case.image_path).filename().string(), 
                                             0).Finalize();
        REQUIRE(maybe_image.has_value());
        auto image = maybe_image.value();
        
        // Create BigSnake instance
        BigSnake big_snake(joint_properties, scanner_config, camera_model->Clone());
        
        // Call Parse method
        auto result = big_snake.Parse(*image, 
                                     std::nullopt,  // No median profile
                                     std::nullopt,  // No updated properties
                                     false,         // Don't use approximation
                                     std::nullopt); // No horizontal hints
        
        if (test_case.should_succeed) {
          REQUIRE(result.has_value());
          
          auto [joint_profile, workspace_coords, time1, time2] = result.value();
          
          // Verify ABW points
          if (!test_case.expected_abw_points.empty()) {
            CHECK(CompareABWPoints(joint_profile.points, test_case.expected_abw_points, test_case.tolerance));
          }
          
          // Additional checks
          CHECK(joint_profile.points.size() == 15);  // Should always have 15 ABW points
          CHECK(time1 > 0);  // Processing time should be positive
          CHECK(time2 > 0);  // Processing time should be positive
          
        } else {
          REQUIRE(!result.has_value());
          if (test_case.expected_error.has_value()) {
            CHECK(result.error() == test_case.expected_error.value());
          }
        }
      }
    }
  }
  
  TEST_CASE("Parse with median profile") {
    auto camera_model = CreateTestCameraModel();
    auto joint_properties = GetDefaultJointProperties();
    auto scanner_config = GetDefaultScannerConfig();
    
    // Load test image
    std::string image_path = "./src/scanner/joint_model/test/test_data/1755001276997.tiff";
    if (!std::filesystem::exists(image_path)) {
      MESSAGE("Warning: Test image not found: ", image_path);
      return;
    }
    
    cv::Mat grayscale_image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    REQUIRE(!grayscale_image.empty());
    
    auto maybe_image = ImageBuilder::From(grayscale_image, "1755001276997.tiff", 0).Finalize();
    REQUIRE(maybe_image.has_value());
    auto image = maybe_image.value();
    
    BigSnake big_snake(joint_properties, scanner_config, camera_model->Clone());
    
    // First parse without median profile
    auto result1 = big_snake.Parse(*image, std::nullopt, std::nullopt, false, std::nullopt);
    REQUIRE(result1.has_value());
    
    auto [joint_profile1, workspace_coords1, time1_1, time2_1] = result1.value();
    
    // Second parse with the first result as median profile
    auto result2 = big_snake.Parse(*image, joint_profile1, std::nullopt, false, std::nullopt);
    REQUIRE(result2.has_value());
    
    auto [joint_profile2, workspace_coords2, time1_2, time2_2] = result2.value();
    
    // Results should be similar but potentially refined with median profile
    CHECK(CompareABWPoints(joint_profile2.points, joint_profile1.points, 1.0));
  }
  
  TEST_CASE("Parse with updated joint properties") {
    auto camera_model = CreateTestCameraModel();
    auto joint_properties = GetDefaultJointProperties();
    auto scanner_config = GetDefaultScannerConfig();
    
    // Load test image
    std::string image_path = "./src/scanner/joint_model/test/test_data/1755001276997.tiff";
    if (!std::filesystem::exists(image_path)) {
      MESSAGE("Warning: Test image not found: ", image_path);
      return;
    }
    
    cv::Mat grayscale_image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    REQUIRE(!grayscale_image.empty());
    
    auto maybe_image = ImageBuilder::From(grayscale_image, "1755001276997.tiff", 0).Finalize();
    REQUIRE(maybe_image.has_value());
    auto image = maybe_image.value();
    
    BigSnake big_snake(joint_properties, scanner_config, camera_model->Clone());
    
    // Create updated properties with tighter tolerance
    auto updated_properties = joint_properties;
    updated_properties.upper_joint_width_tolerance = 4.0;
    
    // Parse with updated properties
    auto result = big_snake.Parse(*image, std::nullopt, updated_properties, false, std::nullopt);
    
    // Should still succeed with valid image
    CHECK(result.has_value());
  }
  
  TEST_CASE("Parse with approximation mode") {
    auto camera_model = CreateTestCameraModel();
    auto joint_properties = GetDefaultJointProperties();
    auto scanner_config = GetDefaultScannerConfig();
    
    // Load test image
    std::string image_path = "./src/scanner/joint_model/test/test_data/1755001276997.tiff";
    if (!std::filesystem::exists(image_path)) {
      MESSAGE("Warning: Test image not found: ", image_path);
      return;
    }
    
    cv::Mat grayscale_image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    REQUIRE(!grayscale_image.empty());
    
    auto maybe_image = ImageBuilder::From(grayscale_image, "1755001276997.tiff", 0).Finalize();
    REQUIRE(maybe_image.has_value());
    auto image = maybe_image.value();
    
    BigSnake big_snake(joint_properties, scanner_config, camera_model->Clone());
    
    // Parse with approximation enabled
    auto result = big_snake.Parse(*image, std::nullopt, std::nullopt, true, std::nullopt);
    
    CHECK(result.has_value());
    
    if (result.has_value()) {
      auto [joint_profile, workspace_coords, time1, time2] = result.value();
      CHECK(joint_profile.points.size() == 15);
    }
  }
  
  TEST_CASE("Parse with horizontal hints") {
    auto camera_model = CreateTestCameraModel();
    auto joint_properties = GetDefaultJointProperties();
    auto scanner_config = GetDefaultScannerConfig();
    
    // Load test image
    std::string image_path = "./src/scanner/joint_model/test/test_data/1755001276997.tiff";
    if (!std::filesystem::exists(image_path)) {
      MESSAGE("Warning: Test image not found: ", image_path);
      return;
    }
    
    cv::Mat grayscale_image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    REQUIRE(!grayscale_image.empty());
    
    auto maybe_image = ImageBuilder::From(grayscale_image, "1755001276997.tiff", 0).Finalize();
    REQUIRE(maybe_image.has_value());
    auto image = maybe_image.value();
    
    BigSnake big_snake(joint_properties, scanner_config, camera_model->Clone());
    
    // Provide horizontal hints for ABW0 and ABW6
    std::tuple<double, double> horizontal_hints = {-15.0, 15.0};
    
    // Parse with horizontal hints
    auto result = big_snake.Parse(*image, std::nullopt, std::nullopt, false, horizontal_hints);
    
    CHECK(result.has_value());
    
    if (result.has_value()) {
      auto [joint_profile, workspace_coords, time1, time2] = result.value();
      CHECK(joint_profile.points.size() == 15);
      
      // Check that ABW0 and ABW6 are influenced by the hints
      CHECK(joint_profile.points[0].x < -10.0);  // ABW0 should be on the left
      CHECK(joint_profile.points[6].x > 10.0);   // ABW6 should be on the right
    }
  }
  
  TEST_CASE("Performance test - processing time") {
    auto camera_model = CreateTestCameraModel();
    auto joint_properties = GetDefaultJointProperties();
    auto scanner_config = GetDefaultScannerConfig();
    
    // Load test image
    std::string image_path = "./src/scanner/joint_model/test/test_data/1755001276997.tiff";
    if (!std::filesystem::exists(image_path)) {
      MESSAGE("Warning: Test image not found: ", image_path);
      return;
    }
    
    cv::Mat grayscale_image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    REQUIRE(!grayscale_image.empty());
    
    auto maybe_image = ImageBuilder::From(grayscale_image, "1755001276997.tiff", 0).Finalize();
    REQUIRE(maybe_image.has_value());
    auto image = maybe_image.value();
    
    BigSnake big_snake(joint_properties, scanner_config, camera_model->Clone());
    
    // Run multiple iterations to measure performance
    const int iterations = 10;
    uint64_t total_time = 0;
    uint64_t max_time = 0;
    
    for (int i = 0; i < iterations; ++i) {
      auto result = big_snake.Parse(*image, std::nullopt, std::nullopt, false, std::nullopt);
      REQUIRE(result.has_value());
      
      auto [joint_profile, workspace_coords, time1, time2] = result.value();
      total_time += time1;
      max_time = std::max(max_time, time1);
    }
    
    uint64_t avg_time = total_time / iterations;
    
    MESSAGE("Average processing time: ", avg_time, " microseconds");
    MESSAGE("Maximum processing time: ", max_time, " microseconds");
    
    // Check that processing is reasonably fast (adjust threshold as needed)
    CHECK(avg_time < 50000);  // Less than 50ms on average
  }
}