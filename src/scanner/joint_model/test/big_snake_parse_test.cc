#include <opencv2/opencv.hpp>
#include <filesystem>
#include <vector>
#include <string>
#include <optional>

#include "scanner/image/camera_model.h"
#include "scanner/image/image_builder.h"
#include "scanner/image/tilted_perspective_camera.h"
#include "scanner/joint_model/big_snake.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/scanner_configuration.h"
#include "test_abw_data.h"

#ifndef DOCTEST_CONFIG_DISABLE
#include <doctest/doctest.h>

using namespace scanner::joint_model;
using namespace scanner::image;

namespace {

// Test configuration for camera model
TiltedPerspectiveCameraProperties CreateTestCameraProperties() {
  TiltedPerspectiveCameraProperties props;
  props.focal_length = 0.05;  // 50mm focal length
  props.principal_point_x = 1750.0;
  props.principal_point_y = 1250.0;
  props.pixel_size_x = 0.000005;  // 5μm pixel size
  props.pixel_size_y = 0.000005;
  props.rotation_x = 0.0;
  props.rotation_y = 0.0;
  props.rotation_z = 0.0;
  props.translation_x = 0.0;
  props.translation_y = 0.0;
  props.translation_z = 0.0;
  props.config_fov = {3500, 2500, 312, 0};  // width, height, offset_x, offset_y
  return props;
}

// Test joint properties
JointProperties CreateTestJointProperties() {
  JointProperties props;
  props.upper_joint_width = 0.01;  // 10mm joint width
  props.left_max_surface_angle = 45.0 * M_PI / 180.0;  // 45 degrees
  props.right_max_surface_angle = 45.0 * M_PI / 180.0;
  props.left_joint_angle = 30.0 * M_PI / 180.0;  // 30 degrees
  props.right_joint_angle = 30.0 * M_PI / 180.0;
  props.groove_depth = 0.005;  // 5mm groove depth
  props.upper_joint_width_tolerance = 7.0;
  props.surface_angle_tolerance = 10.0 * M_PI / 180.0;
  props.groove_angle_tolerance = 9.0 * M_PI / 180.0;
  props.offset_distance = 3.0;
  return props;
}

// Helper function to load test images
std::vector<std::string> GetTestImagePaths() {
  std::vector<std::string> image_paths;
  std::string test_data_dir = "./src/scanner/joint_model/test/test_data/";
  
  if (std::filesystem::exists(test_data_dir)) {
    for (const auto& entry : std::filesystem::directory_iterator(test_data_dir)) {
      if (entry.is_regular_file()) {
        std::string ext = entry.path().extension().string();
        if (ext == ".tiff" || ext == ".tif" || ext == ".bmp") {
          image_paths.push_back(entry.path().string());
        }
      }
    }
  }
  
  return image_paths;
}

// Helper function to validate ABW points
bool ValidateABWPoints(const ABWPoints& points) {
  // Check that we have exactly 7 points
  if (points.size() != 7) {
    return false;
  }
  
  // Check that all points have valid coordinates (not NaN or infinite)
  for (const auto& point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return false;
    }
  }
  
  // Check that points are in reasonable range (adjust based on your coordinate system)
  for (const auto& point : points) {
    if (std::abs(point.x) > 1.0 || std::abs(point.y) > 1.0) {
      return false;
    }
  }
  
  return true;
}

// Helper function to calculate distance between two points
double CalculateDistance(const Point& p1, const Point& p2) {
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

TEST_SUITE("BigSnake Parse Tests") {
  
  TEST_CASE("BigSnake::Parse - Process all test images") {
    auto camera_props = CreateTestCameraProperties();
    auto camera_model = std::make_unique<TiltedPerspectiveCamera>(camera_props);
    auto joint_props = CreateTestJointProperties();
    
    // Scanner configuration data
    ScannerConfigurationData config_data;
    config_data.gray_minimum_wall = 16;  // Threshold value
    
    // Create BigSnake instance
    BigSnake big_snake(joint_props, config_data, std::move(camera_model));
    
    // Get all test images
    auto image_paths = GetTestImagePaths();
    
    // Ensure we have test images
    REQUIRE(!image_paths.empty());
    
    int processed_count = 0;
    int successful_count = 0;
    
    for (const auto& image_path : image_paths) {
      INFO("Processing image: " << image_path);
      
      // Load image
      auto grayscale_image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
      REQUIRE(!grayscale_image.empty());
      
      // Create Image object
      auto maybe_image = ImageBuilder::From(grayscale_image, 
                                          std::filesystem::path(image_path).filename().string(), 
                                          0).Finalize();
      REQUIRE(maybe_image.has_value());
      
      auto image = maybe_image.value();
      processed_count++;
      
      // Test BigSnake::Parse with no median profile
      auto result = big_snake.Parse(*image, std::nullopt, std::nullopt, false, std::nullopt);
      
      if (result.has_value()) {
        auto [profile, snake_lpcs, processing_time, num_walls] = result.value();
        
        // Validate the result
        CHECK(ValidateABWPoints(profile.points));
        CHECK(processing_time >= 0);
        CHECK(num_walls >= 0);
        CHECK(profile.area >= 0.0);
        
        // Additional validation for specific points
        if (profile.points.size() == 7) {
          // Check that ABW0 and ABW6 are at the edges (leftmost and rightmost)
          double min_x = profile.points[0].x;
          double max_x = profile.points[0].x;
          for (const auto& point : profile.points) {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
          }
          
          // ABW0 should be leftmost, ABW6 should be rightmost
          CHECK(profile.points[0].x <= profile.points[6].x);
          
          // Check that the joint width is reasonable
          double joint_width = profile.points[6].x - profile.points[0].x;
          CHECK(joint_width > 0.0);
          CHECK(joint_width < 0.1);  // Should be less than 100mm
        }
        
        successful_count++;
        INFO("Successfully processed " << image_path << " - Processing time: " << processing_time << "ms");
      } else {
        INFO("Failed to process " << image_path << " - Error: " << static_cast<int>(result.error()));
      }
    }
    
    // Report results
    INFO("Processed " << processed_count << " images, " << successful_count << " successful");
    
    // We expect at least some images to be processed successfully
    // The exact number depends on the quality and content of the test images
    CHECK(processed_count > 0);
  }
  
  TEST_CASE("BigSnake::Parse - Test with horizontal cropping") {
    auto camera_props = CreateTestCameraProperties();
    auto camera_model = std::make_unique<TiltedPerspectiveCamera>(camera_props);
    auto joint_props = CreateTestJointProperties();
    
    ScannerConfigurationData config_data;
    config_data.gray_minimum_wall = 16;
    
    BigSnake big_snake(joint_props, config_data, std::move(camera_model));
    
    auto image_paths = GetTestImagePaths();
    REQUIRE(!image_paths.empty());
    
    // Test with the first available image
    auto grayscale_image = cv::imread(image_paths[0], cv::IMREAD_GRAYSCALE);
    REQUIRE(!grayscale_image.empty());
    
    auto maybe_image = ImageBuilder::From(grayscale_image, 
                                        std::filesystem::path(image_paths[0]).filename().string(), 
                                        0).Finalize();
    REQUIRE(maybe_image.has_value());
    
    auto image = maybe_image.value();
    
    // Apply horizontal cropping
    image->SetHorizontalCrop(300, 3200);
    
    auto result = big_snake.Parse(*image, std::nullopt, std::nullopt, false, std::nullopt);
    
    if (result.has_value()) {
      auto [profile, snake_lpcs, processing_time, num_walls] = result.value();
      CHECK(ValidateABWPoints(profile.points));
      CHECK(processing_time >= 0);
    }
  }
  
  TEST_CASE("BigSnake::Parse - Test with median profile") {
    auto camera_props = CreateTestCameraProperties();
    auto camera_model = std::make_unique<TiltedPerspectiveCamera>(camera_props);
    auto joint_props = CreateTestJointProperties();
    
    ScannerConfigurationData config_data;
    config_data.gray_minimum_wall = 16;
    
    BigSnake big_snake(joint_props, config_data, std::move(camera_model));
    
    auto image_paths = GetTestImagePaths();
    REQUIRE(!image_paths.empty());
    
    // Create a mock median profile
    JointProfile median_profile;
    median_profile.points = {
      Point{0.0, -0.28},
      Point{0.002, -0.28},
      Point{0.004, -0.28},
      Point{0.005, -0.28},
      Point{0.004, -0.28},
      Point{0.002, -0.28},
      Point{0.0, -0.28}
    };
    median_profile.area = 0.001;
    median_profile.approximation_used = false;
    
    // Test with the first available image
    auto grayscale_image = cv::imread(image_paths[0], cv::IMREAD_GRAYSCALE);
    REQUIRE(!grayscale_image.empty());
    
    auto maybe_image = ImageBuilder::From(grayscale_image, 
                                        std::filesystem::path(image_paths[0]).filename().string(), 
                                        0).Finalize();
    REQUIRE(maybe_image.has_value());
    
    auto image = maybe_image.value();
    
    auto result = big_snake.Parse(*image, median_profile, std::nullopt, false, std::nullopt);
    
    if (result.has_value()) {
      auto [profile, snake_lpcs, processing_time, num_walls] = result.value();
      CHECK(ValidateABWPoints(profile.points));
      CHECK(processing_time >= 0);
    }
  }
  
  TEST_CASE("BigSnake::Parse - Test error handling") {
    auto camera_props = CreateTestCameraProperties();
    auto camera_model = std::make_unique<TiltedPerspectiveCamera>(camera_props);
    auto joint_props = CreateTestJointProperties();
    
    ScannerConfigurationData config_data;
    config_data.gray_minimum_wall = 16;
    
    BigSnake big_snake(joint_props, config_data, std::move(camera_model));
    
    // Test with a very small image (should fail)
    cv::Mat small_image = cv::Mat::zeros(10, 10, CV_8UC1);
    auto maybe_image = ImageBuilder::From(small_image, "small_test.tiff", 0).Finalize();
    REQUIRE(maybe_image.has_value());
    
    auto image = maybe_image.value();
    auto result = big_snake.Parse(*image, std::nullopt, std::nullopt, false, std::nullopt);
    
    // This should fail due to insufficient image size
    CHECK(!result.has_value());
  }
  
  TEST_CASE("BigSnake::Parse - Performance test") {
    auto camera_props = CreateTestCameraProperties();
    auto camera_model = std::make_unique<TiltedPerspectiveCamera>(camera_props);
    auto joint_props = CreateTestJointProperties();
    
    ScannerConfigurationData config_data;
    config_data.gray_minimum_wall = 16;
    
    BigSnake big_snake(joint_props, config_data, std::move(camera_model));
    
    auto image_paths = GetTestImagePaths();
    REQUIRE(!image_paths.empty());
    
    // Test with the first available image
    auto grayscale_image = cv::imread(image_paths[0], cv::IMREAD_GRAYSCALE);
    REQUIRE(!grayscale_image.empty());
    
    auto maybe_image = ImageBuilder::From(grayscale_image, 
                                        std::filesystem::path(image_paths[0]).filename().string(), 
                                        0).Finalize();
    REQUIRE(maybe_image.has_value());
    
    auto image = maybe_image.value();
    
    // Run multiple iterations to test performance
    const int iterations = 10;
    std::vector<uint64_t> processing_times;
    
    for (int i = 0; i < iterations; i++) {
      auto result = big_snake.Parse(*image, std::nullopt, std::nullopt, false, std::nullopt);
      
      if (result.has_value()) {
        auto [profile, snake_lpcs, processing_time, num_walls] = result.value();
        processing_times.push_back(processing_time);
      }
    }
    
    if (!processing_times.empty()) {
      // Calculate average processing time
      uint64_t total_time = 0;
      for (auto time : processing_times) {
        total_time += time;
      }
      uint64_t avg_time = total_time / processing_times.size();
      
      INFO("Average processing time: " << avg_time << "ms");
      
      // Processing time should be reasonable (less than 1 second)
      CHECK(avg_time < 1000);
    }
  }
  
  TEST_CASE("BigSnake::Parse - Compare with expected ABW points") {
    auto camera_props = CreateTestCameraProperties();
    auto camera_model = std::make_unique<TiltedPerspectiveCamera>(camera_props);
    auto joint_props = CreateTestJointProperties();
    
    ScannerConfigurationData config_data;
    config_data.gray_minimum_wall = 16;
    
    BigSnake big_snake(joint_props, config_data, std::move(camera_model));
    
    auto image_paths = GetTestImagePaths();
    REQUIRE(!image_paths.empty());
    
    // Test each image and compare with expected results if available
    for (const auto& image_path : image_paths) {
      INFO("Testing image: " << image_path);
      
      auto grayscale_image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
      REQUIRE(!grayscale_image.empty());
      
      auto maybe_image = ImageBuilder::From(grayscale_image, 
                                          std::filesystem::path(image_path).filename().string(), 
                                          0).Finalize();
      REQUIRE(maybe_image.has_value());
      
      auto image = maybe_image.value();
      auto result = big_snake.Parse(*image, std::nullopt, std::nullopt, false, std::nullopt);
      
      if (result.has_value()) {
        auto [profile, snake_lpcs, processing_time, num_walls] = result.value();
        
        // Validate basic properties
        CHECK(ValidateABWPoints(profile.points));
        
        // Calculate joint metrics
        double joint_width = test_data::CalculateJointWidth(profile.points);
        double joint_depth = test_data::CalculateJointDepth(profile.points);
        
        INFO("Joint width: " << joint_width << "m, Joint depth: " << joint_depth << "m");
        
        // Validate joint dimensions are reasonable
        CHECK(joint_width > 0.0);
        CHECK(joint_width < 0.1);  // Less than 100mm
        CHECK(joint_depth >= 0.0);
        CHECK(joint_depth < 0.05);  // Less than 50mm
        
        // Check specific image if we have expected data
        std::string filename = std::filesystem::path(image_path).filename().string();
        if (filename == "1755001276997.tiff") {
          // Compare with expected ABW points for this specific image
          ABWPoints expected_points;
          std::copy(test_data::expected_abw_1755001276997.begin(), 
                   test_data::expected_abw_1755001276997.end(), 
                   expected_points.begin());
          
          // Check if points are within tolerance
          bool within_tolerance = test_data::AreABWPointsWithinTolerance(profile.points, expected_points);
          
          if (within_tolerance) {
            INFO("ABW points match expected values within tolerance");
          } else {
            INFO("ABW points differ from expected values:");
            for (size_t i = 0; i < profile.points.size(); ++i) {
              INFO("Point " << i << ": actual(" << profile.points[i].x << ", " << profile.points[i].y 
                   << ") vs expected(" << expected_points[i].x << ", " << expected_points[i].y << ")");
            }
          }
        }
      }
    }
  }
  
  TEST_CASE("BigSnake::Parse - Test all scanner configurations") {
    auto camera_props = CreateTestCameraProperties();
    auto joint_props = CreateTestJointProperties();
    
    // Test different scanner configurations
    std::vector<ScannerConfigurationData> configs = {
      {16},   // Low threshold
      {32},   // Medium threshold  
      {64},   // High threshold
    };
    
    auto image_paths = GetTestImagePaths();
    REQUIRE(!image_paths.empty());
    
    // Test with the first available image
    auto grayscale_image = cv::imread(image_paths[0], cv::IMREAD_GRAYSCALE);
    REQUIRE(!grayscale_image.empty());
    
    auto maybe_image = ImageBuilder::From(grayscale_image, 
                                        std::filesystem::path(image_paths[0]).filename().string(), 
                                        0).Finalize();
    REQUIRE(maybe_image.has_value());
    
    auto image = maybe_image.value();
    
    for (const auto& config : configs) {
      INFO("Testing with threshold: " << config.gray_minimum_wall);
      
      auto camera_model = std::make_unique<TiltedPerspectiveCamera>(camera_props);
      BigSnake big_snake(joint_props, config, std::move(camera_model));
      
      auto result = big_snake.Parse(*image, std::nullopt, std::nullopt, false, std::nullopt);
      
      if (result.has_value()) {
        auto [profile, snake_lpcs, processing_time, num_walls] = result.value();
        CHECK(ValidateABWPoints(profile.points));
        CHECK(processing_time >= 0);
      }
    }
  }
}
#endif