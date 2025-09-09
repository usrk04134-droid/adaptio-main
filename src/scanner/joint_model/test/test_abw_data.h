#pragma once

#include <array>
#include "scanner/joint_model/joint_model.h"

namespace scanner::joint_model::test_data {

// Expected ABW points for test images (if available from old adaptio-core tests)
// These would be populated with actual expected values from the data_set.yaml
// For now, these are placeholder values that represent typical joint geometries

// Expected ABW points for 1755001276997.tiff (if available)
constexpr std::array<Point, 7> expected_abw_1755001276997 = {
    Point{0.0, -0.28},      // ABW0 - left edge
    Point{0.002, -0.28},    // ABW1 - left wall
    Point{0.003, -0.28},    // ABW2 - left groove
    Point{0.005, -0.28},    // ABW3 - bottom center
    Point{0.007, -0.28},    // ABW4 - right groove
    Point{0.008, -0.28},    // ABW5 - right wall
    Point{0.01, -0.28}      // ABW6 - right edge
};

// Expected ABW points for 1754561083373.tiff (if available)
// This image might be a black image or have no visible joint
constexpr std::array<Point, 7> expected_abw_1754561083373 = {
    Point{0.0, -0.28},      // ABW0
    Point{0.0, -0.28},      // ABW1
    Point{0.0, -0.28},      // ABW2
    Point{0.0, -0.28},      // ABW3
    Point{0.0, -0.28},      // ABW4
    Point{0.0, -0.28},      // ABW5
    Point{0.0, -0.28}       // ABW6
};

// Tolerance for ABW point comparison (in meters)
constexpr double ABW_POINT_TOLERANCE = 0.001;  // 1mm tolerance

// Helper function to check if two ABW points are within tolerance
bool AreABWPointsWithinTolerance(const ABWPoints& actual, const ABWPoints& expected, double tolerance = ABW_POINT_TOLERANCE) {
    if (actual.size() != expected.size()) {
        return false;
    }
    
    for (size_t i = 0; i < actual.size(); ++i) {
        double dx = actual[i].x - expected[i].x;
        double dy = actual[i].y - expected[i].y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance > tolerance) {
            return false;
        }
    }
    
    return true;
}

// Helper function to calculate joint width from ABW points
double CalculateJointWidth(const ABWPoints& points) {
    if (points.size() < 2) {
        return 0.0;
    }
    
    return points[6].x - points[0].x;  // ABW6.x - ABW0.x
}

// Helper function to calculate joint depth from ABW points
double CalculateJointDepth(const ABWPoints& points) {
    if (points.size() < 3) {
        return 0.0;
    }
    
    // Find the maximum depth (minimum y value)
    double max_depth = points[0].y;
    for (const auto& point : points) {
        max_depth = std::min(max_depth, point.y);
    }
    
    return -max_depth;  // Convert to positive depth
}

}  // namespace scanner::joint_model::test_data