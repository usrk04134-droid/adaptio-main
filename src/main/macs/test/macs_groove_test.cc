#include "macs/macs_groove.h"

#include <doctest/doctest.h>

#include <optional>

// NOLINTBEGIN(*-magic-numbers, readability-function-cognitive-complexity)

namespace {
struct Test {
  macs::Groove groove;
  std::optional<double> area;
  std::optional<double> top_width;
  std::optional<double> bottom_width;
  std::optional<double> left_depth;
  std::optional<double> right_depth;
  std::optional<double> avg_depth;
  std::optional<double> top_slope;
  std::optional<double> bottom_slope;
  std::optional<double> left_wall_angle;
  std::optional<double> right_wall_angle;
};

void Check(const Test& test) {
  if (test.area) {
    REQUIRE(test.groove.Area() == doctest::Approx(test.area.value()));
  }

  if (test.top_width) {
    REQUIRE(test.groove.TopWidth() == doctest::Approx(test.top_width.value()));
  }

  if (test.bottom_width) {
    REQUIRE(test.groove.BottomWidth() == doctest::Approx(test.bottom_width.value()));
  }

  if (test.left_depth) {
    REQUIRE(test.groove.LeftDepth() == doctest::Approx(test.left_depth.value()));
  }

  if (test.right_depth) {
    REQUIRE(test.groove.RightDepth() == doctest::Approx(test.right_depth.value()));
  }

  if (test.avg_depth) {
    REQUIRE(test.groove.AvgDepth() == doctest::Approx(test.avg_depth.value()));
  }

  if (test.top_slope) {
    REQUIRE(test.groove.TopSlope() == doctest::Approx(test.top_slope.value()));
  }

  if (test.bottom_slope) {
    REQUIRE(test.groove.BottomSlope() == doctest::Approx(test.bottom_slope.value()));
  }

  if (test.left_wall_angle) {
    REQUIRE(test.groove.LeftWallAngle() == doctest::Approx(test.left_wall_angle.value()));
  }

  if (test.right_wall_angle) {
    REQUIRE(test.groove.RightWallAngle() == doctest::Approx(test.right_wall_angle.value()));
  }
}

}  // namespace

namespace macs {

TEST_SUITE("MacsGroove") {
  TEST_CASE("Unitialized") {
    Check(Test{
        .groove       = Groove(),
        .area         = 0.0,
        .top_width    = 0.0,
        .bottom_width = 0.0,
        .left_depth   = 0.0,
        .right_depth  = 0.0,
        .avg_depth    = 0.0,
    });
  }

  TEST_CASE("Groove1") {
    // Top line horizontal
    // Bottom line horizontal
    Check(Test{
        .groove           = Groove({.horizontal = 75., .vertical = 25.}, {.horizontal = 25., .vertical = -25.},
                                   {.horizontal = 12.5, .vertical = -25.}, {.horizontal = 0., .vertical = -25.},
                                   {.horizontal = -12.5, .vertical = -25.}, {.horizontal = -25, .vertical = -25.},
                                   {.horizontal = -75, .vertical = 25.}),
        .area             = 5000.0,
        .top_width        = 150.0,
        .bottom_width     = 50.0,
        .left_depth       = 50.0,
        .right_depth      = 50.0,
        .avg_depth        = 50.0,
        .top_slope        = 0.0,
        .bottom_slope     = 0.0,
        .left_wall_angle  = 0.7854,  // 45.0 deg
        .right_wall_angle = 0.7854,  // 45.0 deg
    });
  }

  TEST_CASE("Groove2") {
    // Top line upward slope
    // Bottom line horizontal
    Check(Test{
        .groove           = Groove({.horizontal = 75., .vertical = 25.}, {.horizontal = 25., .vertical = -25.},
                                   {.horizontal = 12.5, .vertical = -25.}, {.horizontal = 0., .vertical = -25.},
                                   {.horizontal = -12.5, .vertical = -25.}, {.horizontal = -25, .vertical = -25.},
                                   {.horizontal = -75, .vertical = 50.}),
        .area             = 6250.0,
        .top_width        = 150.0,
        .bottom_width     = 50.0,
        .left_depth       = 50.0,
        .right_depth      = 75.0,
        .avg_depth        = 62.5,
        .top_slope        = 0.166667,
        .bottom_slope     = 0.0,
        .left_wall_angle  = 0.7854,    // 45.0 deg
        .right_wall_angle = 0.588005,  // 33.7 deg
    });
  }

  TEST_CASE("Groove3") {
    // Top line downward slope
    // Bottom line horizontal
    Check(Test{
        .groove           = Groove({.horizontal = 75., .vertical = 25.}, {.horizontal = 25., .vertical = -25.},
                                   {.horizontal = 12.5, .vertical = -25.}, {.horizontal = 0., .vertical = -25.},
                                   {.horizontal = -12.5, .vertical = -25.}, {.horizontal = -25, .vertical = -25.},
                                   {.horizontal = -75, .vertical = 10.}),
        .area             = 4250.0,
        .top_width        = 150.0,
        .bottom_width     = 50.0,
        .left_depth       = 50.0,
        .right_depth      = 35.0,
        .avg_depth        = 42.5,
        .top_slope        = -0.1,
        .bottom_slope     = 0.0,
        .left_wall_angle  = 0.7854,   // 45.0 deg
        .right_wall_angle = 0.96007,  // 55.0 deg
    });
  }

  TEST_CASE("Groove4") {
    // Top line horizontal
    // Bottom line downward slope
    Check(Test{
        .groove           = Groove({.horizontal = 75., .vertical = 25.}, {.horizontal = 25., .vertical = -25.},
                                   {.horizontal = 12.5, .vertical = -26.}, {.horizontal = 0., .vertical = -27.},
                                   {.horizontal = -12.5, .vertical = -28.}, {.horizontal = -25, .vertical = -29.},
                                   {.horizontal = -75, .vertical = 25.}),
        .area             = 5200.0,
        .top_width        = 150.0,
        .bottom_width     = 50.0,
        .left_depth       = 50.0,
        .right_depth      = 54.0,
        .avg_depth        = 52.0,
        .top_slope        = 0.0,
        .bottom_slope     = -0.08,
        .left_wall_angle  = 0.7854,    // 45.0 deg
        .right_wall_angle = 0.746956,  // 42.8 deg
    });
  }

  TEST_CASE("Groove5") {
    // Top line horizontal
    // Bottom line upward slope
    Check(Test{
        .groove           = Groove({.horizontal = 75., .vertical = 25.}, {.horizontal = 25., .vertical = -25.},
                                   {.horizontal = 12.5, .vertical = -20.}, {.horizontal = 0., .vertical = -15.},
                                   {.horizontal = -12.5, .vertical = -10.}, {.horizontal = -25, .vertical = -5.},
                                   {.horizontal = -75, .vertical = 25.}),
        .area             = 4000.0,
        .top_width        = 150.0,
        .bottom_width     = 50.0,
        .left_depth       = 50.0,
        .right_depth      = 30.0,
        .avg_depth        = 40.0,
        .top_slope        = 0.0,
        .bottom_slope     = 0.4,
        .left_wall_angle  = 0.7854,   // 45.0 deg
        .right_wall_angle = 1.03038,  // 59.0 deg
    });
  }

  TEST_CASE("Groove6") {
    // Top line upward slope
    // Bottom line upward slope (parallel with topline)
    Check(Test{
        .groove           = Groove({.horizontal = 75., .vertical = 0.}, {.horizontal = 25., .vertical = -10.},
                                   {.horizontal = 12.5, .vertical = -7.5}, {.horizontal = 0., .vertical = -5.},
                                   {.horizontal = -12.5, .vertical = -2.5}, {.horizontal = -25, .vertical = 0.},
                                   {.horizontal = -75, .vertical = 30.}),
        .area             = 2000.0,
        .top_width        = 150.0,
        .bottom_width     = 50.0,
        .left_depth       = 10.0,
        .right_depth      = 30.0,
        .avg_depth        = 20.0,
        .top_slope        = 0.2,
        .bottom_slope     = 0.2,
        .left_wall_angle  = 1.3734,   // 78.7 deg
        .right_wall_angle = 1.03038,  // 59.0 deg
    });
  }

  TEST_CASE("Groove7") {
    // Top downward slope
    // Bottom line upward slope
    Check(Test{
        .groove           = Groove({.horizontal = 75., .vertical = 25.}, {.horizontal = 25., .vertical = -25.},
                                   {.horizontal = 12.5, .vertical = -20.}, {.horizontal = 0., .vertical = -15.},
                                   {.horizontal = -12.5, .vertical = -10.}, {.horizontal = -25, .vertical = -5.},
                                   {.horizontal = -75, .vertical = 15.}),
        .area             = 3500.0,
        .top_width        = 150.0,
        .bottom_width     = 50.0,
        .left_depth       = 50.0,
        .right_depth      = 20.0,
        .avg_depth        = 35.0,
        .top_slope        = -0.066667,
        .bottom_slope     = 0.4,
        .left_wall_angle  = 0.785398,  // 45.0 deg
        .right_wall_angle = 1.19029,   // 68.2 deg
    });
  }

  TEST_CASE("Groove8") {
    // Top upward slope
    // Bottom line downward slope
    Check(Test{
        .groove           = Groove({.horizontal = 75., .vertical = 15.}, {.horizontal = 25., .vertical = -10.},
                                   {.horizontal = 12.5, .vertical = -11.}, {.horizontal = 0., .vertical = -15.},
                                   {.horizontal = -12.5, .vertical = -30.}, {.horizontal = -25, .vertical = -35.},
                                   {.horizontal = -75, .vertical = 25.}),
        .area             = 4106.25,
        .top_width        = 150.0,
        .bottom_width     = 50.0,
        .left_depth       = 25.0,
        .right_depth      = 60.0,
        .avg_depth        = 42.5,
        .top_slope        = 0.066667,
        .bottom_slope     = -0.5,
        .left_wall_angle  = 1.10715,   // 63.4 deg
        .right_wall_angle = 0.694738,  // 39.8 deg
    });
  }
}

}  // namespace macs

// NOLINTEND(*-magic-numbers, readability-function-cognitive-complexity)
