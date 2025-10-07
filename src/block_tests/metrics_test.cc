#include <doctest/doctest.h>

#include <nlohmann/json_fwd.hpp>
#include <utility>

#include "helpers.h"
#include "web_hmi/web_hmi_json_helpers.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)

TEST_SUITE("Metrics") {
  TEST_CASE("metrics_collect") {
    TestFixture fixture;
    fixture.StartApplication();

    // Starting calibration will increase counter
    nlohmann::json payload({
        {"offset",   40.0},
        {"angle",    0.4 },
        {"stickout", 20.0}
    });
    auto start_cal = web_hmi::CreateMessage("LaserToTorchCalibration", payload);
    fixture.WebHmiIn()->DispatchMessage(std::move(start_cal));

    // Collect metrics and check counter
    auto collected = fixture.Sut()->Registry()->Collect();
    double laser_torch_calibration_start_count{};
    bool found_joint_metrics_family = false;
    for (auto const& metric_family : collected) {
      if (metric_family.name == "adaptio_laser_torch_calibration_starts_total") {
        laser_torch_calibration_start_count = metric_family.metric[0].counter.value;
      }
      if (metric_family.name == "weld_control_joint_top_width_mm" ||
          metric_family.name == "weld_control_joint_bottom_width_mm" ||
          metric_family.name == "weld_control_joint_left_depth_mm" ||
          metric_family.name == "weld_control_joint_right_depth_mm" ||
          metric_family.name == "weld_control_joint_avg_depth_mm" ||
          metric_family.name == "weld_control_joint_top_height_diff_mm" ||
          metric_family.name == "weld_control_joint_top_slope" ||
          metric_family.name == "weld_control_joint_bottom_slope") {
        found_joint_metrics_family = true;
      }
    }
    CHECK_EQ(1.0, laser_torch_calibration_start_count);
    CHECK(found_joint_metrics_family);
  }
}
// NOLINTEND(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)
