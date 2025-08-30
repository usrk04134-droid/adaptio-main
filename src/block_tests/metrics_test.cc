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
    for (auto const& metric_family : collected) {
      if (metric_family.name == "adaptio_laser_torch_calibration_starts_total") {
        laser_torch_calibration_start_count = metric_family.metric[0].counter.value;
        break;
      }
    }
    CHECK_EQ(1.0, laser_torch_calibration_start_count);
  }
}
// NOLINTEND(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)
