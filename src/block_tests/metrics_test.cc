#include <doctest/doctest.h>

#include <nlohmann/json_fwd.hpp>
#include <utility>

#include "helpers.h"
#include "web_hmi/web_hmi_json_helpers.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)

TEST_SUITE("Metrics") {
  TEST_CASE("metrics_collect") {
    TestFixture fixture;

    // Starting legacy calibration removed; keep metrics test for other metrics if needed

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
