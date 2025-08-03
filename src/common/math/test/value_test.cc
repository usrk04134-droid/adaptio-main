
#include "common/math/value.h"

#include <doctest/doctest.h>

#include <array>

// NOLINTBEGIN(*-magic-numbers)
namespace common::math::value {

TEST_SUITE("Values") {
  TEST_CASE("Median value") {
    std::array<double, 6> arr_even_elem{6.0, 2.0, 5.0, 3.0, 4.0, 1.0};
    CHECK_EQ(FindMedian(arr_even_elem.begin(), arr_even_elem.end(), arr_even_elem.size()), 3.5);

    std::array<double, 5> arr_odd_elem{1.0, 2.0, 3.0, 4.0, 5.0};
    CHECK_EQ(FindMedian(arr_odd_elem.begin(), arr_odd_elem.end(), arr_odd_elem.size()), 3.0);
  }
  // TEST_CASE("Value in range") {
  //   CHECK_EQ(common::math::value::CheckIfValueInRange(0.97, 1.0, 0.05), true);
  //   CHECK_EQ(common::math::value::CheckIfValueInRange(1.01, 1.0, 0.05), true);
  //   CHECK_EQ(common::math::value::CheckIfValueInRange(1.09, 1.0, 0.05), false);
  //   CHECK_EQ(common::math::value::CheckIfValueInRange(0.94, 1.0, 0.05), false);
  // }
}

}  // namespace common::math::value
// NOLINTEND(*-magic-numbers)
