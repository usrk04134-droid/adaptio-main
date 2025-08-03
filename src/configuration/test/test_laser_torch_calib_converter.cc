#include <doctest/doctest.h>

#include <any>
#include <memory>
#include <optional>
#include <string>
#include <trompeloeil.hpp>  // IWYU pragma: keep

#include "../config_manager.h"
#include "../configuration_error.h"
#include "../laser_torch_calib_converter.h"
#include "configuration/conf_factory.h"
#include "configuration/laser_torch_calib_converter.h"
#include "main/calibration/calibration_types.h"
#include "mock/mock_factory.h"
#include "mock/mock_file_handler.h"

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)

namespace configuration {

TEST_SUITE("Laser Torch Calibration Converter") {
  TEST_CASE("Convert") {
    std::string yaml = R"(
valid: false
x: 1.0
y: 2.0
z: 3.0
angle: 4.0)";

    MockFactory mock_factory;
    auto mock = [&]() { return &mock_factory; };
    configuration::SetFactoryGenerator(mock);

    auto mock_fh = std::make_shared<MockFileHander>();

    REQUIRE_CALL(mock_factory, CreateFileHandler()).RETURN(mock_fh);
    REQUIRE_CALL(*mock_fh, ReadFile("/adaptio/config/laser_torch_calibration.yaml")).RETURN(yaml);

    LaserTorchCalibConverter converter(TAG_LTC, "/adaptio/config/laser_torch_calibration.yaml");
    CHECK_EQ(converter.ReadPersistentData(), boost::outcome_v2::success());
    auto config = std::any_cast<std::optional<calibration::LaserTorchCalibration>>(converter.GetConfig());

    // Expect no configuration data since only default values in yaml file
    CHECK_EQ(config, std::nullopt);
  }
  TEST_CASE("Convert - valid configuration") {
    std::string yaml = R"(
valid: true
x: 1.0
y: 2.0
z: 3.0
angle: 4.0)";

    MockFactory mock_factory;
    auto mock = [&]() { return &mock_factory; };
    configuration::SetFactoryGenerator(mock);

    auto mock_fh = std::make_shared<MockFileHander>();

    REQUIRE_CALL(mock_factory, CreateFileHandler()).RETURN(mock_fh);
    REQUIRE_CALL(*mock_fh, ReadFile("/adaptio/config/laser_torch_calibration.yaml")).RETURN(yaml);

    LaserTorchCalibConverter converter(TAG_LTC, "/adaptio/config/laser_torch_calibration.yaml");
    CHECK_EQ(converter.ReadPersistentData(), boost::outcome_v2::success());
    auto maybe_config = std::any_cast<std::optional<calibration::LaserTorchCalibration>>(converter.GetConfig());
    calibration::LaserTorchCalibration ref = {1.0, 2.0, 3.0, 4.0};
    // Expect configuration
    CHECK(maybe_config.has_value());
    auto config = maybe_config.value();
    CHECK_EQ(ref.x, config.x);
    CHECK_EQ(ref.y, config.y);
    CHECK_EQ(ref.z, config.z);
    CHECK_EQ(ref.angle, config.angle);
  }
  TEST_CASE("Convert - exception") {
    std::string yaml = R"(
valid: false
x: -nan
y: 2.0
z: 3.0
angle: 4.0)";

    MockFactory mock_factory;
    auto mock = [&]() { return &mock_factory; };
    configuration::SetFactoryGenerator(mock);

    auto mock_fh = std::make_shared<MockFileHander>();

    REQUIRE_CALL(mock_factory, CreateFileHandler()).RETURN(mock_fh);
    REQUIRE_CALL(*mock_fh, ReadFile("/adaptio/config/laser_torch_calibration.yaml")).RETURN(yaml);

    LaserTorchCalibConverter converter(TAG_LTC, "/adaptio/config/laser_torch_calibration.yaml");
    CHECK_EQ(converter.ReadPersistentData(),
             boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR)));
  }
  TEST_CASE("Convert - store") {
    std::string yaml = R"(
valid: false
x: 1.0
y: 2.0
z: 3.0
angle: 4.0)";

    MockFactory mock_factory;
    auto mock = [&]() { return &mock_factory; };
    configuration::SetFactoryGenerator(mock);

    auto mock_fh = std::make_shared<MockFileHander>();

    REQUIRE_CALL(mock_factory, CreateFileHandler()).RETURN(mock_fh);
    REQUIRE_CALL(*mock_fh, ReadFile("/adaptio/config/laser_torch_calibration.yaml")).RETURN(yaml);

    LaserTorchCalibConverter converter(TAG_LTC, "/adaptio/config/laser_torch_calibration.yaml");
    CHECK_EQ(converter.ReadPersistentData(), boost::outcome_v2::success());

    calibration::LaserTorchCalibration laser_torch_calibration = {10.0, 20.0, 30.0, 40.0};

    std::string yaml_out = R"(valid: true
x: 1.000000000000000e+01
y: 2.000000000000000e+01
z: 3.000000000000000e+01
angle: 4.000000000000000e+01)";

    REQUIRE_CALL(*mock_fh, WriteFile(std::string("/adaptio/config/laser_torch_calibration.yaml"), yaml_out))
        .RETURN(true);
    CHECK_EQ(converter.WritePersistentData(laser_torch_calibration), boost::outcome_v2::success());
  }
}
// NOLINTEND(*-magic-numbers, misc-include-cleaner)
}  // namespace configuration
