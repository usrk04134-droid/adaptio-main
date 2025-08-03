#include <doctest/doctest.h>

#include <any>
#include <memory>
#include <optional>
#include <string>
#include <trompeloeil.hpp>  // IWYU pragma: keep

#include "../circ_weld_object_calib_converter.h"
#include "../config_manager.h"
#include "../configuration_error.h"
#include "common/file/yaml.h"
#include "configuration/conf_factory.h"
#include "main/calibration/calibration_types.h"
#include "mock/mock_factory.h"
#include "mock/mock_file_handler.h"

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)

namespace configuration {

TEST_SUITE("Circular Weld Object Calibration Converter") {
  TEST_CASE("Convert") {
    std::string yaml = R"(
valid: false
y: 2.0
z: 3.0
x_adjustment: 0.6)";

    MockFactory mock_factory;
    auto mock = [&]() { return &mock_factory; };
    configuration::SetFactoryGenerator(mock);

    auto mock_fh = std::make_shared<MockFileHander>();

    REQUIRE_CALL(mock_factory, CreateFileHandler()).RETURN(mock_fh);
    REQUIRE_CALL(*mock_fh, ReadFile("/adaptio/config/circular_weld_object_calibration.yaml")).RETURN(yaml);

    CircWeldObjectCalibConverter converter(TAG_CWOC, "/adaptio/config/circular_weld_object_calibration.yaml");
    CHECK_EQ(converter.ReadPersistentData(), boost::outcome_v2::success());
    auto config = std::any_cast<std::optional<calibration::WeldObjectCalibration>>(converter.GetConfig());

    // Expect no configuration data since only default values in yaml file
    CHECK_EQ(config, std::nullopt);
  }
  TEST_CASE("Convert - valid configuration") {
    std::string yaml = R"(
valid: true
y: 1.0
z: 2.0
x_adjustment: 1.6)";

    MockFactory mock_factory;
    auto mock = [&]() { return &mock_factory; };
    configuration::SetFactoryGenerator(mock);

    auto mock_fh = std::make_shared<MockFileHander>();

    REQUIRE_CALL(mock_factory, CreateFileHandler()).RETURN(mock_fh);
    REQUIRE_CALL(*mock_fh, ReadFile("/adaptio/config/circular_weld_object_calibration.yaml")).RETURN(yaml);

    CircWeldObjectCalibConverter converter(TAG_CWOC, "/adaptio/config/circular_weld_object_calibration.yaml");
    CHECK_EQ(converter.ReadPersistentData(), boost::outcome_v2::success());
    auto maybe_config = std::any_cast<std::optional<calibration::WeldObjectCalibration>>(converter.GetConfig());
    calibration::WeldObjectCalibration ref = {1.0, 2.0, 1.6};
    // Expect configuration
    CHECK(maybe_config.has_value());
    auto config = maybe_config.value();
    CHECK_EQ(ref.y, config.y);
    CHECK_EQ(ref.z, config.z);
    CHECK_EQ(ref.x_adjustment, config.x_adjustment);
  }
  TEST_CASE("Convert - exception") {
    std::string yaml = R"(
valid: true
y: 1.0
z: -nan
x_adjustment: 0.6)";

    MockFactory mock_factory;
    auto mock = [&]() { return &mock_factory; };
    configuration::SetFactoryGenerator(mock);

    auto mock_fh = std::make_shared<MockFileHander>();

    REQUIRE_CALL(mock_factory, CreateFileHandler()).RETURN(mock_fh);
    REQUIRE_CALL(*mock_fh, ReadFile("/adaptio/config/circular_weld_object_calibration.yaml")).RETURN(yaml);

    CircWeldObjectCalibConverter converter(TAG_CWOC, "/adaptio/config/circular_weld_object_calibration.yaml");
    CHECK_EQ(converter.ReadPersistentData(),
             boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR)));
  }
  TEST_CASE("Convert - store") {
    std::string yaml = R"(
valid: false
y: 2.0
z: 3.0
x_adjustment: 0.6)";

    MockFactory mock_factory;
    auto mock = [&]() { return &mock_factory; };
    configuration::SetFactoryGenerator(mock);

    auto mock_fh = std::make_shared<MockFileHander>();

    REQUIRE_CALL(mock_factory, CreateFileHandler()).RETURN(mock_fh);
    REQUIRE_CALL(*mock_fh, ReadFile("/adaptio/config/circular_weld_object_calibration.yaml")).RETURN(yaml);

    CircWeldObjectCalibConverter converter(TAG_CWOC, "/adaptio/config/circular_weld_object_calibration.yaml");
    CHECK_EQ(converter.ReadPersistentData(), boost::outcome_v2::success());

    calibration::WeldObjectCalibration circ_weld_object_calib = {4.0, 5.0, 1.2};

    std::string yaml_out = R"(valid: true
y: 4.000000000000000e+00
z: 5.000000000000000e+00
x_adjustment: 1.200000000000000e+00)";

    REQUIRE_CALL(*mock_fh, WriteFile(std::string("/adaptio/config/circular_weld_object_calibration.yaml"), yaml_out))
        .RETURN(true);
    CHECK_EQ(converter.WritePersistentData(circ_weld_object_calib), boost::outcome_v2::success());
  }
}
// NOLINTEND(*-magic-numbers, misc-include-cleaner)
}  // namespace configuration
