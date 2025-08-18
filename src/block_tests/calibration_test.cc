#include <doctest/doctest.h>

#include <cstdint>
#include <nlohmann/json_fwd.hpp>
#include <utility>

#include "block_tests/helpers_joint_geometry.h"
#include "block_tests/helpers_kinematics.h"
#include "block_tests/helpers_settings.h"
#include "block_tests/helpers_web_hmi.h"
#include "calibration/calibration_types.h"
#include "common/messages/kinematics.h"
#include "common/messages/management.h"
#include "common/messages/scanner.h"
#include "common/messages/weld_system.h"
#include "helpers.h"
#include "helpers_weld_system.h"
#include "tracking/tracking_manager.h"
#include "web_hmi/web_hmi_json_helpers.h"
#include "weld_system_client/weld_system_types.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

const nlohmann::json SET_WOC_PAYLOAD({
    {"y",           -1727.2},
    {"z",           -2445.1},
    {"xAdjustment", 1.6    },
});

// TODO, use the functions in json_payload.h and move it from src folder
inline auto SetLaserTorchCalibrationPayload(const calibration::LaserTorchCalibration& calibration) -> nlohmann::json {
  nlohmann::json payload = {
      {"x",     calibration.x    },
      {"y",     calibration.y    },
      {"z",     calibration.z    },
      {"angle", calibration.angle}
  };

  return payload;
}

inline auto LaserTorchCalibrationValidFromPayload(const nlohmann::json& payload) -> bool {
  bool valid{};

  payload.at("valid").get_to(valid);

  return valid;
}

inline auto LaserTorchCalibrationFromPayload(const nlohmann::json& payload) -> calibration::LaserTorchCalibration {
  calibration::LaserTorchCalibration calibration{};

  payload.at("x").get_to(calibration.x);
  payload.at("y").get_to(calibration.y);
  payload.at("z").get_to(calibration.z);
  payload.at("angle").get_to(calibration.angle);

  return calibration;
}

inline auto WeldObjectCalibrationValidFromPayload(const nlohmann::json& payload) -> bool {
  bool valid{};

  payload.at("valid").get_to(valid);

  return valid;
}

auto GetLaserToTorchCalibration(TestFixture& fixture) -> calibration::LaserTorchCalibration {
  fixture.WebHmiIn()->DispatchMessage(web_hmi::CreateMessage("GetLaserToTorchCalibration", {}));
  auto get_rsp_payload = ReceiveJsonByName(fixture, "GetLaserToTorchCalibrationRsp");
  CHECK(get_rsp_payload != nullptr);
  return LaserTorchCalibrationFromPayload(get_rsp_payload);
}

TEST_SUITE("Calibration") {
  TEST_CASE("set_joint_geometry") {
    TestFixture fixture;

    nlohmann::json payload({
        {"upper_joint_width_mm",        42.0      },
        {"groove_depth_mm",             120.0     },
        {"left_joint_angle_rad",        0.139     },
        {"right_joint_angle_rad",       0.1       },
        {"left_max_surface_angle_rad",  0.34906585},
        {"right_max_surface_angle_rad", 0.34906585}
    });
    auto set_joint_geometry = web_hmi::CreateMessage("SetJointGeometry", payload);
    fixture.WebHmiIn()->DispatchMessage(std::move(set_joint_geometry));

    auto joint_geometry_rsp_payload = ReceiveJsonByName(fixture, "SetJointGeometryRsp");
    CHECK(joint_geometry_rsp_payload != nullptr);

    CheckJointGeometryParamsEqual(fixture, payload);
  }
  // Removed legacy v1 calibration flow; covered by calibration_v2_test.cc

  TEST_CASE("set_weld_object_calibration_not_allowed_when_arcing") {
    TestFixture fixture;

    StoreSettings(fixture, TestSettings{.use_edge_sensor = false}, true);
    StoreDefaultJointGeometryParams(fixture);

    common::msg::management::TrackingStart start_joint_tracking_msg{
        .joint_tracking_mode = static_cast<uint32_t>(tracking::TrackingMode::TRACKING_LEFT_HEIGHT),
        .horizontal_offset   = 10.0,
        .vertical_offset     = 20.0};
    fixture.Management()->Dispatch(start_joint_tracking_msg);

    CHECK(fixture.Scanner()->Receive<common::msg::scanner::Start>());

    fixture.Scanner()->Dispatch(fixture.ScannerData()->Get());

    auto get_position = fixture.Kinematics()->Receive<common::msg::kinematics::GetSlidesPosition>();
    CHECK(get_position);

    fixture.Kinematics()->Dispatch(common::msg::kinematics::GetSlidesPositionRsp{.client_id  = get_position->client_id,
                                                                                 .time_stamp = get_position->time_stamp,
                                                                                 .horizontal = -20,
                                                                                 .vertical   = 5});

    double const position = 1.23;
    double const velocity = 0.0167;
    double const radius   = 1000;

    CheckAndDispatchGetWeldAxis(fixture, position, velocity, radius);

    const common::msg::weld_system::GetWeldSystemDataRsp weld_system_status_rsp{
        .voltage           = 30.123456,
        .current           = 200.0,
        .wire_lin_velocity = 12.1,
        .deposition_rate   = 3.5,
        .heat_input        = 120.0,
        .twin_wire         = true,
        .wire_diameter     = 1.1,
    };

    CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID1, weld_system_status_rsp);
    CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID2, weld_system_status_rsp);

    CHECK(fixture.Kinematics()->Receive<common::msg::kinematics::SetSlidesPosition>());

    DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID1,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);
    DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID2,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);

    auto set_cal = web_hmi::CreateMessage("SetWeldObjectCalibration", SET_WOC_PAYLOAD);
    fixture.WebHmiIn()->DispatchMessage(std::move(set_cal));

    auto rsp_payload = ReceiveJsonByName(fixture, "SetWeldObjectCalibrationRsp");
    CHECK(rsp_payload != nullptr);

    bool result = rsp_payload["result"].get<bool>();
    CHECK_FALSE(result);
  }

  TEST_CASE("set_weld_object_calibration_not_allowed_wrong_tracking_mode") {
    TestFixture fixture;

    StoreDefaultJointGeometryParams(fixture);
    // Start Joint tracking with mode=left
    // Setting weld object calibration is not allowed in this mode
    common::msg::management::TrackingStart start_joint_tracking_msg{
        static_cast<uint32_t>(tracking::TrackingMode::TRACKING_LEFT_HEIGHT), 10, 20};
    fixture.Management()->Dispatch(start_joint_tracking_msg);

    auto set_cal = web_hmi::CreateMessage("SetWeldObjectCalibration", SET_WOC_PAYLOAD);
    fixture.WebHmiIn()->DispatchMessage(std::move(set_cal));

    auto rsp_payload = ReceiveJsonByName(fixture, "SetWeldObjectCalibrationRsp");
    CHECK(rsp_payload != nullptr);

    bool result = rsp_payload["result"].get<bool>();
    CHECK_FALSE(result);
  }

  TEST_CASE("start_weld_object_calibration_not_allowed_when_not_idle") {
    TestFixture fixture;

    StoreDefaultJointGeometryParams(fixture);

    // Start Joint tracking
    // Starting weld object calibration only allowed when idle
    common::msg::management::TrackingStart start_joint_tracking_msg{
        static_cast<uint32_t>(tracking::TrackingMode::TRACKING_LEFT_HEIGHT), 10, 20};
    fixture.Management()->Dispatch(start_joint_tracking_msg);

    // Start calibration
    nlohmann::json payload({
        {"radius",   3000},
        {"stickout", 30.0}
    });
    auto start_cal = web_hmi::CreateMessage("WeldObjectCalibration", payload);
    fixture.WebHmiIn()->DispatchMessage(std::move(start_cal));

    auto rsp_payload = ReceiveJsonByName(fixture, "WeldObjectCalibrationRsp");
    CHECK(rsp_payload != nullptr);

    auto valid = WeldObjectCalibrationValidFromPayload(rsp_payload);
    CHECK_FALSE(valid);
  }

  TEST_CASE("perform_weld_object_calibration") {
    TestFixture fixture;

    StoreDefaultJointGeometryParams(fixture);
    // Start calibration
    nlohmann::json payload_wo({
        {"radius",   3000},
        {"stickout", 30.0}
    });
    auto start_cal = web_hmi::CreateMessage("WeldObjectCalibration", payload_wo);
    fixture.WebHmiIn()->DispatchMessage(std::move(start_cal));

    // Receive Start on Scanner
    CHECK(fixture.Scanner()->Receive<common::msg::scanner::Start>());

    // ABW points on scanner interface
    fixture.Scanner()->Dispatch(fixture.ScannerData()->Get());

    // Receive GetPosition
    auto get_position = fixture.Kinematics()->Receive<common::msg::kinematics::GetSlidesPosition>();
    CHECK(get_position);

    // Dispatch GetPositionRsp
    fixture.Kinematics()->Dispatch(common::msg::kinematics::GetSlidesPositionRsp{.client_id  = get_position->client_id,
                                                                                 .time_stamp = get_position->time_stamp,
                                                                                 .horizontal = -20,
                                                                                 .vertical   = -3000});

    // Receive Stop
    CHECK(fixture.Scanner()->Receive<common::msg::scanner::Stop>());

    // Finally receive WeldObjectCalibrationCompleted
    auto cal_completed = ReceiveJsonByName(fixture, "WeldObjectCalibrationRsp");
    CHECK(cal_completed != nullptr);

    auto valid = WeldObjectCalibrationValidFromPayload(cal_completed);

    CHECK(valid);
  }

  TEST_CASE("duration_supervision") {
    TestFixture fixture;

    // Start calibration
    nlohmann::json payload({
        {"offset",   35.0},
        {"angle",    0.2 },
        {"stickout", 25.0}
    });
    auto start_cal = web_hmi::CreateMessage("LaserToTorchCalibration", payload);
    fixture.WebHmiIn()->DispatchMessage(std::move(start_cal));

    // Receive Start on Scanner
    CHECK(fixture.Scanner()->Receive<common::msg::scanner::Start>());

    // Dispatch timeout on calibration duration supervision
    fixture.Timer()->Dispatch("calibration_supervision");

    // Receive Stop
    CHECK(fixture.Scanner()->Receive<common::msg::scanner::Stop>());

    // Receive LaserToTorchCalibrationRsp with valid=false
    auto calibration_rsp_payload = ReceiveJsonByName(fixture, "LaserToTorchCalibrationRsp");
    CHECK(calibration_rsp_payload != nullptr);
    auto valid = LaserTorchCalibrationValidFromPayload(calibration_rsp_payload);
    CHECK_EQ(valid, false);

    //    LOG_DEBUG("{}", fixture.DescribeQueue());
  }
}
// NOLINTEND(*-magic-numbers, *-optional-access)
