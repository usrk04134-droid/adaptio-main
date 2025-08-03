#pragma once

#include <doctest/doctest.h>

#include "block_tests/helpers_web_hmi.h"
#include "common/messages/kinematics.h"
#include "helpers.h"
#include "helpers_simulator.h"
#include "sim-config.h"
#include "simulator_interface.h"
#include "test_utils/testlog.h"
#include "web_hmi/web_hmi_json_helpers.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)
inline auto Merge(const nlohmann::json& payload1, const nlohmann::json& payload2) -> nlohmann::json {
  nlohmann::json result = payload1;
  result.update(payload2);
  return result;
}

inline void LaserTorchCalSet(TestFixture& fixture, const nlohmann::json& payload) {
  auto msg = web_hmi::CreateMessage("LaserTorchCalSet", payload);
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto LaserTorchCalSetRsp(TestFixture& fixture) -> nlohmann::json {
  return ReceiveJsonByName(fixture, "LaserTorchCalSetRsp");
}

inline void LaserTorchCalGet(TestFixture& fixture) {
  auto msg = web_hmi::CreateMessage("LaserTorchCalGet", {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto LaserTorchCalGetRsp(TestFixture& fixture) -> nlohmann::json {
  return ReceiveJsonByName(fixture, "LaserTorchCalGetRsp");
}

inline void WeldObjectCalSet(TestFixture& fixture, const nlohmann::json& payload) {
  auto msg = web_hmi::CreateMessage("WeldObjectCalSet", payload);
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto WeldObjectCalSetRsp(TestFixture& fixture) -> nlohmann::json {
  return ReceiveJsonByName(fixture, "WeldObjectCalSetRsp");
}

inline void WeldObjectCalGet(TestFixture& fixture) {
  auto msg = web_hmi::CreateMessage("WeldObjectCalGet", {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto WeldObjectCalGetRsp(TestFixture& fixture) -> nlohmann::json {
  return ReceiveJsonByName(fixture, "WeldObjectCalGetRsp");
}

inline void WeldObjectCalStart(TestFixture& fixture, double wire_diameter, double stickout, double weld_object_radius) {
  auto const payload = nlohmann::json({
      {"wireDiameter",     wire_diameter     },
      {"stickout",         stickout          },
      {"weldObjectRadius", weld_object_radius},
  });

  auto msg = web_hmi::CreateMessage("WeldObjectCalStart", payload);
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto WeldObjectCalStartRsp(TestFixture& fixture) -> bool {
  auto const response_payload = ReceiveJsonByName(fixture, "WeldObjectCalStartRsp");
  CHECK(response_payload != nullptr);

  auto const expect_ok = nlohmann::json{
      {"result", "ok"}
  };

  return response_payload == expect_ok;
}

inline void WeldObjectCalStop(TestFixture& fixture) {
  auto msg = web_hmi::CreateMessage("WeldObjectCalStop", {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto WeldObjectCalStopRsp(TestFixture& fixture) -> bool {
  auto const response_payload = ReceiveJsonByName(fixture, "WeldObjectCalStopRsp");
  CHECK(response_payload != nullptr);

  auto const expect_ok = nlohmann::json{
      {"result", "ok"}
  };

  return response_payload == expect_ok;
}

inline void WeldObjectCalLeftPos(TestFixture& fixture) {
  auto msg = web_hmi::CreateMessage("WeldObjectCalLeftPos", {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto WeldObjectCalLeftPosRsp(TestFixture& fixture) -> bool {
  auto const response_payload = ReceiveJsonByName(fixture, "WeldObjectCalLeftPosRsp");
  CHECK(response_payload != nullptr);

  auto const expect_ok = nlohmann::json{
      {"result", "ok"}
  };

  return response_payload == expect_ok;
}

inline void WeldObjectCalRightPos(TestFixture& fixture) {
  auto msg = web_hmi::CreateMessage("WeldObjectCalRightPos", {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto WeldObjectCalRightPosRsp(TestFixture& fixture) -> bool {
  auto const response_payload = ReceiveJsonByName(fixture, "WeldObjectCalRightPosRsp");
  CHECK(response_payload != nullptr);

  auto const expect_ok = nlohmann::json{
      {"result", "ok"}
  };

  return response_payload == expect_ok;
}

inline auto WeldObjectCalResult(TestFixture& fixture, deposition_simulator::SimConfig& sim_config) -> nlohmann::json {
  auto const response_payload = ReceiveJsonByName(fixture, "WeldObjectCalResult");
  CHECK(response_payload != nullptr);

  return response_payload;
}

[[maybe_unused]] inline auto ToString(const deposition_simulator::Point3d& point) -> std::string {
  return fmt::format("x: {:.5f} y: {:.5f} z {:.5f}", point.GetX(), point.GetY(), point.GetZ());
}

inline auto GetSliceData(std::vector<deposition_simulator::Point3d>& abws_lpcs, const std::uint64_t time_stamp)
    -> common::msg::scanner::SliceData {
  common::msg::scanner::SliceData slice_data{
      .groove{{.x = helpers_simulator::ConvertM2Mm(abws_lpcs[0].GetX()),
               .y = helpers_simulator::ConvertM2Mm(abws_lpcs[0].GetY())},
              {.x = helpers_simulator::ConvertM2Mm(abws_lpcs[1].GetX()),
               .y = helpers_simulator::ConvertM2Mm(abws_lpcs[1].GetY())},
              {.x = helpers_simulator::ConvertM2Mm(abws_lpcs[2].GetX()),
               .y = helpers_simulator::ConvertM2Mm(abws_lpcs[2].GetY())},
              {.x = helpers_simulator::ConvertM2Mm(abws_lpcs[3].GetX()),
               .y = helpers_simulator::ConvertM2Mm(abws_lpcs[3].GetY())},
              {.x = helpers_simulator::ConvertM2Mm(abws_lpcs[4].GetX()),
               .y = helpers_simulator::ConvertM2Mm(abws_lpcs[4].GetY())},
              {.x = helpers_simulator::ConvertM2Mm(abws_lpcs[5].GetX()),
               .y = helpers_simulator::ConvertM2Mm(abws_lpcs[5].GetY())},
              {.x = helpers_simulator::ConvertM2Mm(abws_lpcs[6].GetX()),
               .y = helpers_simulator::ConvertM2Mm(abws_lpcs[6].GetY())}},
      .line{{.x = helpers_simulator::ConvertM2Mm(abws_lpcs[0].GetX()),
             .y = helpers_simulator::ConvertM2Mm(abws_lpcs[0].GetY())},
              {.x = helpers_simulator::ConvertM2Mm(abws_lpcs[1].GetX()),
             .y = helpers_simulator::ConvertM2Mm(abws_lpcs[1].GetY())},
              {.x = helpers_simulator::ConvertM2Mm(abws_lpcs[2].GetX()),
             .y = helpers_simulator::ConvertM2Mm(abws_lpcs[2].GetY())},
              {.x = helpers_simulator::ConvertM2Mm(abws_lpcs[3].GetX()),
             .y = helpers_simulator::ConvertM2Mm(abws_lpcs[3].GetY())},
              {.x = helpers_simulator::ConvertM2Mm(abws_lpcs[4].GetX()),
             .y = helpers_simulator::ConvertM2Mm(abws_lpcs[4].GetY())},
              {.x = helpers_simulator::ConvertM2Mm(abws_lpcs[5].GetX()),
             .y = helpers_simulator::ConvertM2Mm(abws_lpcs[5].GetY())},
              {.x = helpers_simulator::ConvertM2Mm(abws_lpcs[6].GetX()),
             .y = helpers_simulator::ConvertM2Mm(abws_lpcs[6].GetY())}  },
      .confidence = common::msg::scanner::SliceConfidence::HIGH,
      .time_stamp = time_stamp,
  };
  return slice_data;
}

inline auto NowTimeStamp(TestFixture& fixture) -> uint64_t {
  return fixture.GetClockNowFuncWrapper()->GetSystemClock().time_since_epoch().count();
}

inline void ProvideScannerAndKinematicsData(TestFixture& fixture, deposition_simulator::ISimulator& simulator,
                                            const deposition_simulator::Point3d& point) {
  auto abws_lpcs  = helpers_simulator::ConvertFromOptionalAbwVector(simulator.GetAbwPoints(deposition_simulator::LPCS));
  auto slice_data = GetSliceData(abws_lpcs, NowTimeStamp(fixture));
  fixture.Scanner()->Dispatch(slice_data);

  // Receive GetSlidesPosition
  auto get_position = fixture.Kinematics()->Receive<common::msg::kinematics::GetSlidesPosition>();

  // GetSlidePosition response
  fixture.Kinematics()->Dispatch(
      common::msg::kinematics::GetSlidesPositionRsp{.client_id  = get_position->client_id,
                                                    .time_stamp = get_position->time_stamp,
                                                    .horizontal = helpers_simulator::ConvertM2Mm(point.GetX()),
                                                    .vertical   = helpers_simulator::ConvertM2Mm(point.GetZ())});
}

inline void ReceiveProgress(TestFixture& fixture) {
  auto const payload = ReceiveJsonByName(fixture, "WeldObjectCalProgress");
  CHECK(payload != nullptr);
  auto progress = payload["progress"].get<double>();
  CHECK(progress > 0);
  TESTLOG(">>>>> Weld Object calibration progress: {:.2f}", progress);
}

inline auto GridMeasurementAttempt(TestFixture& fixture, deposition_simulator::ISimulator& simulator) -> bool {
  auto set_position = fixture.Kinematics()->Receive<common::msg::kinematics::SetSlidesPosition>();
  if (!set_position) {
    return false;
  }

  auto horizontal_pos_m = helpers_simulator::ConvertMm2M(set_position.value().horizontal);
  auto vertical_pos_m   = helpers_simulator::ConvertMm2M(set_position.value().vertical);

  // Update the torch position according to the request
  deposition_simulator::Point3d torch_pos_macs(horizontal_pos_m, 0, vertical_pos_m, deposition_simulator::MACS);
  simulator.UpdateTorchPosition(torch_pos_macs);

  ProvideScannerAndKinematicsData(fixture, simulator, torch_pos_macs);

  // Dispatch timeout on stabilization timer
  fixture.Timer()->Dispatch("stabilization_delay");

  // Provide scannerdata again, this will be recorded (or skipped if an extra movement gridpoint)
  ProvideScannerAndKinematicsData(fixture, simulator, torch_pos_macs);

  ReceiveProgress(fixture);

  return true;
}

inline void PositionTorchInGroove(deposition_simulator::ISimulator& simulator, double stickout_m,
                                  double touch_point_depth_m) {
  // Position the torch with the tip of the wire at a depth TOUCH_POINT_DEPTH_M
  // in the center of the groove
  auto abw_in_torch_plane = simulator.GetSliceInTorchPlane(deposition_simulator::MACS);
  TESTLOG(">>>>> Torch plane ABW0: {}, ABW6: {}", ToString(abw_in_torch_plane.front()),
          ToString(abw_in_torch_plane.back()));

  deposition_simulator::Point3d torch_pos_initial_macs(
      std::midpoint(abw_in_torch_plane.front().GetX(), abw_in_torch_plane.back().GetX()), 0,
      abw_in_torch_plane.front().GetZ() + stickout_m - touch_point_depth_m, deposition_simulator::MACS);

  simulator.UpdateTorchPosition(torch_pos_initial_macs);
}

struct CalibrateConfig {
  double stickout_m{};
  double touch_point_depth_m{};
  double scanner_mount_angle_rad{};
  double wire_diameter_mm{};
  double weld_object_diameter_m{};
};

[[nodiscard]] inline auto Calibrate(TestFixture& fixture, deposition_simulator::SimConfig& sim_config,
                                    deposition_simulator::ISimulator& simulator, const CalibrateConfig& conf) -> bool {
  // Calculate LTC parameters from sim_config
  double ltc_stickout = 0.025;  // m
  double ltc_torch_to_laser_plane_dist =
      helpers_simulator::ComputeLtcTorchToLaserPlaneDistance(sim_config.lpcs_config, ltc_stickout);
  ltc_stickout                  = helpers_simulator::ConvertM2Mm(ltc_stickout);
  ltc_torch_to_laser_plane_dist = helpers_simulator::ConvertM2Mm(ltc_torch_to_laser_plane_dist);

  // ---------- Test sequence starts here --------------

  // Position the torch with the wire at a suitable depth in the groove
  PositionTorchInGroove(simulator, conf.stickout_m, conf.touch_point_depth_m);

  // Subscribe Ready State (checking this later)
  fixture.Management()->Dispatch(common::msg::management::SubscribeReadyState{});
  auto ready_msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
  CHECK(ready_msg.has_value());
  CHECK_EQ(ready_msg->state, common::msg::management::ReadyState::State::NOT_READY);

  // Set laser to torch calibration
  LaserTorchCalSet(fixture, {
                                {"distanceLaserTorch", ltc_torch_to_laser_plane_dist},
                                {"stickout",           ltc_stickout                 },
                                {"scannerMountAngle",  conf.scanner_mount_angle_rad }
  });

  CHECK_EQ(LaserTorchCalSetRsp(fixture), nlohmann::json{
                                             {"result", "ok"}
  });

  // Operator starts the calibration procedure
  WeldObjectCalStart(fixture, conf.wire_diameter_mm, helpers_simulator::ConvertM2Mm(conf.stickout_m),
                     helpers_simulator::ConvertM2Mm(conf.weld_object_diameter_m) / 2.0);

  // Receive StartScanner
  REQUIRE_MESSAGE(fixture.Scanner()->Receive<common::msg::scanner::Start>(), "No Start msg received");
  fixture.Scanner()->Dispatch(common::msg::scanner::StartRsp{.success = true});

  // This Rsp triggers the WebHmi to display the instruction to
  // touch the left wall
  CHECK(WeldObjectCalStartRsp(fixture));

  // Simulate that the operator moves the torch to touch the left wall
  simulator.TouchLeftWall(conf.stickout_m);
  auto torch_pos = simulator.GetTorchPosition();
  TESTLOG(">>>>> deposition_simulator moved torch to left touch position: {}", ToString(torch_pos));

  // Operator presses the left position button
  WeldObjectCalLeftPos(fixture);
  ProvideScannerAndKinematicsData(fixture, simulator, torch_pos);

  CHECK(WeldObjectCalLeftPosRsp(fixture));

  // Simulate that the operator moves the torch to touch the right wall
  simulator.TouchRightWall(conf.stickout_m);
  torch_pos = simulator.GetTorchPosition();
  TESTLOG(">>>>> deposition_simulator moved torch to right touch position: {}", ToString(torch_pos));

  // Operator presses the right position button
  WeldObjectCalRightPos(fixture);
  ProvideScannerAndKinematicsData(fixture, simulator, torch_pos);

  // Check Ready state
  ready_msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
  CHECK(ready_msg);
  CHECK_EQ(ready_msg->state, common::msg::management::ReadyState::State::NOT_READY_AUTO_CAL_MOVE);

  CHECK(WeldObjectCalRightPosRsp(fixture));

  TESTLOG(">>>>> Automatic grid measurement sequence started");

  while (GridMeasurementAttempt(fixture, simulator)) {
    // Loop while new grid positions are requested
  }

  // The procedure is complete here
  auto calibration_result = WeldObjectCalResult(fixture, sim_config);
  CHECK(calibration_result["result"] == "ok");
  TESTLOG(">>>>> WeldObjectCalResult: {}", calibration_result.dump());
  auto torch_to_lpcs_translation_c2 = calibration_result["torchToLpcsTranslation"]["c2"].get<double>();
  CHECK_EQ(round(torch_to_lpcs_translation_c2), round(1000 * sim_config.lpcs_config.y));

  // Check Ready state
  ready_msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
  CHECK(ready_msg);
  CHECK_EQ(ready_msg->state, common::msg::management::ReadyState::State::NOT_READY);

  // Now apply this calibration result
  WeldObjectCalSet(fixture, calibration_result);
  return true;
}

// NOLINTEND(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)
