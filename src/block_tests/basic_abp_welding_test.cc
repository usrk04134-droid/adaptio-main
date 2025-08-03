#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "block_tests/helpers_settings.h"
#include "block_tests/helpers_weld_system.h"
#include "common/messages/kinematics.h"
#include "common/messages/management.h"
#include "common/messages/scanner.h"
#include "common/messages/weld_system.h"
#include "helpers.h"
#include "helpers_abp_parameters.h"
#include "helpers_calibration_v2.h"
#include "helpers_kinematics.h"
#include "helpers_simulator.h"
#include "helpers_weld_control.h"
#include "helpers_weld_system.h"
#include "point3d.h"
#include "sim-config.h"
#include "simulator_interface.h"
#include "tracking/tracking_manager.h"
#include "weld_system_client/weld_system_types.h"

//  NOLINTBEGIN(*-magic-numbers, *-optional-access)

#define TESTLOG_DISABLED
#include "test_utils/testlog.h"

namespace depsim   = deposition_simulator;
namespace help_sim = helpers_simulator;

namespace {
// Constants
const int NUMBER_OF_STEPS_PER_REV{200};
const double DELTA_ANGLE{2 * help_sim::PI / NUMBER_OF_STEPS_PER_REV};
const int START_BEAD_STATE_MONITORING_AT_STEP{90};
const int CHECK_MONITORED_BEAD_STATE_AT_STEP{80};

const double SCANNER_MOUNT_ANGLE = 6.0 * help_sim::PI / 180.0;
const double TOUCH_POINT_DEPTH_M = 10e-3;
// Common expects
const bool EXPECT_OK{true};
// Helpers
auto GetSliceData(std::vector<depsim::Point3d> &abws_lpcs, const std::uint64_t time_stamp)
    -> common::msg::scanner::SliceData {
  common::msg::scanner::SliceData slice_data{
      .groove{{.x = help_sim::ConvertM2Mm(abws_lpcs[0].GetX()), .y = help_sim::ConvertM2Mm(abws_lpcs[0].GetY())},
              {.x = help_sim::ConvertM2Mm(abws_lpcs[1].GetX()), .y = help_sim::ConvertM2Mm(abws_lpcs[1].GetY())},
              {.x = help_sim::ConvertM2Mm(abws_lpcs[2].GetX()), .y = help_sim::ConvertM2Mm(abws_lpcs[2].GetY())},
              {.x = help_sim::ConvertM2Mm(abws_lpcs[3].GetX()), .y = help_sim::ConvertM2Mm(abws_lpcs[3].GetY())},
              {.x = help_sim::ConvertM2Mm(abws_lpcs[4].GetX()), .y = help_sim::ConvertM2Mm(abws_lpcs[4].GetY())},
              {.x = help_sim::ConvertM2Mm(abws_lpcs[5].GetX()), .y = help_sim::ConvertM2Mm(abws_lpcs[5].GetY())},
              {.x = help_sim::ConvertM2Mm(abws_lpcs[6].GetX()), .y = help_sim::ConvertM2Mm(abws_lpcs[6].GetY())}},
      .line{{.x = help_sim::ConvertM2Mm(abws_lpcs[0].GetX()), .y = help_sim::ConvertM2Mm(abws_lpcs[0].GetY())},
              {.x = help_sim::ConvertM2Mm(abws_lpcs[1].GetX()), .y = help_sim::ConvertM2Mm(abws_lpcs[1].GetY())},
              {.x = help_sim::ConvertM2Mm(abws_lpcs[2].GetX()), .y = help_sim::ConvertM2Mm(abws_lpcs[2].GetY())},
              {.x = help_sim::ConvertM2Mm(abws_lpcs[3].GetX()), .y = help_sim::ConvertM2Mm(abws_lpcs[3].GetY())},
              {.x = help_sim::ConvertM2Mm(abws_lpcs[4].GetX()), .y = help_sim::ConvertM2Mm(abws_lpcs[4].GetY())},
              {.x = help_sim::ConvertM2Mm(abws_lpcs[5].GetX()), .y = help_sim::ConvertM2Mm(abws_lpcs[5].GetY())},
              {.x = help_sim::ConvertM2Mm(abws_lpcs[6].GetX()), .y = help_sim::ConvertM2Mm(abws_lpcs[6].GetY())}},
      .confidence = common::msg::scanner::SliceConfidence::HIGH,
      .time_stamp = time_stamp,
  };
  return slice_data;
}
auto DumpAbw([[maybe_unused]] const std::vector<depsim::Point3d> &abws) -> void {
  TESTLOG("ABW MACS x=[{:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f}]", abws[0].GetX(), abws[1].GetX(),
          abws[2].GetX(), abws[3].GetX(), abws[4].GetX(), abws[5].GetX(), abws[6].GetX());
  TESTLOG("ABW MACS z=[{:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f}]", abws[0].GetZ(), abws[1].GetZ(),
          abws[2].GetZ(), abws[3].GetZ(), abws[4].GetZ(), abws[5].GetZ(), abws[6].GetZ());
}
template <class T>
class StateMonitor {
 public:
  explicit StateMonitor() = default;
  ~StateMonitor()         = default;
  auto Clear() -> void { states_.clear(); }
  auto Add(T &state) -> void {
    if (states_.empty() || state != states_.back()) {
      states_.push_back(state);
    }
  }
  auto Equal(const std::list<T> &states) -> bool { return states == states_; }
  auto Dump() -> std::string {
    std::ostringstream dump_string;
    for (auto const &state : states_) {
      dump_string << state << " ";
    }
    return dump_string.str();
  }

 private:
  std::list<T> states_;
};
// test function
auto BasicABPWeldingTest(help_sim::TestParameters &test_parameters) -> void;
};  // End namespace

//
// Tests
//
TEST_SUITE("BasicABPWeldingTest") {
  TEST_CASE("basic_abp_welding_wide_joint_test") {
    // Test parameters
    //
    // Purpose: Verify basic abp welding logic, test ends on ready-for-cap.
    /// Joint Geometry: helpers_sim_config::TEST_JOINT_GEOMETRY_WIDE.
    help_sim::TestParameters test_parameters{
        .abp_parameters{.wall_offset_mm = 3.7,
                        .bead_overlap   = 20.,
                        .step_up_value  = 0.5,
                        .k_gain         = 0.,
                        .heat_input{.min = 2.1, .max = 2.8},
                        .weld_system_2_current{.min = 700., .max = 700.}},
        .welding_parameters{.weld_object_diameter_m       = 2.,
                        .weld_object_speed_cm_per_min = 100.,
                        .stickout_m                   = 25e-3,
                        .weld_system_1{
                                .voltage                      = 28.0,
                                .current                      = 620.0,
                                .wire_lin_velocity_mm_per_sec = 23.0,
                                .deposition_rate              = 10.4,
                                .heat_input                   = 125.0,
                                .twin_wire                    = false,
                                .wire_diameter_mm             = 4.0,
                            }, .weld_system_2{
                                .voltage                      = 30.0,
                                .current                      = 700.0,
                                .wire_lin_velocity_mm_per_sec = 38.0,
                                .deposition_rate              = 10.6,
                                .heat_input                   = 125.0,
                                .twin_wire                    = false,
                                .wire_diameter_mm             = 4.0,
                            }},
        .test_joint_geometry{help_sim::TEST_JOINT_GEOMETRY_WIDE},

        .testcase_parameters{.configuration_file{"src/block_tests/configs/basic_abp_configuration.yaml"},
                        .expected_beads_in_layer{{1, 4}, {2, 4}, {3, 4}}}
    };
    BasicABPWeldingTest(test_parameters);
  }
  TEST_CASE("basic_abp_welding_narrow_joint_test") {
    // Test parameters
    //
    // Purpose: Verify basic abp welding logic, test ends on ready-for-cap.
    /// Joint Geometry: helpers_sim_config::TEST_JOINT_GEOMETRY_NARROW.
    help_sim::TestParameters test_parameters{
        .abp_parameters{.wall_offset_mm = 3.7,
                        .bead_overlap   = 20.,
                        .step_up_value  = 0.5,
                        .k_gain         = 0.,
                        .heat_input{.min = 2.1, .max = 2.8},
                        .weld_system_2_current{.min = 700.0, .max = 700.0}},
        .welding_parameters{.weld_object_diameter_m       = 2.,
                        .weld_object_speed_cm_per_min = 100.,
                        .stickout_m                   = 25e-3,
                        .weld_system_1{
                                .voltage                      = 28.0,
                                .current                      = 620.0,
                                .wire_lin_velocity_mm_per_sec = 12.0,
                                .deposition_rate              = 3,
                                .heat_input                   = 100.0,
                                .twin_wire                    = false,
                                .wire_diameter_mm             = 4.0,
                            }, .weld_system_2{
                                .voltage                      = 30.0,
                                .current                      = 700.0,
                                .wire_lin_velocity_mm_per_sec = 24.0,
                                .deposition_rate              = 4.5,
                                .heat_input                   = 125.0,
                                .twin_wire                    = false,
                                .wire_diameter_mm             = 4.0,
                            }},
        .test_joint_geometry{help_sim::TEST_JOINT_GEOMETRY_NARROW},

        .testcase_parameters{.configuration_file{"src/block_tests/configs/basic_abp_configuration.yaml"},
                        .expected_beads_in_layer{{1, 3}, {2, 4}, {3, 4}, {4, 4}}}
    };

    SUBCASE("Basic") { BasicABPWeldingTest(test_parameters); }
    SUBCASE("WithStepUpLimits") {
      test_parameters.abp_parameters.step_up_limits = {22.0 /* require >22mm bottom groove width for 3 beads */};

      BasicABPWeldingTest(test_parameters);
    }
  }
}

namespace {
auto BasicABPWeldingTest(help_sim::TestParameters &test_parameters) -> void {
  // WS1
  const common::msg::weld_system::GetWeldSystemDataRsp weld_system_1_status_rsp{
      .voltage = static_cast<float>(test_parameters.welding_parameters.weld_system_1.voltage),
      .current = static_cast<float>(test_parameters.welding_parameters.weld_system_1.current),
      .wire_lin_velocity =
          static_cast<float>(test_parameters.welding_parameters.weld_system_1.wire_lin_velocity_mm_per_sec),
      .deposition_rate = static_cast<float>(test_parameters.welding_parameters.weld_system_1.deposition_rate),
      .heat_input      = static_cast<float>(test_parameters.welding_parameters.weld_system_1.heat_input),
      .twin_wire       = test_parameters.welding_parameters.weld_system_1.twin_wire,
      .wire_diameter   = static_cast<float>(test_parameters.welding_parameters.weld_system_1.wire_diameter_mm),
  };
  // WS2
  const common::msg::weld_system::GetWeldSystemDataRsp weld_system_2_status_rsp{
      .voltage = static_cast<float>(test_parameters.welding_parameters.weld_system_2.voltage),
      .current = static_cast<float>(test_parameters.welding_parameters.weld_system_2.current),
      .wire_lin_velocity =
          static_cast<float>(test_parameters.welding_parameters.weld_system_2.wire_lin_velocity_mm_per_sec),
      .deposition_rate = static_cast<float>(test_parameters.welding_parameters.weld_system_2.deposition_rate),
      .heat_input      = static_cast<float>(test_parameters.welding_parameters.weld_system_2.heat_input),
      .twin_wire       = test_parameters.welding_parameters.weld_system_2.twin_wire,
      .wire_diameter   = static_cast<float>(test_parameters.welding_parameters.weld_system_2.wire_diameter_mm),
  };

  TestFixture fixture(test_parameters.testcase_parameters.configuration_file.value());

  StoreSettings(fixture, TestSettings{.use_edge_sensor = false}, true);

  // Set up to use timer wrapper
  fixture.SetupTimerWrapper();

  const std::chrono::milliseconds time_per_step_ms{static_cast<int>(help_sim::CalculateStepTimeMs(
      test_parameters.welding_parameters.weld_object_diameter_m,
      test_parameters.welding_parameters.weld_object_speed_cm_per_min, NUMBER_OF_STEPS_PER_REV))};

  // Create simulator
  std::unique_ptr<depsim::ISimulator> simulator = depsim::CreateSimulator();
  depsim::SimConfig sim_config                  = simulator->CreateSimConfig();
  help_sim::SetSimulatorDefault(sim_config, NUMBER_OF_STEPS_PER_REV);

  // Set up joint geometry
  help_sim::SetJointGeometry(fixture, sim_config, test_parameters.test_joint_geometry);

  sim_config.travel_speed =
      help_sim::ConvertCMPerMin2MPerS(test_parameters.welding_parameters.weld_object_speed_cm_per_min);

  // Add static torches
  auto depsim_ws1_torch = simulator->AddSingleWireTorch(
      help_sim::ConvertMm2M(test_parameters.welding_parameters.weld_system_1.wire_diameter_mm),
      help_sim::ConvertMmPerS2MPerS(test_parameters.welding_parameters.weld_system_1.wire_lin_velocity_mm_per_sec));
  auto depsim_ws2_torch = simulator->AddSingleWireTorch(
      help_sim::ConvertMm2M(test_parameters.welding_parameters.weld_system_2.wire_diameter_mm),
      help_sim::ConvertMmPerS2MPerS(test_parameters.welding_parameters.weld_system_2.wire_lin_velocity_mm_per_sec));

  // Simulator config OPCS/LPCS
  help_sim::ConfigOPCS(sim_config, test_parameters.welding_parameters.weld_object_diameter_m,
                       test_parameters.welding_parameters.stickout_m);
  help_sim::ConfigLPCS(sim_config, test_parameters.welding_parameters.stickout_m, SCANNER_MOUNT_ANGLE);

  simulator->Initialize(sim_config);

  CalibrateConfig conf{.stickout_m              = test_parameters.welding_parameters.stickout_m,
                       .touch_point_depth_m     = TOUCH_POINT_DEPTH_M,
                       .scanner_mount_angle_rad = SCANNER_MOUNT_ANGLE,
                       .wire_diameter_mm        = test_parameters.welding_parameters.weld_system_1.wire_diameter_mm,
                       .weld_object_diameter_m  = test_parameters.welding_parameters.weld_object_diameter_m};

  CHECK(Calibrate(fixture, sim_config, *simulator, conf));

  depsim::Point3d torch_pos_macs(-30e-3, 0, -19e-3 + test_parameters.welding_parameters.stickout_m, depsim::MACS);
  simulator->UpdateTorchPosition(torch_pos_macs);
  TESTLOG("Set DepSim torch start position MACS, vertical {}m, horizontal: {}m", torch_pos_macs.GetX(),
          torch_pos_macs.GetZ());

  //
  // set up adaptio
  //
  auto abws        = helpers_simulator::ConvertFromOptionalAbwVector(simulator->GetAbwPoints(depsim::MACS));
  auto joint_depth = std::abs(abws[0].GetZ() - abws[1].GetZ());  // Approximate depth
  for (auto &p : abws) {
    joint_depth = std::max(std::abs(abws[0].GetZ() - p.GetZ()), joint_depth);
  }
  DumpAbw(abws);
  simulator->UpdateTorchPosition(torch_pos_macs);
  TESTLOG("Set DepSim torch position MACS (1st fill layer), horizontal {:.5f}m, vertical: {:.5f}m",
          torch_pos_macs.GetX(), torch_pos_macs.GetZ());

  {
    // Store ABP parameters (don't exist in DepSim)
    auto json_step_up_limits = nlohmann::json::array();

    for (auto const &step_up_limit : test_parameters.abp_parameters.step_up_limits) {
      json_step_up_limits.push_back(step_up_limit);
    }
    auto const payload = nlohmann::json({
        {"wallOffset",         test_parameters.abp_parameters.wall_offset_mm},
        {"beadOverlap",        test_parameters.abp_parameters.bead_overlap  },
        {"stepUpValue",        test_parameters.abp_parameters.step_up_value },
        {"kGain",              test_parameters.abp_parameters.k_gain        },
        {"heatInput",
         {
             {"min", test_parameters.abp_parameters.heat_input.min},
             {"max", test_parameters.abp_parameters.heat_input.max},
         }                                                                  },
        {"weldSystem2Current",
         {
             {"min", test_parameters.abp_parameters.weld_system_2_current.min},
             {"max", test_parameters.abp_parameters.weld_system_2_current.max},
         }                                                                  },
        {"stepUpLimits",       json_step_up_limits                          },
    });
    StoreABPParams(fixture, payload, EXPECT_OK);
  }

  // Start Joint tracking
  common::msg::management::TrackingStart start_joint_tracking_msg{
      .joint_tracking_mode = static_cast<uint32_t>(tracking::TrackingMode::TRACKING_LEFT_HEIGHT),
      .horizontal_offset   = 10.0,
      .vertical_offset     = static_cast<float>(help_sim::ConvertM2Mm(test_parameters.welding_parameters.stickout_m))};
  fixture.Management()->Dispatch(start_joint_tracking_msg);

  // Receive StartScanner
  REQUIRE_MESSAGE(fixture.Scanner()->Receive<common::msg::scanner::Start>(), "No Start msg received");

  CheckWeldControlStatus(fixture, "jt");

  DispatchKinematicsStateChange(fixture, common::msg::kinematics::StateChange::State::HOMED);
  DispatchKinematicsEdgeStateChange(fixture, common::msg::kinematics::EdgeStateChange::State::AVAILABLE);
  CheckAndDispatchGetWeldAxis(fixture, 0.0, 0.0, help_sim::ConvertM2Mm(sim_config.joint_def_left.outer_diameter / 2.));

  StartABP(fixture);
  CheckWeldControlStatus(fixture, "abp");

  DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID1,
                                common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);
  DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID2,
                                common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);

  //
  //  Execute ABP welding
  //
  auto test_ready         = false;
  auto handover_to_manual = false;
  StateMonitor<std::string> state_monitor;
  std::map<int, int> beads_per_layer;
  while (!test_ready) {
    abws        = helpers_simulator::ConvertFromOptionalAbwVector(simulator->GetAbwPoints(depsim::MACS));
    joint_depth = std::abs(abws[0].GetZ() - abws[1].GetZ());  // Approximate depth
    for (auto &p : abws) {
      joint_depth = std::max(std::abs(abws[0].GetZ() - p.GetZ()), joint_depth);
    }

    DumpAbw(abws);

    double current_angle{.0};
    for (int step = 0; step < NUMBER_OF_STEPS_PER_REV; step++)  // Steps per rev
    {
      abws = helpers_simulator::ConvertFromOptionalAbwVector(simulator->GetAbwPoints(depsim::LPCS));
      //
      // Update adaptio
      //
      // ABW points on scanner interface
      auto slice_data = GetSliceData(
          abws,
          static_cast<std::uint64_t>(fixture.GetClockNowFuncWrapper()->GetSystemClock().time_since_epoch().count()));
      fixture.Scanner()->Dispatch(slice_data);

      // Receive GetSlidesPosition
      auto get_position = fixture.Kinematics()->Receive<common::msg::kinematics::GetSlidesPosition>();

      // GetSlidePosition response
      fixture.Kinematics()->Dispatch(
          common::msg::kinematics::GetSlidesPositionRsp{.client_id  = get_position->client_id,
                                                        .time_stamp = get_position->time_stamp,
                                                        .horizontal = help_sim::ConvertM2Mm(torch_pos_macs.GetX()),
                                                        .vertical   = help_sim::ConvertM2Mm(torch_pos_macs.GetZ())});
      TESTLOG("Set Adaptio torch position MACS, horizontal {:.5f}mm, vertical: {:.5f}mm",
              help_sim::ConvertM2Mm(torch_pos_macs.GetX()), help_sim::ConvertM2Mm(torch_pos_macs.GetZ()));

      if (!fixture.Management()->Receive<common::msg::management::ScannerError>()) {
        //  Check GetWeldAxis request and dispatch response
        CheckAndDispatchGetWeldAxis(
            fixture, current_angle,
            help_sim::ConvertMPerS2RadPerS(sim_config.travel_speed, sim_config.joint_def_left.outer_diameter / 2.),
            help_sim::ConvertM2Mm(sim_config.joint_def_left.outer_diameter / 2.));

        // Check GetWeldSystemStatus requests for both weld-systems and send response
        CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID1, weld_system_1_status_rsp);
        CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID2, weld_system_2_status_rsp);

        // Dispatch expired timers
        fixture.GetTimerWrapper()->DispatchAllExpired();
        if (fixture.Management()->Receive<common::msg::management::NotifyHandoverToManual>()) {
          TESTLOG("Received handover-to-manual - wait for stop to exit test case!");
          handover_to_manual = true;
        }
        //
        // Check adaptio output
        //
        // Receive SetSlidesPosition and update DepSim torch position
        auto set_slides_position = fixture.Kinematics()->Receive<common::msg::kinematics::SetSlidesPosition>();
        REQUIRE_MESSAGE(set_slides_position, "No SetSlidesPosition received");

        // auto update_scanner = fixture.Kinematics()->Receive<common::msg::scanner::Update>();
        // REQUIRE_MESSAGE(update_scanner, "No Update scanner received");

        torch_pos_macs = depsim::Point3d(help_sim::ConvertMm2M(set_slides_position->horizontal), 0,
                                         help_sim::ConvertMm2M(set_slides_position->vertical), depsim::MACS);
        simulator->UpdateTorchPosition(torch_pos_macs);
        TESTLOG("Adaptio to set DepSim torch position MACS, horizontal {:.5f}m, vertical: {:.5f}m",
                torch_pos_macs.GetX(), torch_pos_macs.GetZ());

        {  // Check ABP weld control status
          auto weld_control_status = GetWeldControlStatus(fixture);
          if (weld_control_status.weld_control_mode.has_value()) {
            REQUIRE_MESSAGE(weld_control_status.weld_control_mode.value() == "abp", "Unexpected weld control state");
          }
          // Check bead control state by monitoring state changes
          if (step == START_BEAD_STATE_MONITORING_AT_STEP) {
            state_monitor.Clear();
          }
          if (step > START_BEAD_STATE_MONITORING_AT_STEP || step < CHECK_MONITORED_BEAD_STATE_AT_STEP) {
            state_monitor.Add(weld_control_status.bead_operation.value());
          }

          beads_per_layer[weld_control_status.layer_number.value()] = weld_control_status.bead_number.value();
        }
        // Step position
        current_angle += DELTA_ANGLE;
        simulator->RunWithRotation(DELTA_ANGLE,
                                   test_parameters.test_joint_geometry.simulator_joint_geometry.bead_radians_m);
        // Step clocks
        fixture.GetClockNowFuncWrapper()->StepSystemClock(time_per_step_ms);
        fixture.GetClockNowFuncWrapper()->StepSteadyClock(time_per_step_ms);

      } else {
        TESTLOG("Received Stop - exit test!");
        test_ready = true;
        break;
      }
    }
  }

  REQUIRE_MESSAGE(handover_to_manual, "handover to manual signal missing!");

  std::ostringstream error_string;
  error_string << "Unexpected number of beads layer\nactual:   ";
  for (auto [layer, beads] : beads_per_layer) {
    error_string << "{" << layer << ", " << beads << "} ";
  }

  error_string << "\nexpected: ";
  for (auto [layer, beads] : test_parameters.testcase_parameters.expected_beads_in_layer) {
    error_string << "{" << layer << ", " << beads << "} ";
  }

  REQUIRE_MESSAGE(beads_per_layer == test_parameters.testcase_parameters.expected_beads_in_layer, error_string.str());
}
}  // End namespace
// NOLINTEND(*-magic-numbers, *-optional-access)
