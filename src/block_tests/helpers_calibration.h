#pragma once

// NOLINTBEGIN(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)

#include <doctest/doctest.h>

#include "block_tests/helpers_web_hmi.h"
#include "helpers.h"
#include "web_hmi/web_hmi_json_helpers.h"

inline auto SetJointGeometry(TestFixture& fixture, nlohmann::json const& payload) {
  auto msg = web_hmi::CreateMessage("SetJointGeometry", payload);
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto joint_geometry_rsp_payload = ReceiveJsonByName(fixture, "SetJointGeometryRsp");
  CHECK(joint_geometry_rsp_payload != nullptr);
}

// NOLINTEND(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)
