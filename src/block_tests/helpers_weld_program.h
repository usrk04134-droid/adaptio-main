#pragma once

#include <nlohmann/detail/value_t.hpp>

#include "block_tests/helpers_web_hmi.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)

#include <doctest/doctest.h>
#include <SQLiteCpp/Database.h>
#include <SQLiteCpp/Statement.h>
#include <SQLiteCpp/VariadicBind.h>

#include <string>

#include "helpers.h"
#include "web_hmi/web_hmi_json_helpers.h"

inline const std::string WPROG_STORE     = "StoreWeldProgram";
inline const std::string WPROG_STORE_RSP = "StoreWeldProgramRsp";
inline const std::string WPROG_GET       = "GetWeldPrograms";
inline const std::string WPROG_GET_RSP   = "GetWeldProgramsRsp";

[[nodiscard]] inline auto StoreWeldProgram(TestFixture& fixture, nlohmann::json const& payload, bool expect_ok)
    -> bool {
  auto msg = web_hmi::CreateMessage(WPROG_STORE, payload);
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto const response_payload = ReceiveJsonByName(fixture, WPROG_STORE_RSP);
  CHECK(response_payload != nullptr);

  auto const expected = expect_ok
    ? nlohmann::json { { "result", "ok"} }
    : nlohmann::json { { "result", "fail"} };

  return response_payload == expected;
}

[[nodiscard]] inline auto CheckWeldProgramsEqual(TestFixture& fixture, nlohmann::json const& expected) -> bool {
  auto msg = web_hmi::CreateMessage(WPROG_GET, {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto const response_payload = ReceiveJsonByName(fixture, WPROG_GET_RSP);
  return response_payload != nullptr && response_payload == expected;
}

// NOLINTEND(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)
