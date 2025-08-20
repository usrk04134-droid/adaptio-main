#include "web_hmi_calibration.h"

#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>
#include <string>
#include <utility>

#include "calibration/src/calibration_manager_v2_impl.h"
#include "calibration/calibration_types.h"
#include "common/logging/application_log.h"
#include "common/zevs/zevs_core.h"
#include "coordination/activity_status.h"
#include "joint_geometry/joint_geometry_provider.h"
#include "web_hmi/src/json_payload.h"
#include "web_hmi/web_hmi_json_helpers.h"

using web_hmi::WebHmiCalibration;
const std::string ADAPTIO_IO = "adaptio_io";

WebHmiCalibration::WebHmiCalibration(zevs::CoreSocket* socket, void* /*unused_calibration_manager*/,
                                     joint_geometry::JointGeometryProvider* joint_geometry_provider,
                                     coordination::ActivityStatus* activity_status)
    : socket_(socket),
      calibration_manager_(nullptr),
      joint_geometry_provider_(joint_geometry_provider),
      activity_status_(activity_status) {
  // Legacy v1 manager removed; v2 registers its own WebHMI subscriptions
}

void WebHmiCalibration::OnMessage(const std::string& message_name, const nlohmann::json& payload) {
  if (message_name == "LaserToTorchCalibration") {
    if (!activity_status_->IsIdle()) {
      // In this version do nothing except log
      LOG_ERROR("Cannot start LaserToTorchCalibration - busy with status: {}", activity_status_->ToString());
      return;
    }

    double offset                   = payload.at("offset").get<double>();
    double angle                    = payload.at("angle").get<double>();
    double stickout                 = payload.at("stickout").get<double>();
    auto calibration_joint_geometry = joint_geometry_provider_->GetFixtureJointGeometry();
    // Legacy API removed. Request is ignored; use v2 endpoints (LaserTorchCalSet / Get) instead.

    activity_status_->Set(coordination::ActivityStatusE::LASER_TORCH_CALIBRATION);

  } else if (message_name == "GetLaserToTorchCalibration") {
    // Legacy API removed. Respond with invalid.
    std::optional<calibration::LaserTorchCalibration> calibration;
    bool valid       = false;
    if (calibration) {
      valid = true;
    } else {
      calibration = calibration::LaserTorchCalibration{};
    }
    auto payload = LaserTorchCalibrationToPayload(valid, calibration.value());
    auto message = CreateMessage("GetLaserToTorchCalibrationRsp", payload);
    socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
  } else if (message_name == "SetLaserToTorchCalibration") {
    if (!activity_status_->IsIdle()) {
      // In this version do nothing except log
      LOG_ERROR("Cannot set LaserToTorchCalibration - busy with status: {}", activity_status_->ToString());
      return;
    }
    auto calibration = LaserTorchCalibrationFromPayload(payload);
    // Legacy API removed. Always fail.
    auto payload     = ResultPayload(false);
    auto message     = CreateMessage("SetLaserToTorchCalibrationRsp", payload);
    socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
  } else if (message_name == "WeldObjectCalibration") {
    if (!activity_status_->IsIdle()) {
      LOG_ERROR("Cannot start WeldObjectCalibration - busy with status: {}", activity_status_->ToString());
      auto payload = WeldObjectCalibrationToPayload(false, {});
      auto message = CreateMessage("WeldObjectCalibrationRsp", payload);
      socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
      return;
    }

    double radius       = payload.at("radius").get<double>();
    double stickout     = payload.at("stickout").get<double>();
    auto joint_geometry = joint_geometry_provider_->GetJointGeometry();
    if (!joint_geometry.has_value()) {
      LOG_ERROR("Joint Geometry unavailable");
      return;
    }

    // Legacy API removed. Use v2 procedures (WeldObjectCalStart/LeftPos/RightPos) instead.

    activity_status_->Set(coordination::ActivityStatusE::WELD_OBJECT_CALIBRATION);
  } else if (message_name == "GetWeldObjectCalibration") {
    // Legacy API removed. Respond with invalid.
    std::optional<calibration::WeldObjectCalibration> calibration;
    bool valid       = false;
    if (calibration) {
      valid = true;
    } else {
      calibration = calibration::WeldObjectCalibration{};
    }
    auto payload = WeldObjectCalibrationToPayload(valid, calibration.value());
    auto message = CreateMessage("GetWeldObjectCalibrationRsp", payload);
    socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));

  } else if (message_name == "SetWeldObjectCalibration") {
    // Legacy API removed. Always fail.
    auto payload     = ResultPayload(false);
    auto message     = CreateMessage("SetWeldObjectCalibrationRsp", payload);
    socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));

  } else {
    LOG_ERROR("Unknown Message: message_name={}", message_name);
  }
}

// Legacy observer callbacks removed
