#include "web_hmi_calibration.h"

#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>
#include <string>
#include <utility>

#include "calibration/calibration_manager.h"
#include "calibration/calibration_types.h"
#include "common/logging/application_log.h"
#include "common/zevs/zevs_core.h"
#include "coordination/activity_status.h"
#include "joint_geometry/joint_geometry_provider.h"
#include "web_hmi/src/json_payload.h"
#include "web_hmi/web_hmi_json_helpers.h"

using web_hmi::WebHmiCalibration;
const std::string ADAPTIO_IO = "adaptio_io";

WebHmiCalibration::WebHmiCalibration(zevs::CoreSocket* socket, calibration::CalibrationManager* calibration_manager,
                                     joint_geometry::JointGeometryProvider* joint_geometry_provider,
                                     coordination::ActivityStatus* activity_status)
    : socket_(socket),
      calibration_manager_(calibration_manager),
      joint_geometry_provider_(joint_geometry_provider),
      activity_status_(activity_status) {
  if (calibration_manager_) {
    calibration_manager_->SetObserver(this);
  }
}

void WebHmiCalibration::OnMessage(const std::string& message_name, const nlohmann::json& payload) {
  // Legacy calibration message handlers - return failure responses since v1 calibration is removed
  // The v2 calibration system (CalibrationManagerV2Impl) handles its own web HMI messages directly
  
  if (message_name == "LaserToTorchCalibration") {
    LOG_ERROR("Legacy LaserToTorchCalibration not supported - use LaserTorchCalGet/LaserTorchCalSet instead");
    auto response_payload = LaserTorchCalibrationToPayload(false, {});
    auto message = CreateMessage("LaserToTorchCalibrationRsp", response_payload);
    socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
    
  } else if (message_name == "GetLaserToTorchCalibration") {
    LOG_ERROR("Legacy GetLaserToTorchCalibration not supported - use LaserTorchCalGet instead");
    auto response_payload = LaserTorchCalibrationToPayload(false, {});
    auto message = CreateMessage("GetLaserToTorchCalibrationRsp", response_payload);
    socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
    
  } else if (message_name == "SetLaserToTorchCalibration") {
    LOG_ERROR("Legacy SetLaserToTorchCalibration not supported - use LaserTorchCalSet instead");
    auto response_payload = ResultPayload(false);
    auto message = CreateMessage("SetLaserToTorchCalibrationRsp", response_payload);
    socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
    
  } else if (message_name == "WeldObjectCalibration") {
    LOG_ERROR("Legacy WeldObjectCalibration not supported - use WeldObjectCalStart/WeldObjectCalLeftPos/WeldObjectCalRightPos sequence instead");
    auto response_payload = WeldObjectCalibrationToPayload(false, {});
    auto message = CreateMessage("WeldObjectCalibrationRsp", response_payload);
    socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
    
  } else if (message_name == "GetWeldObjectCalibration") {
    LOG_ERROR("Legacy GetWeldObjectCalibration not supported - use WeldObjectCalGet instead");
    auto response_payload = WeldObjectCalibrationToPayload(false, {});
    auto message = CreateMessage("GetWeldObjectCalibrationRsp", response_payload);
    socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));

  } else if (message_name == "SetWeldObjectCalibration") {
    LOG_ERROR("Legacy SetWeldObjectCalibration not supported - use WeldObjectCalSet instead");
    auto response_payload = ResultPayload(false);
    auto message = CreateMessage("SetWeldObjectCalibrationRsp", response_payload);
    socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));

  } else {
    LOG_ERROR("Unknown Message: message_name={}", message_name);
  }
}

void WebHmiCalibration::LaserTorchCalibrationCompleted(const calibration::LaserTorchCalibration& data) {
  LOG_DEBUG("LaserToTorchCalibration completed");

  auto payload = LaserTorchCalibrationToPayload(true, data);
  auto message = CreateMessage("LaserToTorchCalibrationRsp", payload);
  socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));

  activity_status_->Set(coordination::ActivityStatusE::IDLE);
}

void WebHmiCalibration::LaserTorchCalibrationFailed() {
  LOG_DEBUG("LaserToTorchCalibration failed");

  auto payload = LaserTorchCalibrationToPayload(false, {});
  auto message = CreateMessage("LaserToTorchCalibrationRsp", payload);
  socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));

  activity_status_->Set(coordination::ActivityStatusE::IDLE);
}

void WebHmiCalibration::WeldObjectCalibrationCompleted(const calibration::WeldObjectCalibration& data) {
  LOG_DEBUG("WeldObjectCalibration completed");

  auto payload = WeldObjectCalibrationToPayload(true, data);
  auto message = CreateMessage("WeldObjectCalibrationRsp", payload);
  socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));

  activity_status_->Set(coordination::ActivityStatusE::IDLE);
}

void WebHmiCalibration::WeldObjectCalibrationFailed() {
  LOG_DEBUG("WeldObjectCalibration failed");

  auto payload = WeldObjectCalibrationToPayload(false, {});
  auto message = CreateMessage("WeldObjectCalibrationRsp", payload);
  socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));

  activity_status_->Set(coordination::ActivityStatusE::IDLE);
}
