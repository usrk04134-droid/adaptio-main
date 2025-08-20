#pragma once

// Legacy v1 CalibrationManager removed
#include "common/zevs/zevs_core.h"
#include "coordination/activity_status.h"
#include "joint_geometry/joint_geometry_provider.h"
#include "service.h"

namespace web_hmi {

class WebHmiCalibration : public Service {
 public:
  explicit WebHmiCalibration(zevs::CoreSocket* socket, void* /*unused_calibration_manager*/,
                             joint_geometry::JointGeometryProvider* joint_geometry_provider,
                             coordination::ActivityStatus* activity_status);

  // Service
  void OnMessage(const std::string& message_name, const nlohmann::json& payload) override;

 private:
  zevs::CoreSocket* socket_;
  void* calibration_manager_;
  joint_geometry::JointGeometryProvider* joint_geometry_provider_;
  coordination::ActivityStatus* activity_status_;
};

}  // namespace web_hmi
