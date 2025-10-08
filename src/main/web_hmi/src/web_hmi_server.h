#pragma once

#include <memory>
#include <regex>
#include <prometheus/gauge.h>
#include <prometheus/registry.h>

#include "../web_hmi.h"
#include "calibration/calibration_manager.h"
#include "common/zevs/zevs_core.h"
#include "coordination/activity_status.h"
#include "joint_geometry/joint_geometry_provider.h"
#include "kinematics/kinematics_client.h"
#include "macs/macs_point.h"
#include "macs/macs_slice.h"
#include "slice_translator/slice_observer.h"
#include "web_hmi_calibration.h"

namespace web_hmi {

class WebHmiServer : public slice_translator::SliceObserver, public WebHmi {
 public:
  WebHmiServer(zevs::CoreSocket* in_socket, zevs::CoreSocket* out_socket,
               joint_geometry::JointGeometryProvider* joint_geometry_provider,
               kinematics::KinematicsClient* kinematics_client, coordination::ActivityStatus* activity_status,
               prometheus::Registry* registry);

  void OnMessage(zevs::MessagePtr message);

  // SliceObserver
  void Receive(const macs::Slice& machine_data, const lpcs::Slice& scanner_data, const macs::Point& axis_position,
               const double angle_from_torch_to_scanner) override;

  // WebHmi interface
  void Subscribe(std::string const& topic, OnRequest on_request) override;
  void SubscribePattern(std::regex const& pattern, OnRequest on_request) override;
  void Send(nlohmann::json const& data) override;
  void Send(std::string const& topic, nlohmann::json const& payload) override;

 private:
  void GetSlidesPositionRsp(double horizontal, double vertical);

  void SetupMetrics(prometheus::Registry* registry);
  void UpdateJointMetrics(const macs::Groove& groove);

  zevs::CoreSocket* in_socket_;
  zevs::CoreSocket* out_socket_;
  kinematics::KinematicsClient* kinematics_client_;
  coordination::ActivityStatus* activity_status_;

  // Last groove estimate
  std::optional<macs::Groove> groove_;

  struct {
    prometheus::Gauge* top_width_mm{nullptr};
    prometheus::Gauge* bottom_width_mm{nullptr};
    prometheus::Gauge* left_depth_mm{nullptr};
    prometheus::Gauge* right_depth_mm{nullptr};
    prometheus::Gauge* avg_depth_mm{nullptr};
    prometheus::Gauge* top_edge_vertical_diff_mm{nullptr};
    prometheus::Gauge* area_mm2{nullptr};
    prometheus::Gauge* left_wall_angle_rad{nullptr};
    prometheus::Gauge* right_wall_angle_rad{nullptr};
  } metrics_;

  auto CheckSubscribers(std::string const& topic, nlohmann::json const& payload) -> bool;

  struct Subscriber {
    std::regex pattern;
    OnRequest on_request;
  };
  std::vector<Subscriber> subscribers_;
};

}  // namespace web_hmi
