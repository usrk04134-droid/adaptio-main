#include "web_hmi_server.h"

#include <cstdint>
#include <cstring>
#include <memory>
#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>
#include <regex>
#include <string>
#include <utility>

#include "../web_hmi_json_helpers.h"
#include "calibration/calibration_manager.h"
#include "common/logging/application_log.h"
#include "common/zevs/zevs_core.h"
#include "coordination/activity_status.h"
#include "joint_geometry/joint_geometry_provider.h"
#include "json_payload.h"
#include "kinematics/kinematics_client.h"
#include "lpcs/lpcs_slice.h"
#include "macs/macs_point.h"
#include "macs/macs_slice.h"
#include "version.h"
#include "web_hmi/src/web_hmi_calibration.h"
#include "web_hmi/web_hmi.h"

namespace {

const std::string ADAPTIO_IO = "adaptio_io";

}  // namespace

using web_hmi::WebHmiServer;

WebHmiServer::WebHmiServer(zevs::CoreSocket* in_socket, zevs::CoreSocket* out_socket,
                           joint_geometry::JointGeometryProvider* joint_geometry_provider,
                           kinematics::KinematicsClient* kinematics_client,
                           coordination::ActivityStatus* activity_status,
                           prometheus::Registry* registry)
    : in_socket_(in_socket),
      out_socket_(out_socket),
      kinematics_client_(kinematics_client),
      activity_status_(activity_status) {
  LOG_DEBUG("Starting WebHmiServer");
  auto handler = [this](zevs::MessagePtr msg) { this->OnMessage(std::move(msg)); };
  in_socket_->SetHandler(handler);
  SetupMetrics(registry);
}

auto WebHmiServer::CheckSubscribers(std::string const& topic, nlohmann::json const& payload) -> bool {
  bool found = false;
  for (const Subscriber& sub : subscribers_) {
    if (std::regex_match(topic, sub.pattern)) {
      sub.on_request(topic, payload);
      found = true;
    }
  }
  return found;
}

void WebHmiServer::OnMessage(zevs::MessagePtr message) {
  std::string message_name;

  try {
    nlohmann::json payload;
    UnpackMessage(message, message_name, payload);

    if (CheckSubscribers(message_name, payload)) {
      return;
    }

    if (message_name == "GetAdaptioVersion") {
      auto response = CreateMessage("GetAdaptioVersionRsp", VersionToPayload(ADAPTIO_VERSION));
      out_socket_->SendWithEnvelope(ADAPTIO_IO, std::move(response));

    } else if (message_name == "GetSlidesPosition") {
      auto on_get_slides_position = [this](std::uint64_t /*time_stamp*/, double horizontal, double vertical) {
        auto payload = PositionToPayload(horizontal, vertical);
        auto message = CreateMessage("GetSlidesPositionRsp", payload);
        out_socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
      };
      kinematics_client_->GetSlidesPosition(on_get_slides_position);
    } else if (message_name == "GetSlidesStatus") {
      auto on_get_slides_status = [this](bool horizontal_in_position, bool vertical_in_position) {
        auto payload = SlidesStatusToPayload(horizontal_in_position, vertical_in_position);
        auto message = CreateMessage("GetSlidesStatusRsp", payload);
        out_socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
      };
      kinematics_client_->GetSlidesStatus(on_get_slides_status);
    } else if (message_name == "GetActivityStatus") {
      auto payload = ActivityStatusToPayload(activity_status_->Get());
      auto message = CreateMessage("GetActivityStatusRsp", payload);
      out_socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));

    } else if (message_name == "GetGroove") {
      nlohmann::json payload = nlohmann::json::object();
      LOG_DEBUG("GetGroove: groove_ available: {}", groove_->ToString());
      if (groove_) {
        payload = GrooveToPayload(groove_.value());
      }
      auto message = CreateMessage("GetGrooveRsp", payload);
      out_socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
    } else if (message_name == "GetEdgePosition") {
      auto on_get_edge_position = [this](double position) {
        nlohmann::json payload = {
            {"position", position}
        };
        auto message = CreateMessage("GetEdgePositionRsp", payload);
        out_socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
      };
      kinematics_client_->GetEdgePosition(on_get_edge_position);
    } else {
      LOG_ERROR("Unknown Message: message_name={}", message_name);
    }
  } catch (nlohmann::json::exception& exception) {
    LOG_ERROR("nlohman JSON exception: {}, for message: {}", exception.what(), message_name);
  }
}

// SliceObserver
void WebHmiServer::Receive(const macs::Slice& data, const lpcs::Slice& /*scanner_data*/,
                           const macs::Point& /*axis_position*/, const double /*angle_from_torch_to_scanner*/) {
  // could be an empty optional
  groove_ = data.groove;
  LOG_DEBUG("GetGroove: groove_ available: {}", groove_->ToString());
  if (groove_) {
    UpdateJointMetrics(*groove_);
  }
}

void WebHmiServer::SetupMetrics(prometheus::Registry* registry) {
  if (registry == nullptr) {
    return;
  }

  auto& gauge_family = prometheus::BuildGauge()
                           .Name("adaptio_joint_characteristic")
                           .Help("Joint characteristics derived from ABW points (mm, rad, mm2)")
                           .Register(*registry);

  metrics_.top_width_mm = &gauge_family.Add({{"name", "top_width_mm"}});
  metrics_.bottom_width_mm = &gauge_family.Add({{"name", "bottom_width_mm"}});
  metrics_.left_depth_mm = &gauge_family.Add({{"name", "left_depth_mm"}});
  metrics_.right_depth_mm = &gauge_family.Add({{"name", "right_depth_mm"}});
  metrics_.avg_depth_mm = &gauge_family.Add({{"name", "avg_depth_mm"}});
  metrics_.top_edge_vertical_diff_mm = &gauge_family.Add({{"name", "top_edge_vertical_diff_mm"}});
  metrics_.area_mm2 = &gauge_family.Add({{"name", "area_mm2"}});
  metrics_.left_wall_angle_rad = &gauge_family.Add({{"name", "left_wall_angle_rad"}});
  metrics_.right_wall_angle_rad = &gauge_family.Add({{"name", "right_wall_angle_rad"}});
}

void WebHmiServer::UpdateJointMetrics(const macs::Groove& groove) {
  if (!metrics_.top_width_mm) {
    return;
  }

  const double top_width = groove.TopWidth();
  const double bottom_width = groove.BottomWidth();
  const double left_depth = groove.LeftDepth();
  const double right_depth = groove.RightDepth();
  const double avg_depth = groove.AvgDepth();
  const double area = groove.Area();
  const double left_angle = groove.LeftWallAngle();
  const double right_angle = groove.RightWallAngle();

  // Top edge vertical difference: ABW0 - ABW6 vertical
  const double top_edge_vertical_diff = groove[macs::ABW_UPPER_LEFT].vertical - groove[macs::ABW_UPPER_RIGHT].vertical;

  metrics_.top_width_mm->Set(top_width);
  metrics_.bottom_width_mm->Set(bottom_width);
  metrics_.left_depth_mm->Set(left_depth);
  metrics_.right_depth_mm->Set(right_depth);
  metrics_.avg_depth_mm->Set(avg_depth);
  metrics_.top_edge_vertical_diff_mm->Set(top_edge_vertical_diff);
  metrics_.area_mm2->Set(area);
  metrics_.left_wall_angle_rad->Set(left_angle);
  metrics_.right_wall_angle_rad->Set(right_angle);
}

void WebHmiServer::SubscribePattern(std::regex const& pattern, OnRequest on_request) {
  /* allow for multiple subscribers for the same topic */
  const Subscriber sub = {
      .pattern    = pattern,
      .on_request = std::move(on_request),
  };

  subscribers_.push_back((sub));
}

void WebHmiServer::Subscribe(std::string const& topic, OnRequest on_request) {
  SubscribePattern(std::regex(topic), on_request);
}

void WebHmiServer::Send(nlohmann::json const& data) {
  auto jstr = data.dump();

  LOG_TRACE("web_hmi::Send data: {}", jstr.c_str());

  auto message = zevs::GetCoreFactory()->CreateRawMessage(jstr.size());
  std::memcpy(message->Data(), jstr.c_str(), jstr.size());
  out_socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
}

void WebHmiServer::Send(std::string const& topic, nlohmann::json const& payload) {
  LOG_TRACE("web_hmi::Send topic: {} payload: {}", topic.c_str(), payload.dump());

  auto message = CreateMessage(topic, payload);
  out_socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
}
