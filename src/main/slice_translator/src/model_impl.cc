#include "model_impl.h"

#include <array>
#include <cmath>
#include <Eigen/Eigen>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <vector>

#include "common/geometric_primitives/src/circle3d.h"
#include "common/geometric_primitives/src/coordinate-systems.h"
#include "common/geometric_primitives/src/line3d.h"
#include "common/geometric_primitives/src/plane3d.h"
#include "common/geometric_primitives/src/point3d.h"
#include "common/logging/application_log.h"
#include "common/types/vector_3d.h"
#include "common/types/vector_3d_helpers.h"
#include "lpcs/lpcs_point.h"
#include "macs/macs_point.h"

using Eigen::Matrix3d;
using Eigen::Vector3d;
using geometric_primitives::Circle3d;
using geometric_primitives::CoordinateSystem;
using geometric_primitives::Line3d;
using geometric_primitives::LPCS;
using geometric_primitives::MACS;
using geometric_primitives::Plane3d;
using geometric_primitives::Point3d;
using slice_translator::ModelImpl;

namespace slice_translator {

auto ModelImpl::LPCSToMCS(const std::vector<lpcs::Point>& lpcs_points, const macs::Point& slide_position)
    -> std::optional<std::vector<macs::Point>> {
  std::vector<macs::Point> mcs_points;
  macs::Point mcs_point;

    for (std::size_t i = 0; i < lpcs_points.size(); ++i) {
        LOG_DEBUG("groove[{}]: ({:.6f}, {:.6f})", i, lpcs_points[i].x, lpcs_points[i].y);
      }

  for (const auto& lpcs_point : lpcs_points) {
    mcs_point = TransformAndRotateToTorchPlane(rot_center_, {scanner_mount_angle_, delta_rot_y_, delta_rot_z_},
                                               weld_object_rotation_axis_, torch_to_laser_translation_, lpcs_point,
                                               slide_position);
    mcs_points.push_back(mcs_point);
  }

  return mcs_points;
}

auto ModelImpl::MCSToLPCS(const std::vector<macs::Point>& mcs_points, const macs::Point& slide_position)
    -> std::optional<std::vector<lpcs::Point>> {
  std::vector<lpcs::Point> lpcs_points;
  lpcs::Point lpcs_point;

  for (std::size_t i = 0; i < lpcs_points.size(); ++i) {
        LOG_DEBUG("MCSToLPCS[{}]: ({:.6f}, {:.6f})", i, lpcs_points[i].x, lpcs_points[i].y);
      }
  for (const auto& mcs_point : mcs_points) {
    lpcs_point = TransformAndRotateToLaserPlane(rot_center_, {scanner_mount_angle_, delta_rot_y_, delta_rot_z_},
                                                weld_object_rotation_axis_, torch_to_laser_translation_, mcs_point,
                                                slide_position);
    lpcs_points.push_back(lpcs_point);
  }

  return lpcs_points;
}
auto ModelImpl::TransformAndRotateToLaserPlane(const common::Vector3D& rot_center, std::array<double, 3> scanner_angles,
                                               const common::Vector3D& weld_object_rotation_axis,
                                               const common::Vector3D& torch_to_laser_translation,
                                               macs::Point point_macs, macs::Point slide_position) const
    -> lpcs::Point {
  // Define laser plane to project/rotate onto (MACS)
  Point3d laser_plane_point_macs =
      TransformLPCStoMACS(scanner_angles, torch_to_laser_translation, slide_position, {0.0, 0.0, 0.0, LPCS});
  
  Point3d laser_plane_normal_macs =
      TransformLPCStoMACS(scanner_angles, torch_to_laser_translation, slide_position, {0.0, 0.0, 1.0, LPCS}, false);
  Plane3d laser_plane_macs{laser_plane_normal_macs.ToVec(), laser_plane_point_macs};

  LOG_DEBUG("laser_plane_normal_macs: x={:.6f}, y={:.6f}, z={:.6f}, ref={}",
            laser_plane_normal_macs.GetX(),
            laser_plane_normal_macs.GetY(),
            laser_plane_normal_macs.GetZ(),
             static_cast<int>(laser_plane_point_macs.GetRefSystem()));

  // The point (MACS) to project and transform to LPCS
  Point3d point_to_project_macs{point_macs.horizontal, 0.0, point_macs.vertical, MACS};

  // The projection circle (MACS)
  Circle3d projection_circle_macs =
      CreateProjectionCircle(rot_center, weld_object_rotation_axis, point_to_project_macs);
  Point3d mcs_point = RotateToPlane(projection_circle_macs, laser_plane_macs);

  // Convert the projected point to LPCS
  Point3d projected_point_lpcs =
      TransformMACStoLPCS(scanner_angles, torch_to_laser_translation, slide_position, mcs_point);

  return {.x = projected_point_lpcs.GetX(), .y = projected_point_lpcs.GetY()};
}

auto ModelImpl::TransformAndRotateToTorchPlane(const common::Vector3D& rot_center, std::array<double, 3> scanner_angles,
                                               const common::Vector3D& weld_object_rotation_axis,
                                               const common::Vector3D& torch_to_laser_translation,
                                               lpcs::Point point_lpcs, macs::Point slide_position) const
    -> macs::Point {
  // Torch plane to project/rotate onto (MACS)
  Point3d point_in_torch_plane{0.0, 0.0, 0.0, CoordinateSystem::MACS};
  Plane3d torch_plane{
      {0, 1, 0},
      point_in_torch_plane
  };

  // Transform the point from LPCS to MACS
  Point3d point_to_project_macs = TransformLPCStoMACS(scanner_angles, torch_to_laser_translation, slide_position,
                                                      {point_lpcs.x, point_lpcs.y, 0.0, LPCS});

  // The projection circle defined in MACS
  Circle3d projection_circle = CreateProjectionCircle(rot_center, weld_object_rotation_axis, point_to_project_macs);

  // Do the projection to torch plane.
  Point3d mcs_point = RotateToPlane(projection_circle, torch_plane);

  return {.horizontal = mcs_point.GetX(), .vertical = mcs_point.GetZ()};
}

auto ModelImpl::CreateProjectionCircle(const common::Vector3D& rot_center,
                                       const common::Vector3D& weld_object_rotation_axis,
                                       Point3d& point_to_project) const -> Circle3d {
  // Projection/rotation entities
  Vector3d calib_rot_center_1 = CommonVector2EigenVector(rot_center);
  Vector3d n_circle_1         = CommonVector2EigenVector(weld_object_rotation_axis).normalized();
  Vector3d p_macs             = point_to_project.ToVec();

  // Adjusted projection circle center
  Vector3d from_center_to_point = p_macs - calib_rot_center_1;
  double shift_along_rotax      = from_center_to_point.dot(n_circle_1);
  Vector3d p_circle_1           = calib_rot_center_1 + shift_along_rotax * n_circle_1;
  Point3d circle_center{p_circle_1(0), p_circle_1(1), p_circle_1(2), CoordinateSystem::MACS};

  // Rotate/project to torch plane by computing intersection between ABW circle and torch plane
  double dist_to_center = (p_macs - p_circle_1).norm();  // Projection circle radius
  Circle3d projection_circle{n_circle_1, dist_to_center, circle_center};

LOG_DEBUG("projection circle: r={:.6f}, center=({:.6f}, {:.6f}, {:.6f}), n=({:.6f}, {:.6f}, {:.6f}), ref={}",
          dist_to_center,
          circle_center.GetX(), circle_center.GetY(), circle_center.GetZ(),
          n_circle_1(0), n_circle_1(1), n_circle_1(2),
          static_cast<int>(circle_center.GetRefSystem()));

  return projection_circle;
}

auto ModelImpl::RotateToPlane(const Circle3d& projection_circle, Plane3d& target_plane) const -> Point3d {
  std::vector<Point3d> intersection_points = projection_circle.Intersect(target_plane);
  // Filter out any invalid points (NaNs/Infs) defensively
  std::vector<Point3d> valid_points;
  valid_points.reserve(intersection_points.size());
  for (std::size_t i = 0; i < intersection_points.size(); ++i) {
    const auto& pt = intersection_points[i];
    const double x = pt.GetX();
    const double y = pt.GetY();
    const double z = pt.GetZ();
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
      valid_points.push_back(pt);
      LOG_DEBUG("intersection_points[{}]: ({:.6f}, {:.6f}, {:.6f}), ref={}", i, x, y, z,
                static_cast<int>(pt.GetRefSystem()));
    } else {
      LOG_ERROR("Dropping invalid intersection point[{}]: ({}, {}, {}), ref={}", i, x, y, z,
                static_cast<int>(pt.GetRefSystem()));
    }
  }
  if (valid_points.empty()) {
    // Fallback: return orthogonal projection of origin onto target plane
    Eigen::Vector3d n = target_plane.GetNormal();
    if (n.norm() < 1e-12 || !std::isfinite(n.norm())) {
      n = {0.0, 1.0, 0.0};
    } else {
      n.normalize();
    }
    Eigen::Vector3d p0   = target_plane.GetPointInPlane().ToVec();
    Eigen::Vector3d proj = p0 - n.dot(p0) * n;
    LOG_ERROR("No valid circle-plane intersection; using plane origin projection fallback");
    return {proj(0), proj(1), proj(2), target_plane.GetRefSystem()};
  }
  Point3d int_point = FindClosestPoint(valid_points, {0, 0, 0, CoordinateSystem::MACS});
  return int_point;
}

auto ModelImpl::TransformMACStoLPCS(std::array<double, 3> scanner_angles,
                                    const common::Vector3D& torch_to_laser_translation, macs::Point slide_position,
                                    Point3d point_macs, bool use_translation) const -> Point3d {
  // Orientation matrices
  Matrix3d R_1to4 = ComputeLpcsOrientation(scanner_angles.at(0), scanner_angles.at(1), scanner_angles.at(2));

  // Translation vectors
  Vector3d t_3to4 = CommonVector2EigenVector(torch_to_laser_translation);
  Vector3d t_1to3 = {slide_position.horizontal, 0.0, slide_position.vertical};
  Vector3d t_1to4 = t_1to3 + t_3to4;

  // Transform
  Vector3d vec_macs = point_macs.ToVec();
  Vector3d transformed_point;  //= R_1to4.transpose() * (vec_macs - t_1to4);

  transformed_point =
      use_translation ? (R_1to4.transpose() * (vec_macs - t_1to4)).eval() : (R_1to4.transpose() * vec_macs).eval();

  return {transformed_point(0), transformed_point(1), transformed_point(2), LPCS};
}

auto ModelImpl::TransformLPCStoMACS(std::array<double, 3> scanner_angles,
                                    const common::Vector3D& torch_to_laser_translation, macs::Point slide_position,
                                    Point3d point_lpcs, bool use_translation) const -> Point3d {
  // Orientation matrices
  Matrix3d R_1to4 = ComputeLpcsOrientation(scanner_angles.at(0), scanner_angles.at(1), scanner_angles.at(2));

  // Translation vectors
  Vector3d t_3to4 = CommonVector2EigenVector(torch_to_laser_translation);
  Vector3d t_1to3 = {slide_position.horizontal, 0.0, slide_position.vertical};
  Vector3d t_1to4 = t_1to3 + t_3to4;

  // Transform
  Vector3d vec_lpcs          = point_lpcs.ToVec();
  Vector3d transformed_point = use_translation ? (R_1to4 * vec_lpcs + t_1to4).eval() : (R_1to4 * vec_lpcs).eval();

  return {transformed_point(0), transformed_point(1), transformed_point(2), MACS};
}

auto ModelImpl::FindClosestPoint(std::vector<Point3d> points, Point3d ref_point) const -> Point3d {
  int min_idx      = -1;
  double curr_dist = NAN;
  double min_dist  = INFINITY;
  int count        = 0;

  for (const auto& p : points) {
    curr_dist = p.DistanceTo(ref_point);

    if (curr_dist < min_dist) {
      min_idx  = count;
      min_dist = curr_dist;
    }

    count++;
  }

  if (min_idx == -1) {
    throw std::runtime_error("No point found");
  }

  return points.at(min_idx);
}

auto ModelImpl::AngleFromTorchToScanner(const std::vector<lpcs::Point>& lpcs_points, const macs::Point& axis_position)
    -> std::optional<double> {
  // Here we calculate the angle between the laser plane point and its projection onto torch plane

  std::array<double, 3> scanner_angles = {scanner_mount_angle_, delta_rot_y_, delta_rot_z_};

  // Convert the laser point to MACS
  Point3d laserplane_point_lpcs{lpcs_points.at(0).x, lpcs_points.at(1).y, 0.0, LPCS};
  Point3d laserplane_point_macs =
      TransformLPCStoMACS(scanner_angles, torch_to_laser_translation_, axis_position, laserplane_point_lpcs);

  // Project the laser point onto torch plane
  macs::Point tmp = TransformAndRotateToTorchPlane(rot_center_, scanner_angles, weld_object_rotation_axis_,
                                                   torch_to_laser_translation_, lpcs_points.at(0), axis_position);

  // Create vectors of torch plane and laser plane points and determine angle with dot product
  Vector3d vec_rotcenter_macs  = {rot_center_.c1, rot_center_.c2, rot_center_.c3};
  Vector3d vec_torchplane_macs = {tmp.horizontal, 0.0, tmp.vertical};
  Vector3d vec_laserplane_macs = laserplane_point_macs.ToVec();
  Vector3d vec_torchplane_rocs = vec_torchplane_macs - vec_rotcenter_macs;
  Vector3d vec_laserplane_rocs = vec_laserplane_macs - vec_rotcenter_macs;
  double angle_between_vectors = std::acos(vec_torchplane_rocs.normalized().dot(vec_laserplane_rocs.normalized()));

  return angle_between_vectors;
};

auto ModelImpl::Available() const -> bool { return available_; }

void ModelImpl::Set(const common::Vector3D& rot_center, std::array<double, 3> scanner_angles,
                    const common::Vector3D& weld_object_rotation_axis,
                    const common::Vector3D& torch_to_laser_translation) {
  rot_center_                 = rot_center;
  weld_object_rotation_axis_  = weld_object_rotation_axis;
  torch_to_laser_translation_ = torch_to_laser_translation;
  scanner_mount_angle_        = scanner_angles.at(0);
  delta_rot_y_                = scanner_angles.at(1);
  delta_rot_z_                = scanner_angles.at(2);
  available_                  = true;
  LOG_INFO("Model parameters set");
}

void ModelImpl::Reset() {
  rot_center_                 = {};
  weld_object_rotation_axis_  = {};
  torch_to_laser_translation_ = {};
  scanner_mount_angle_        = {};
  available_                  = false;
  LOG_INFO("Model parameters reset ");
}

auto ModelImpl::ComputeLpcsOrientation(double scanner_mount_angle, double delta_rot_y, double delta_rot_z) const
    -> Eigen::Matrix3d {
  Vector3d rot_axis(0.0, 0.0, 1.0);
  const Matrix3d rot_z  = CreateRotationMatrix(std::numbers::pi, rot_axis);
  rot_axis              = {1.0, 0.0, 0.0};
  const Matrix3d rot_x  = CreateRotationMatrix((3 * std::numbers::pi / 2) - scanner_mount_angle, rot_axis);
  rot_axis              = {0.0, 1.0, 0.0};
  const Matrix3d rot_ey = CreateRotationMatrix(delta_rot_y, rot_axis);
  rot_axis              = {0.0, 0.0, 1.0};
  const Matrix3d rot_ez = CreateRotationMatrix(delta_rot_z, rot_axis);

  return rot_ez * (rot_x * rot_z) * rot_ey;
}

auto ModelImpl::ComputeWeldObjectOrientation(const common::Vector3D& weld_object_rotation_axis) const
    -> Eigen::Matrix3d {
  // Compute weld object orientation
  Vector3d x_axis_11 = {1, 0, 0};  // This is object rotation axis in CS 11.
  Vector3d x_axis_10 = {weld_object_rotation_axis.c1, weld_object_rotation_axis.c2, weld_object_rotation_axis.c3};
  Vector3d axis      = x_axis_10.cross(x_axis_11);  // Rotation axis for transformation between CS 10 and 11
  axis.normalize();
  double angle = std::acos(x_axis_10.dot(x_axis_11));
  Matrix3d R_10to11;

  if (std::fabs(angle) < 1e-5) {
    R_10to11 = Matrix3d::Identity();
  } else {
    R_10to11 = Eigen::AngleAxisd{angle, axis}.toRotationMatrix();
  }

  return R_10to11;
}

auto ModelImpl::CreateRotationMatrix(double angle, Eigen::Vector3d& rot_axis) const -> Eigen::Matrix3d {
  rot_axis.normalize();
  Eigen::Matrix3d transform;
  transform = Eigen::Matrix3d::Zero();

  double factor = 0.0;
  if (!std::isnan(angle)) {
    factor = 1.0 - std::cos(angle);

    // This can be initilized much easier in Eigen. Doing this way to ensure that definitions are exactly the same as
    // in Scilab simulator. This definition of the transformation is equivalent to T = Trans4D * Rotation4D.
    transform(0, 0) = factor * std::pow(rot_axis(0), 2) + std::cos(angle);
    transform(0, 1) = factor * rot_axis(0) * rot_axis(1) - rot_axis(2) * std::sin(angle);
    transform(0, 2) = factor * rot_axis(0) * rot_axis(2) + rot_axis(1) * std::sin(angle);
    transform(1, 0) = factor * rot_axis(0) * rot_axis(1) + rot_axis(2) * std::sin(angle);
    transform(1, 1) = factor * std::pow(rot_axis(1), 2) + std::cos(angle);
    transform(1, 2) = factor * rot_axis(1) * rot_axis(2) - rot_axis(0) * std::sin(angle);
    transform(2, 0) = factor * rot_axis(2) * rot_axis(0) - rot_axis(1) * std::sin(angle);
    transform(2, 1) = factor * rot_axis(2) * rot_axis(1) + rot_axis(0) * std::sin(angle);
    transform(2, 2) = factor * std::pow(rot_axis(2), 2) + std::cos(angle);
  } else {
    transform = Eigen::Matrix3d::Identity();
  }

  return transform;
}

}  // Namespace slice_translator
