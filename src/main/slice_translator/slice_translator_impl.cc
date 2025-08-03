
#include "slice_translator/slice_translator_impl.h"

#include <cmath>
#include <cstddef>
#include <Eigen/Core>
#include <optional>
#include <string>
#include <system_error>
#include <vector>

#include "calibration/calibration_types.h"
#include "common/logging/application_log.h"
#include "lpcs/lpcs_point.h"
#include "lpcs/lpcs_slice.h"
#include "macs/macs_point.h"
#include "slice_translator/slice_translator.h"

using Eigen::MatrixXd;
using Eigen::RowVectorXd;

using slice_translator::SliceTranslatorErrorCode;
using slice_translator::SliceTranslatorImpl;

namespace outcome = BOOST_OUTCOME_V2_NAMESPACE;

auto SliceTranslatorImpl::VectorToMatrix(const std::vector<lpcs::Point>& coord_vec) -> Eigen::MatrixXd {
  size_t num_rows = coord_vec.size();
  Eigen::MatrixXd slice(num_rows, 2);  // n x 2 matrix

  for (size_t i = 0; i < num_rows; ++i) {
    const auto& point = coord_vec[i];
    slice(i, 0)       = point.x;
    slice(i, 1)       = point.y;
  }

  return slice;
}

auto SliceTranslatorImpl::MatrixToVector(Eigen::MatrixXd matrix) -> std::vector<macs::Point> {
  std::vector<macs::Point> coord_vec;

  // Iterate over each row of the matrix
  for (int i = 0; i < matrix.rows(); ++i) {
    macs::Point point{matrix(i, 0), matrix(i, 2)};
    coord_vec.push_back(point);
  }

  return coord_vec;
}

auto SliceTranslatorImpl::GetRotMatrix(double angle) -> Eigen::MatrixXd {
  Eigen::MatrixXd rot_matrix(2, 3);
  rot_matrix << -1, 0, 0, 0, sin(angle), cos(angle);
  return rot_matrix;
}

auto SliceTranslatorImpl::CalibrateLaserToTorch(double angle, double offset, double stickout,
                                                const lpcs::Slice& scanner_data)
    -> std::optional<calibration::LaserTorchCalibration> {
  RowVectorXd expected_tcp_point(3);
  calibration::LaserTorchCalibration calibration{};

  if (!scanner_data.groove) {
    return {};
  }
  auto groove_matrix = VectorToMatrix(scanner_data.groove.value());

  auto left_corner  = groove_matrix.row(0);
  auto right_corner = groove_matrix.row(groove_matrix.rows() - 1);

  // Maybe assert that the y values are similar for both slices (i.e the calibration piece is correctly placed).
  expected_tcp_point << 0, offset, -stickout;
  RowVectorXd corner_mid_point = (left_corner + right_corner) / 2;
  auto rot_matrix              = GetRotMatrix(angle);

  Eigen::RowVectorXd result = expected_tcp_point - corner_mid_point * rot_matrix;
  calibration.angle         = angle;
  calibration.x             = result(0);
  calibration.y             = result(1);
  calibration.z             = result(2);

  return calibration;
}

auto SliceTranslatorImpl::CalibrateWeldObject(double radius, double stickout, const lpcs::Slice& scanner_data,
                                              const macs::Point& axis_position)
    -> std::optional<calibration::WeldObjectCalibration> {
  if (!scanner_data.groove) {
    return {};
  }
  auto groove_matrix = VectorToMatrix(scanner_data.groove.value());

  auto maybe_tcp_slice = FromLaserPlane(groove_matrix, axis_position);

  if (!maybe_tcp_slice) {
    LOG_ERROR("No tcp slice");
    return {};
  }
  auto tcp_slice = maybe_tcp_slice.value();

  // Set the offset so the macs will be 0 at the current position.
  auto x_adjustment = axis_position.horizontal - tcp_slice(3, 0);

  // Subtract the wall height from the radius to account for being in the bottom of the groove.
  auto left_wall_height    = tcp_slice(0, 2) - tcp_slice(1, 2);
  auto right_wall_height   = tcp_slice(6, 2) - tcp_slice(5, 2);
  auto average_wall_height = (left_wall_height + right_wall_height) / 2.0;
  auto inner_radius        = radius - average_wall_height;

  // Calculate the center of a circle given that the wire and the laser are on the circumerense.
  Eigen::Vector2d laser_point(tcp_slice(3, 1), tcp_slice(3, 2) - axis_position.vertical);
  Eigen::Vector2d wire_tip_point(0, -stickout);

  LOG_DEBUG("Laser point coordinates: y:{}, z:{}", laser_point.x(), laser_point.y());
  LOG_DEBUG("Wire point coordinates: y:{}, z:{}", wire_tip_point.x(), wire_tip_point.y());

  Eigen::Vector2d chord      = laser_point - wire_tip_point;
  Eigen::Vector2d mid_point  = (laser_point + wire_tip_point) / 2.0;
  double dist_between_points = chord.norm();

  if (chord.x() == 0.0) {
    LOG_ERROR("Laser and torch is at the same y position during calibration.");
    return {};
  }

  // Edge case: impossible circle
  if (dist_between_points > 2 * inner_radius) {
    LOG_ERROR("No circle possible: points are too far apart for the given radius.");
    return {};
  }

  Eigen::Vector2d circle_center;
  LOG_DEBUG("inner radius: {}", inner_radius);
  LOG_DEBUG("Distance between points: {}", dist_between_points);
  auto dist_to_center = std::sqrt((inner_radius * inner_radius) - ((dist_between_points * dist_between_points) / 4.0));
  Eigen::Vector2d perp(chord.y(), -chord.x());
  LOG_DEBUG("Distance to center: {}", dist_to_center);
  perp.normalize();
  LOG_DEBUG("Perp normilized: y: {}, z: {}", perp.x(), perp.y());
  circle_center = mid_point + dist_to_center * perp;

  if (circle_center.x() > 0.0) {
    LOG_WARNING("Center of object is behind torch, setting weld object calibration to 12 o'clock.");
    circle_center = Eigen::Vector2d(0.0, -inner_radius) + wire_tip_point;
  }

  LOG_DEBUG("circle_center: y:{}, z:{}", circle_center.x(), circle_center.y());

  calibration::WeldObjectCalibration calibration(circle_center.x(), circle_center.y() + axis_position.vertical,
                                                 x_adjustment);
  if (std::isnan(calibration.y) || std::isnan(calibration.z)) {
    LOG_ERROR("Compute error");
    return {};
  }
  LOG_DEBUG("Calibration: x_adjustment:{}, y: {}, z:{}", calibration.x_adjustment, calibration.y, calibration.z);

  return calibration;
}

auto SliceTranslatorImpl::AngleFromTorchToScanner(const std::vector<lpcs::Point>& lpcs_points,
                                                  const macs::Point& axis_position) -> outcome::result<double> {
  if (!opt_weld_object_calibration_.has_value()) {
    return outcome::failure(MakeErrorCode(SliceTranslatorErrorCode::MISSING_CALIBRATION));
  }
  auto rotation_center = opt_weld_object_calibration_.value();

  // Where, as seen from the side, is the torch and the lpcs point?
  // Torch is at (0, axis_position.vertical)
  // Rotation center is at (ry, rz + axis_position.vertical) ... sign?
  // Where, in relation to the torch, are the lpcs points?
  auto lpcs_matrix      = VectorToMatrix(lpcs_points);
  auto maybe_tcp_matrix = FromLaserPlane(lpcs_matrix, axis_position);
  if (!maybe_tcp_matrix) {
    return maybe_tcp_matrix.error();
  }
  auto tcp_matrix = maybe_tcp_matrix.value();

  // Get back weld object radius
  const double weld_object_radius = std::sqrt(std::pow(rotation_center.y, 2) + std::pow(rotation_center.z, 2));

  // Get distance from torch to first lpcs point
  const double dy       = tcp_matrix(0, 1);
  const double dz       = tcp_matrix(0, 2) - axis_position.vertical;
  const double distance = std::sqrt(dy * dy + dz * dz);

  // Get angle of the circle segment
  const double theta = 2.0 * std::asin(distance / (2.0 * weld_object_radius));

  return theta;
}

auto SliceTranslatorImpl::LPCSToMCS(const std::vector<lpcs::Point>& lpcs_points, const macs::Point& axis_position)
    -> outcome::result<std::vector<macs::Point>> {
  auto lpcs_matrix      = VectorToMatrix(lpcs_points);
  auto maybe_tcp_matrix = FromLaserPlane(lpcs_matrix, axis_position);
  if (!maybe_tcp_matrix) {
    return maybe_tcp_matrix.error();
  }

  auto maybe_machine_matrix = RotateToTool(maybe_tcp_matrix.value());
  if (!maybe_machine_matrix) {
    return maybe_machine_matrix.error();
  }
  return MatrixToVector(maybe_machine_matrix.value());
}

auto SliceTranslatorImpl::FromLaserPlane(const MatrixXd& slice, const macs::Point& axis_position)
    -> outcome::result<MatrixXd> {
  if (!opt_laser_torch_calibration_.has_value()) {
    return outcome::failure(MakeErrorCode(SliceTranslatorErrorCode::FAILED_TO_GET_POSITION));
  }
  auto calibration = opt_laser_torch_calibration_.value();
  RowVectorXd offset_vector(3);

  offset_vector << calibration.x, calibration.y, calibration.z;

  auto rot_matrix         = GetRotMatrix(calibration.angle);
  auto pos_relative_torch = (slice * rot_matrix).rowwise() + offset_vector;

  auto pos_torch = axis_position;

  Eigen::Vector3d torch_vector(pos_torch.horizontal, 0.0, pos_torch.vertical);
  return pos_relative_torch.rowwise() + torch_vector.transpose();
}

auto SliceTranslatorImpl::RotateToTool(const MatrixXd& slice) -> outcome::result<MatrixXd> {
  if (!opt_weld_object_calibration_.has_value()) {
    return outcome::failure(MakeErrorCode(SliceTranslatorErrorCode::MISSING_CALIBRATION));
  }
  auto rotation_center = opt_weld_object_calibration_.value();

  int num_point = slice.rows();
  Eigen::MatrixXd transformed(num_point, 3);

  for (size_t i = 0; i < num_point; ++i) {
    double px = slice(i, 0);
    double py = slice(i, 1);
    double pz = slice(i, 2);

    double radius     = std::sqrt(std::pow(py - rotation_center.y, 2) + std::pow(pz - rotation_center.z, 2));
    transformed(i, 0) = px + rotation_center.x_adjustment;
    transformed(i, 1) = 0;
    transformed(i, 2) = rotation_center.z + std::sqrt(std::pow(radius, 2) - std::pow(rotation_center.y, 2));
  }

  return transformed;
}

auto SliceTranslatorImpl::GetLaserToTorchCalibration() const -> std::optional<calibration::LaserTorchCalibration> {
  return opt_laser_torch_calibration_;
}

auto SliceTranslatorImpl::SetLaserToTorchCalibration(const calibration::LaserTorchCalibration& data)
    -> boost::outcome_v2::result<void> {
  boost::outcome_v2::result<void> result = boost::outcome_v2::success();
  if (data.store_persistent) {
    result = laser_torch_config_handle_->WritePersistentData(data);
  }

  if (result == boost::outcome_v2::success()) {
    opt_laser_torch_calibration_ = data;
  }

  return result;
}

auto SliceTranslatorImpl::GetWeldObjectCalibration() const -> std::optional<calibration::WeldObjectCalibration> {
  return opt_weld_object_calibration_;
}

auto SliceTranslatorImpl::SetWeldObjectCalibration(const calibration::WeldObjectCalibration& data)
    -> boost::outcome_v2::result<void> {
  boost::outcome_v2::result<void> result = boost::outcome_v2::success();
  if (data.store_persistent) {
    result = weld_object_config_handle_->WritePersistentData(data);
  }

  if (result == boost::outcome_v2::success()) {
    opt_weld_object_calibration_ = data;
  }

  return result;
}

// Error code implementation
namespace {

struct ErrorCategory : std::error_category {
  auto name() const noexcept -> const char* final;          // NOLINT(*-use-nodiscard)
  auto message(int error_code) const -> std::string final;  // NOLINT(*-use-nodiscard)
  auto default_error_condition(int other) const noexcept    // NOLINT(*-use-nodiscard)
      -> std::error_condition final;                        // NOLINT(*-use-nodiscard)
};

auto ErrorCategory::name() const noexcept -> const char* { return "SliceTranslatorError"; }

auto ErrorCategory::message(int error_code) const -> std::string {
  switch (static_cast<SliceTranslatorErrorCode>(error_code)) {
    case SliceTranslatorErrorCode::NO_ERROR:
      return "No error";
    case SliceTranslatorErrorCode::FAILED_TO_GET_POSITION:
      return "Failed to get position";
    case SliceTranslatorErrorCode::INCORRECT_INPUT_SLICE:
      return "Incorrect input slice";
    case SliceTranslatorErrorCode::MISSING_CALIBRATION:
      return "Missing calibration";
    case SliceTranslatorErrorCode::COMPUTE_ERROR:
      return "Compute error";
  }
}

auto ErrorCategory::default_error_condition(int other) const noexcept -> std::error_condition {
  switch (static_cast<SliceTranslatorErrorCode>(other)) {
    default:
      return {other, *this};
  }
}

const ErrorCategory ERROR_CATEGORY{};

}  // namespace

[[maybe_unused]] auto slice_translator::MakeErrorCode(SliceTranslatorErrorCode error_code)
    -> std::error_code {  // NOLINT(*-identifier-naming)
  return {static_cast<int>(error_code), ERROR_CATEGORY};
}
/*

// NOLINTBEGIN(*-magic-numbers)

#include <doctest/doctest.h>

#include <trompeloeil.hpp>

#include "components/kinematics/direct_kinematics.h"

using components::kinematics::Kinematics;
using components::kinematics::Position;
using doctest::Approx;
using std::numbers::pi;
using trompeloeil::mock_interface;

class KinematicsMock : public mock_interface<Kinematics> {
  IMPLEMENT_MOCK0(Update);
  IMPLEMENT_MOCK1(PushBack);
  IMPLEMENT_MOCK0(ClearQueue);
  IMPLEMENT_MOCK0(Start);
  IMPLEMENT_MOCK0(Stop);
  IMPLEMENT_MOCK0(Halt);
  IMPLEMENT_MOCK0(Continue);
  IMPLEMENT_MOCK0(Busy);
  IMPLEMENT_MOCK0(InPosition);
  IMPLEMENT_MOCK0(IsEnabled);
  IMPLEMENT_MOCK0(CurrentPosition);
  IMPLEMENT_MOCK1(SetFollowPosition);
  IMPLEMENT_MOCK0(IsFollowingPosition);
  IMPLEMENT_MOCK1(UpdateHorizontalSlideAxis);
  IMPLEMENT_MOCK1(UpdateVerticalSlideAxis);
};

JointProfile ProfileFromMatrix(MatrixXd mat) {
  JointProfile profile;
  std::vector<common::scanner::LineSegment> segments;
  std::vector<std::tuple<double, double, double>> points;

  int n = mat.rows();

  for (size_t i = 0; i < n; ++i) {
    auto px = mat(i, 0);
    auto py = mat(i, 1);

    points.push_back(std::tuple(px, py, 0));
  }
  profile.segments = segments;
  profile.points   = points;
  return profile;
}

TEST_SUITE("Coordinate Translator") {
  TEST_CASE("No angle or offset") {
    double angle    = 0;
    auto kinematics = std::make_shared<KinematicsMock>();
    REQUIRE_CALL(*kinematics, CurrentPosition()).RETURN(Position(0.0, 0.0));
    LaserTorchCalibration LaserTorchCalibration(0, 0, 0, angle);
    SliceTranslatorImpl translator(kinematics, LaserTorchCalibration);

    MatrixXd slice(7, 2);
    slice << 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0;
    auto profile = ProfileFromMatrix(slice);

    auto transformedSlice = translator.LaserToTCP(scanner_data).value();
    RowVectorXd expected(3);

    expected << 0, 0, 0;
    CHECK(transformedSlice.row(0).isApprox(expected));

    expected << -1, 0, 1;
    CHECK(transformedSlice.row(1).isApprox(expected));

    expected << -1, 0, 0;
    CHECK(transformedSlice.row(2).isApprox(expected));

    expected << 0, 0, 1;
    CHECK(transformedSlice.row(3).isApprox(expected));
  }
  TEST_CASE("Angle") {
    double angle    = pi / 6;
    auto kinematics = std::make_shared<KinematicsMock>();
    REQUIRE_CALL(*kinematics, CurrentPosition()).RETURN(Position(0.0, 0.0));
    LaserTorchCalibration LaserTorchCalibration(0, 0, 0, angle);
    SliceTranslatorImpl translator(kinematics, LaserTorchCalibration);
    MatrixXd slice(7, 2);

    slice << 0, 0, 1, -1, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0;
    auto profile          = ProfileFromMatrix(slice);
    auto transformedSlice = translator.LaserToTCP(scanner_data).value();
    RowVectorXd expected(3);

    expected << 0, 0, 0;
    CHECK(transformedSlice.row(0).isApprox(expected));

    expected << -1, -0.5, -sqrt(3) / 2;
    CHECK(transformedSlice.row(1).isApprox(expected));

    expected << -1, 0, 0;
    CHECK(transformedSlice.row(2).isApprox(expected));

    expected << 0, -0.5, -sqrt(3) / 2;
    CHECK(transformedSlice.row(3).isApprox(expected));
  }

  TEST_CASE("Offset") {
    double angle    = pi / 4;
    auto kinematics = std::make_shared<KinematicsMock>();
    REQUIRE_CALL(*kinematics, CurrentPosition()).RETURN(Position(0.0, 0.0));
    LaserTorchCalibration LaserTorchCalibration(1, 2, 3, angle);
    SliceTranslatorImpl translator(kinematics, LaserTorchCalibration);

    MatrixXd slice(7, 2);

    slice << 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0;
    auto profile          = ProfileFromMatrix(slice);
    auto transformedSlice = translator.LaserToTCP(scanner_data).value();
    RowVectorXd expected(3);

    expected << 1, 2, 3;
    CHECK(transformedSlice.row(0).isApprox(expected));

    expected << 0, 2 + 1 / sqrt(2), 3 + 1 / sqrt(2);
    CHECK(transformedSlice.row(1).isApprox(expected));

    expected << 0, 2, 3;
    CHECK(transformedSlice.row(2).isApprox(expected));

    expected << 1, 2 + 1 / sqrt(2), 3 + 1 / sqrt(2);
    CHECK(transformedSlice.row(3).isApprox(expected));
  }
  TEST_CASE("Slide Offset") {
    double angle    = 0.723;
    auto kinematics = std::make_shared<KinematicsMock>();
    REQUIRE_CALL(*kinematics, CurrentPosition()).RETURN(Position(0.0, 1.0));
    LaserTorchCalibration LaserTorchCalibration(0, 2, 1, angle);
    RotationCenter rotationCenter(-2.43, -3.02);
    SliceTranslatorImpl translator(kinematics, LaserTorchCalibration, true, rotationCenter);

    MatrixXd slice(7, 2);

    slice << 0, -2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    auto profile          = ProfileFromMatrix(slice);
    auto transformedSlice = translator.LaserToTCP(scanner_data).value();
    RowVectorXd expected(3);
    expected << 0, 0, 1;
    CHECK(transformedSlice.row(0).isApprox(expected, 0.1));

  }
   TEST_CASE("Test 2") {
     double angle = pi / 4;
     auto kinematics = std::make_shared<KinematicsMock>();
     REQUIRE_CALL(*kinematics, CurrentPosition()).RETURN(Position(0.0, 0.0));
     LaserTorchCalibration LaserTorchCalibration(0, 1.64, 0.78);
     RotationCenter rotationCenter(-0.5, -3.5);
     SliceTranslator translator(angle, kinematics, LaserTorchCalibration, true, rotationCenter);

     MatrixXd slice(2, 2);

     slice << 0, -1.86, 1, -1.45;

     auto transformedSlice = translator.LaserToTCP(slice);
     RowVectorXd expected(3);
     expected << 0, 0, -0.47;
     CHECK(transformedSlice.row(0).isApprox(expected, 0.1));

     expected << -1, 0, -0.1;
     CHECK(transformedSlice.row(1).isApprox(expected, 0.1));
   }
   TEST_CASE("Calibrate LaserOrigin") {
     double angle = pi / 6;
     double stickout = 2.0;
     double distance = 5.0;

     auto kinematics = std::make_shared<KinematicsMock>();
     REQUIRE_CALL(*kinematics, CurrentPosition()).RETURN(Position(0.0, 0.0));

     MatrixXd slice(7, 2);
     RowVectorXd expected(3);
     SliceTranslator translator(angle, kinematics);
     slice << -1, -3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -3;
     translator.CalibrateLaserOrigin(distance, stickout, slice);
     auto transformedSlice = translator.LaserToTCP(slice);

     expected << 0, 6.5, 0.6;
     CHECK(transformedSlice.row(1).isApprox(expected, 0.01));

     expected << 1, 5, -2;
     CHECK(transformedSlice.row(0).isApprox(expected, 0.01));
   }

   TEST_CASE("Calibrate Workpiece Center") {
     double angle = 0.621;
     auto kinematics = std::make_shared<KinematicsMock>();
     REQUIRE_CALL(*kinematics, CurrentPosition()).RETURN(Position(0.0, 0.0)).TIMES(3);

     LaserTorchCalibration LaserTorchCalibration(0, 2, 1);
     SliceTranslator translator(angle, kinematics, LaserTorchCalibration , true);

     double radius = 4.35;
     RowVectorXd wobjCenter(2) << -1.2, -1.6;
     MatrixXd slice(1, 2);
     RowVectorXd expected(3);
     slice << 0, -1.85;


     translator.CalibrateRotationCenter(slice, radius);
     auto transformedSlice = translator.LaserToTCP(slice);
     expected << 0, 0, 0;
     CHECK(transformedSlice.row(0).isApprox(expected));
     auto rotCenter = translator.GetRotationCenter().value();
     CHECK(rotCenter.y == Approx(-1.61).epsilon(0.001));
     CHECK(rotCenter.z == Approx(-4.04).epsilon(0.001));
   }
   TEST_CASE("Calibrate Workpiece Center With Tool Offset") {
     double angle = 0.723;
     auto kinematics = std::make_shared<KinematicsMock>();
     REQUIRE_CALL(*kinematics, CurrentPosition()).RETURN(Position(0.0, 1.0)).TIMES(3);

     LaserTorchCalibration LaserTorchCalibration(0, 2, 1);
     SliceTranslator translator(angle, kinematics, LaserTorchCalibration , true);

     double radius = 4.7;
     MatrixXd slice(1, 2);
     RowVectorXd expected(3);
     slice << 0, -2;


     translator.CalibrateRotationCenter(slice, radius);
     auto transformedSlice = translator.LaserToTCP(slice);
      expected << 0, 0, 1;
     CHECK(transformedSlice.row(0).isApprox(expected));
     auto rotCenter = translator.GetRotationCenter().value();
     CHECK(rotCenter.y == Approx(-2.43).epsilon(0.01));
     CHECK(rotCenter.z == Approx(-3.02).epsilon(0.01));
   }



}

// NOLINTEND(*-magic-numbers)
#endif*/
