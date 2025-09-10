#include "scanner/joint_model/big_snake.h"

#include <math.h>

#include <algorithm>
#include <boost/math/statistics/linear_regression.hpp>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <Eigen/Eigen>
#include <expected>
#include <optional>
#include <tuple>

#include "scanner/image/camera_model.h"
#include "scanner/image/image.h"
#include "scanner/image/image_types.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/joint_model/slice.h"
#include "scanner/joint_model/snake.h"

namespace scanner::joint_model {
// #define VISUAL_DEBUG_OUTPUT 1

const double MASK_OFFSET        = 0.002;
const int NUM_TRIANGLES_TO_MASK = 4;
const int START_SNAKE_OFFSET    = 200;

/*
 ____  _       ____              _
| __ )(_) __ _/ ___| _ __   __ _| | _____
|  _ \| |/ _` \___ \| '_ \ / _` | |/ / _ \
| |_) | | (_| |___) | | | | (_| |   <  __/
|____/|_|\__, |____/|_| |_|\__,_|_|\_\___|
         |___/
*/
auto BigSnake::Parse(image::Image& image, std::optional<JointProfile> median_profile,
                     std::optional<JointProperties> updated_properties, bool use_approximation,
                     std::optional<std::tuple<double, double>> abw0_abw6_horizontal)
    -> std::expected<std::tuple<JointProfile, image::WorkspaceCoordinates, uint64_t, uint64_t>, JointModelErrorCode> {
  auto start = std::clock();

  auto current_properties = updated_properties.has_value() ? updated_properties.value() : properties_;

  CropImageHorizontal(image, median_profile);

  auto mask = GenerateMask(image, median_profile);

  auto maybe_snake = Snake::FromImage(image, mask, threshold_);
  if (!maybe_snake) {
    // Fallback 1: auto-crop horizontally around bright columns and retry
    const auto& data = image.Data();
    const int cols   = static_cast<int>(data.cols());
    const int rows   = static_cast<int>(data.rows());
    const int fallback_threshold = std::min(std::max(2 * static_cast<int>(threshold_), 32), 255);

    int first_bright = -1;
    int last_bright  = -1;
    for (int c = 0; c < cols; c++) {
      // Efficiently compute column max
      uint8_t max_val = 0;
      for (int r = 0; r < rows; r++) {
        const uint8_t v = data(r, c);
        if (v > max_val) {
          max_val = v;
          if (max_val > fallback_threshold) {
            break;
          }
        }
      }
      if (max_val > fallback_threshold) {
        if (first_bright < 0) {
          first_bright = c;
        }
        last_bright = c;
      }
    }

    if (first_bright >= 0 && last_bright >= 0 && last_bright - first_bright > 20) {
      const int margin = 100;
      const int start_col = std::max(0, first_bright - margin);
      const int stop_col  = std::min(cols, last_bright + margin);
      image.SetHorizontalCrop(start_col, stop_col);
      maybe_snake = Snake::FromImage(image, mask, threshold_);
    }

    if (!maybe_snake) {
      return std::unexpected(JointModelErrorCode::SURFACE_NOT_FOUND);
    }
  }
  const auto& snake = maybe_snake.value();

  // Snake from image to LPCS
  auto maybe_snake_lpcs = snake.ToLPCS(camera_model_.get(), image.GetVerticalCropStart());
  if (!maybe_snake_lpcs) {
    return std::unexpected(JointModelErrorCode::SURFACE_NOT_FOUND);
  }
  auto [snake_lpcs, min, max] = maybe_snake_lpcs.value();

  // Snake to slice
  auto maybe_slice = Slice::FromSnake(snake_lpcs, current_properties, median_profile, found_out_of_spec_joint_width_,
                                      updated_properties.has_value(), use_approximation, abw0_abw6_horizontal);

  if (!maybe_slice) {
    // Fallback 2: if walls were not found (or similar), retry enabling the
    // "updated properties" path to activate the joint standard deviation fallback.
    auto err_code = maybe_slice.error();
    (void)err_code;  // silence unused in release

    auto retry_slice = Slice::FromSnake(snake_lpcs, current_properties, median_profile, found_out_of_spec_joint_width_,
                                        true /*joint_properties_updated*/, use_approximation, abw0_abw6_horizontal);
    if (!retry_slice) {
      return std::unexpected(maybe_slice.error());
    }
    maybe_slice = retry_slice;
  }

  auto [points, num_walls, approximation_used] = maybe_slice.value();

  JointProfile profile;
  profile.points             = points;
  profile.area               = BigSnake::CalculateJointArea(profile.points);
  profile.approximation_used = approximation_used;

  // Vertical limits
  const auto crop_start = image.GetVerticalCropStart();
  auto maybe_abw_points_in_image_coordinates =
      camera_model_->WorkspaceToImage(ABWPointsToMatrix(profile.points), crop_start);

  if (maybe_abw_points_in_image_coordinates) {
    auto abw_points_in_image_coordinates = maybe_abw_points_in_image_coordinates.value();
    auto bottom_pixel                    = static_cast<int>(abw_points_in_image_coordinates.row(1).maxCoeff());
    auto top_pixel                       = static_cast<int>(abw_points_in_image_coordinates.row(1).minCoeff());
    profile.vertical_limits              = {top_pixel + crop_start, bottom_pixel + crop_start};
  }

  const auto min_value = static_cast<double>(snake.min_pixel_value);
  const auto threshold = static_cast<double>(threshold_);

  if (min_value < threshold || min_value > 2.0 * threshold) {
    // Below 1.0, set target to 1.2
    profile.suggested_gain_change = {(1.5 * threshold) / min_value};
  }

  auto processing_time = (std::clock() - start) * 1000 / CLOCKS_PER_SEC;
  return std::make_tuple(profile, snake_lpcs, processing_time, num_walls);
}

auto BigSnake::GenerateMask(image::Image& image, std::optional<JointProfile> median_profile)
    -> std::optional<image::RawImageData> {
  // Is joint deep enough to apply a mask
  if (median_profile.has_value()) {
    auto profile = median_profile.value();
    if (profile.LeftDepth() < HIGH_CONFIDENCE_WALL_HEIGHT || profile.RightDepth() < HIGH_CONFIDENCE_WALL_HEIGHT) {
      return std::nullopt;
    }
  }

  return median_profile.transform(
      [this, image](JointProfile profile) -> Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> {
        const auto p = profile.points;

        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> mask =
            Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Ones(image.Data().rows(),
                                                                                          image.Data().cols());

        image::WorkspaceCoordinates mask_points_wcs(3, NUM_TRIANGLES_TO_MASK * 3);
        mask_points_wcs <<
            // clang-format off
                p[0].x + MASK_OFFSET, p[1].x, p[6].x - MASK_OFFSET,  // Triangle 1
                p[0].x + MASK_OFFSET, p[5].x, p[6].x - MASK_OFFSET, 
                p[0].x - MASK_OFFSET, p[0].x, p[1].x - MASK_OFFSET, 
                p[6].x + MASK_OFFSET, p[5].x + MASK_OFFSET, p[6].x, 
                p[0].y, p[1].y + MASK_OFFSET * 2., p[6].y,  // Triangle 1
                p[0].y, p[5].y + MASK_OFFSET * 2., p[6].y, 
                p[0].y - MASK_OFFSET, p[1].y, p[1].y, 
                p[6].y - MASK_OFFSET, p[5].y, p[5].y, 
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        // clang-format on

        auto maybe_img = camera_model_->WorkspaceToImage(mask_points_wcs, image.GetVerticalCropStart());

        if (maybe_img.has_value()) {
          const auto img   = maybe_img.value();
          const auto x_min = std::clamp(static_cast<long>(img.row(0).minCoeff()), 0l, mask.cols() - 1);
          const auto x_max = std::clamp(static_cast<long>(img.row(0).maxCoeff()), 0l, mask.cols() - 1);
          const auto y_min = std::clamp(static_cast<long>(img.row(1).minCoeff()), 0l, mask.rows() - 1);
          const auto y_max = std::clamp(static_cast<long>(img.row(1).maxCoeff()), 0l, mask.rows() - 1);

          auto sign = [](double p1x, double p1y, double p2x, double p2y, double p3x, double p3y) {
            return (p1x - p3x) * (p2y - p3y) - (p2x - p3x) * (p1y - p3y);
          };

          for (auto y = y_min; y <= y_max; y++) {
            const auto dy = static_cast<double>(y);
            for (auto x = x_min; x <= x_max; x++) {
              for (int triangle_index = 0; triangle_index < NUM_TRIANGLES_TO_MASK; triangle_index++) {
                const auto index_base = triangle_index * 3;
                const auto dx         = static_cast<double>(x);
                const auto v1x        = img(0, index_base);
                const auto v2x        = img(0, index_base + 1);
                const auto v3x        = img(0, index_base + 2);
                const auto v1y        = img(1, index_base);
                const auto v2y        = img(1, index_base + 1);
                const auto v3y        = img(1, index_base + 2);
                auto d1               = sign(dx, dy, v1x, v1y, v2x, v2y);
                auto d2               = sign(dx, dy, v2x, v2y, v3x, v3y);
                auto d3               = sign(dx, dy, v3x, v3y, v1x, v1y);
                if (!(((d1 < 0) || (d2 < 0) || (d3 < 0)) && ((d1 > 0) || (d2 > 0) || (d3 > 0)))) {
                  mask(y, x) = 0;
                }
              }
            }
          }
        }
        return mask;
      });
}

void BigSnake::CropImageHorizontal(image::Image& image, std::optional<JointProfile> median_profile) {
  if (median_profile.has_value()) {
    auto points =
        camera_model_->WorkspaceToImage(ABWPointsToMatrix(median_profile.value().points), image.GetVerticalCropStart())
            .value();
    auto start = static_cast<int>(points.row(0)[0] - START_SNAKE_OFFSET);
    auto stop  = static_cast<int>(points.row(0)[6] + START_SNAKE_OFFSET);
    image.SetHorizontalCrop(start, stop);
  }
}

auto BigSnake::CalculateJointArea(const ABWPoints& points) -> double {
  // Calculate the area with the shoelace formula
  double area = 0;
  for (int i = 0; i < 7; i++) {
    int j  = (i == 0) ? 6 : (i - 1);
    area  += points[j].x * points[i].y - points[i].x * points[j].y;
  }

  return 0.5 * area;
}
}  // namespace scanner::joint_model
