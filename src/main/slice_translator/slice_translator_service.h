#pragma once

#include <boost/outcome.hpp>
#include <vector>

#include "lpcs/lpcs_point.h"
#include "macs/macs_point.h"

namespace slice_translator {

class SliceTranslatorService {
 public:
  virtual ~SliceTranslatorService() = default;

  virtual auto LPCSToMCS(const std::vector<lpcs::Point>& lpcs_points, const macs::Point& axis_position)
      -> boost::outcome_v2::result<std::vector<macs::Point>> = 0;
  virtual auto AngleFromTorchToScanner(const std::vector<lpcs::Point>& lpcs_points, const macs::Point& axis_position)
      -> boost::outcome_v2::result<double> = 0;
};

}  // namespace slice_translator
