#pragma once

#include <common/types/vector_3d.h>

#include <array>

namespace slice_translator {

/*This interface is to be used to set model parameters as precondition for
  joint tracking or automatic welding.*/

class ModelConfig {
 public:
  virtual ~ModelConfig() = default;

  virtual void Set(const common::Vector3D& rot_center, std::array<double, 3> scanner_angles,
                   const common::Vector3D& weld_object_rotation_axis,
                   const common::Vector3D& torch_to_laser_translation) = 0;
  virtual void Reset()                                                                                         = 0;
};

}  // namespace slice_translator
