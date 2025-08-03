#include "scanner/scanner_factory_impl.h"

#include <prometheus/registry.h>

#include <chrono>
#include <functional>
#include <memory>
#include <utility>

#include "scanner/image/tiff_handler_impl.h"
#include "scanner/image/tilted_perspective_camera.h"
#include "scanner/image_logger/image_logger.h"
#include "scanner/image_logger/image_logger_impl.h"
#include "scanner/image_provider/basler_camera.h"
#include "scanner/image_provider/image_provider.h"
#include "scanner/image_provider/image_provider_configuration.h"
#include "scanner/image_provider/simulation/camera_simulation.h"
#include "scanner/joint_buffer/circular_joint_buffer.h"
#include "scanner/joint_model/big_snake.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/scanner.h"
#include "scanner/scanner_calibration_configuration.h"
#include "scanner/scanner_configuration.h"
#include "scanner/scanner_factory.h"
#include "scanner/scanner_impl.h"
#include "scanner/slice_provider/slice_provider_impl.h"

namespace scanner {

std::unique_ptr<ScannerFactory> s_factory;

// Generator is typically a lambda capturing a Factory instance
// controlled by testcase.
std::function<ScannerFactory*()> s_generator;

auto GetFactory() -> ScannerFactory* {
  if (s_generator) {
    return s_generator();
  }

  if (!s_factory) {
    s_factory = std::make_unique<ScannerFactoryImpl>();
  }

  return s_factory.get();
}

// For test
// can set an empty std::function to release Factory instance
// captured in generator previously set.
void SetFactoryGenerator(std::function<ScannerFactory*()> generator) { s_generator = std::move(generator); }

auto ScannerFactoryImpl::CreateScanner(image_provider::ImageProvider* image_provider,
                                       const ScannerCalibrationData& scanner_calibration,
                                       const ScannerConfigurationData& scanner_configuration,
                                       const image_provider::Fov& fov,
                                       const joint_model::JointProperties& joint_properties,
                                       ScannerOutputCB* scanner_output, image_logger::ImageLogger* logger,
                                       prometheus::Registry* registry) -> ScannerPtr {
  auto joint_buffer          = std::make_unique<joint_buffer::CircularJointBuffer>();
  auto steady_clock_now_func = []() { return std::chrono::steady_clock::now(); };
  auto slice_provider =
      std::make_unique<slice_provider::SliceProviderImpl>(std::move(joint_buffer), steady_clock_now_func);
  image::TiltedPerspectiveCameraProperties camera_properties;
  camera_properties.config_calib = scanner_calibration;
  camera_properties.config_fov   = fov;

  auto camera_model = std::make_unique<image::TiltedPerspectiveCamera>(camera_properties);
  return std::make_unique<ScannerImpl>(
      image_provider, std::move(slice_provider), [](bool state) {}, scanner_output,
      joint_model::JointModelPtr(
          new joint_model::BigSnake(joint_properties, scanner_configuration, std::move(camera_model))),
      logger, registry);
}

auto ScannerFactoryImpl::CreateImageProvider(const image_provider::ImageProviderConfigData& image_provider_config,
                                             prometheus::Registry* registry) -> image_provider::ImageProviderPtr {
  if (image_provider_config.image_provider == image_provider::ImageProviderType::BASLER) {
    return std::make_unique<image_provider::BaslerCamera>(image_provider_config.basler_config,
                                                          image_provider_config.fov, registry);
  }

  return std::make_unique<image_provider::CameraSimulation>(image_provider_config.sim_config,
                                                            true /*loop images for tests*/);
}

auto ScannerFactoryImpl::CreateImageLogger() -> image_logger::ImageLoggerPtr {
  auto tiff_handler = std::make_unique<image::TiffHandlerImpl>();

  return std::make_unique<image_logger::ImageLoggerImpl>(std::move(tiff_handler));
}
}  // namespace scanner
