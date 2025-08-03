#pragma once

#include <boost/outcome.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <utility>

#include "scanner/image/image.h"
#include "scanner/scanner_types.h"

namespace scanner::image_provider {

class ImageProvider {
 public:
  using OnImage = std::function<void(std::unique_ptr<image::Image>)>;

  ImageProvider() {};

  ImageProvider(ImageProvider&)                     = delete;
  auto operator=(ImageProvider&) -> ImageProvider&  = delete;
  ImageProvider(ImageProvider&&)                    = delete;
  auto operator=(ImageProvider&&) -> ImageProvider& = delete;

  virtual ~ImageProvider() = default;

  virtual void SetOnImage(OnImage on_image) = 0;

  virtual auto Start(enum scanner::ScannerSensitivity sensitivity) -> boost::outcome_v2::result<void> = 0;
  virtual void Stop()                                                                                 = 0;
  [[nodiscard]] virtual auto Started() const -> bool                                                  = 0;

  virtual void ResetFOVAndGain()                               = 0;
  virtual void SetVerticalFOV(int offset_from_top, int height) = 0;
  virtual void AdjustGain(double factor)                       = 0;
  virtual auto GetVerticalFOVOffset() -> int                   = 0;
  virtual auto GetVerticalFOVHeight() -> int                   = 0;
  virtual auto GetSerialNumber() -> std::string                = 0;
};

using ImageProviderPtr = std::unique_ptr<ImageProvider>;

}  // namespace scanner::image_provider
