#include "coordinates_translator.h"

#include "common/logging/application_log.h"
#include "lpcs/lpcs_slice.h"
#include "macs/macs_groove.h"
#include "macs/macs_point.h"
#include "macs/macs_slice.h"
#include "slice_translator/slice_observer.h"
#include "slice_translator/slice_translator_service.h"
#include "slice_translator/slice_translator_service_v2.h"

using slice_translator::CoordinatesTranslator;

CoordinatesTranslator::CoordinatesTranslator(SliceTranslatorService* slice_translator,
                                             SliceTranslatorServiceV2* slice_translator_v2)
    : slice_translator_(slice_translator), slice_translator_v2_(slice_translator_v2) {}

void CoordinatesTranslator::AddObserver(SliceObserver* observer) { observers_.push_back(observer); }

void CoordinatesTranslator::OnScannerDataUpdate(const lpcs::Slice& data, const macs::Point& axis_position) {
  if (slice_translator_v2_->Available()) {
    OnScannerDataUpdateV2(data, axis_position);
    return;
  }

  auto machine_slice                 = macs::Slice{};
  machine_slice.time_stamp           = data.time_stamp;
  double angle_from_torch_to_scanner = 0.0;

  auto maybe_groove_lpcs = data.groove;
  if (maybe_groove_lpcs) {
    auto maybe_groove_mcs = slice_translator_->LPCSToMCS(maybe_groove_lpcs.value(), axis_position);
    if (maybe_groove_mcs) {
      machine_slice.groove = macs::Groove(maybe_groove_mcs.value());
      LOG_DEBUG("Transformed points with axis positions hori: {:.2f}, vert: {:.2f}", axis_position.horizontal,
                axis_position.vertical);
      LOG_DEBUG("Machine points: {}", machine_slice.Describe());
    } else {
      LOG_TRACE("No transformation of groove points: {}", maybe_groove_mcs.error().what());
    }
    auto maybe_angle_from_torch_to_scanner =
        slice_translator_->AngleFromTorchToScanner(maybe_groove_lpcs.value(), axis_position);
    if (maybe_angle_from_torch_to_scanner.has_value()) {
      angle_from_torch_to_scanner = maybe_angle_from_torch_to_scanner.value();
    }
  }

  auto maybe_line_mcs = slice_translator_->LPCSToMCS(data.line, axis_position);
  if (!maybe_line_mcs) {
    LOG_TRACE("No transformation of line points: {}", maybe_line_mcs.error().what());
    return;
  }

  machine_slice.line = maybe_line_mcs.value();

  for (auto* observer : observers_) {
    observer->Receive(machine_slice, data, axis_position, angle_from_torch_to_scanner);
  }
}

void CoordinatesTranslator::OnScannerDataUpdateV2(const lpcs::Slice& data, const macs::Point& axis_position) {
  macs::Slice machine_slice{.time_stamp = data.time_stamp};
  double angle_from_torch_to_scanner = 0.0;

  if (data.groove.has_value()) {
    auto groove_mcs = slice_translator_v2_->LPCSToMCS(data.groove.value(), axis_position);
    if (groove_mcs.has_value()) {
      machine_slice.groove = macs::Groove(groove_mcs.value());
      LOG_DEBUG("Transformed points with axis positions hori: {:.2f}, vert: {:.2f}", axis_position.horizontal,
                axis_position.vertical);
      LOG_DEBUG("Machine points: {}", machine_slice.Describe());
    } else {
      LOG_TRACE("LPCSToMCS() failed");
    }
    auto angle = slice_translator_v2_->AngleFromTorchToScanner(data.groove.value(), axis_position);
    if (angle.has_value()) {
      angle_from_torch_to_scanner = angle.value();
    }
  }

  auto line_mcs = slice_translator_v2_->LPCSToMCS(data.line, axis_position);
  if (!line_mcs.has_value()) {
    LOG_TRACE("LPCSToMCS() for line failed");
    return;
  }

  machine_slice.line = line_mcs.value();

  for (auto* observer : observers_) {
    observer->Receive(machine_slice, data, axis_position, angle_from_torch_to_scanner);
  }
}
