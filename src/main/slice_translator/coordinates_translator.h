#pragma once

#include "lpcs/lpcs_slice.h"
#include "scanner_client/scanner_client.h"
#include "slice_observer.h"
#include "slice_translator_service.h"
#include "slice_translator_service_v2.h"

namespace slice_translator {

class CoordinatesTranslator : public scanner_client::ScannerObserver {
 public:
  CoordinatesTranslator(SliceTranslatorService* slice_translator, SliceTranslatorServiceV2* slice_translator_v2);
  void AddObserver(SliceObserver* observer);

  // ScannerObserver
  void OnScannerStarted(bool success) override {};
  void OnScannerStopped(bool success) override {};
  void OnScannerDataUpdate(const lpcs::Slice& data, const macs::Point& axis_position) override;

 private:
  void OnScannerDataUpdateV2(const lpcs::Slice& data, const macs::Point& axis_position);

  SliceTranslatorService* slice_translator_;
  SliceTranslatorServiceV2* slice_translator_v2_;
  std::vector<SliceObserver*> observers_;
};

}  // namespace slice_translator
