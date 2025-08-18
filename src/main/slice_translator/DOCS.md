# Diagram

```plantuml
left to right direction

package slice_translator {
  package calibration {
    interface "CalibrationManager" as calibration::CalibrationManager
    class "CalibrationManagerV2Impl" as calibration::CalibrationManagerV2Impl

    calibration::CalibrationManager <|-- calibration::CalibrationManagerV2Impl
  }

  class "SliceTranslator" as slice_translator::SliceTranslator
  struct "CircularWeldObjectCalibration" as calibration::WeldObjectCalibration
  struct "LaserTorchCalibration" as calibration::LaserTorchCalibration
}
```
