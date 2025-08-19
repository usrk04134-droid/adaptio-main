# Diagram

```plantuml
left to right direction

package slice_translator {
  package calibration {
    ' legacy CalibrationManager removed
  }

  class "SliceTranslator" as slice_translator::SliceTranslator
  struct "CircularWeldObjectCalibration" as calibration::WeldObjectCalibration
  struct "LaserTorchCalibration" as calibration::LaserTorchCalibration
}
```