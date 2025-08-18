# Calibration Tests V2 Migration

## Overview

The Python system tests have been updated to use the new v2 calibration API after the removal of the legacy v1 calibration system. This document explains the changes and how to use the new tests.

## Changes Made

### 1. Message Factory Updates (`message_factory.py`)

**Added new v2 calibration message functions:**
- `WeldObjectCalStart()` - Start weld object calibration
- `WeldObjectCalStop()` - Stop weld object calibration
- `WeldObjectCalLeftPos()` - Record left wall touch position
- `WeldObjectCalRightPos()` - Record right wall touch position
- `WeldObjectCalGet()` - Get current weld object calibration
- `WeldObjectCalSet()` - Set weld object calibration
- `LaserTorchCalGet()` - Get laser torch calibration
- `LaserTorchCalSet()` - Set laser torch calibration

**Deprecated legacy functions:**
- `LaserToTorchCalibration()` - Now deprecated, will return failure responses
- `WeldObjectCalibration()` - Now deprecated, will return failure responses

### 2. Updated Test Files

#### `test_laser_to_torch_calibration.py`
**Old behavior:** Used legacy `LaserToTorchCalibration` message with automatic calibration
**New behavior:** 
- Uses `LaserTorchCalGet()` to retrieve current calibration
- Uses `LaserTorchCalSet()` to set new calibration values
- Verifies the calibration was set correctly

#### `test_weld_object_calibration.py`
**Old behavior:** Used legacy `WeldObjectCalibration` message with automatic calibration
**New behavior:**
- Uses `WeldObjectCalStart()` to begin calibration session
- Simulates operator touch inputs with `WeldObjectCalLeftPos()` and `WeldObjectCalRightPos()`
- Waits for automatic `WeldObjectCalResult` push message
- Uses `WeldObjectCalSet()` to save the calibration
- Verifies with `WeldObjectCalGet()`

#### `test_legacy_calibration_removal.py` (New)
**Purpose:** Verifies that legacy calibration methods correctly return failure responses

## V2 Calibration Process

### Laser Torch Calibration
The v2 laser torch calibration is now a simple get/set operation:

1. **Get current calibration:** `LaserTorchCalGet` → `LaserTorchCalGetRsp`
2. **Set new values:** `LaserTorchCalSet` → `LaserTorchCalSetRsp`
3. **Verify:** `LaserTorchCalGet` → `LaserTorchCalGetRsp`

**Parameters:**
- `distanceLaserTorch`: Distance between laser and torch
- `stickout`: Wire stickout length
- `scannerMountAngle`: Scanner mounting angle

### Weld Object Calibration
The v2 weld object calibration uses a multi-step process with operator interaction:

1. **Start calibration:** `WeldObjectCalStart` → `WeldObjectCalStartRsp`
2. **Left wall touch:** `WeldObjectCalLeftPos` → `WeldObjectCalLeftPosRsp`
3. **Right wall touch:** `WeldObjectCalRightPos` → `WeldObjectCalRightPosRsp`
4. **Automatic calculation:** System sends `WeldObjectCalResult` (push message)
5. **Save calibration:** `WeldObjectCalSet` → `WeldObjectCalSetRsp`
6. **Verify:** `WeldObjectCalGet` → `WeldObjectCalGetRsp`

**Start Parameters:**
- `wireDiameter`: Wire diameter in mm
- `stickout`: Wire stickout length in mm
- `weldObjectRadius`: Weld object radius in mm

**Result Parameters:**
- `laserlineClockPosition`: Laser line clock position
- `weldObjectOrientation`: Object orientation vector
- `calibrationCenter`: Calibration center coordinates

## Running the Tests

### Prerequisites
- Adaptio application built and ready to run
- Test configuration files updated (legacy config file references removed)
- ZMQ sockets configured for communication

### Test Execution
```python
# Run individual tests
python -m pytest tests/test_laser_to_torch_calibration.py
python -m pytest tests/test_weld_object_calibration.py
python -m pytest tests/test_legacy_calibration_removal.py

# Run all calibration tests
python -m pytest tests/test_*calibration*.py
```

## Important Notes

1. **Legacy API Removal:** The old `LaserToTorchCalibration` and `WeldObjectCalibration` messages will return `valid: false` responses since the CalibrationManagerImpl has been removed.

2. **V2 API Requirements:** The v2 calibration system requires:
   - Edge sensor tracking enabled
   - Proper joint geometry configuration
   - Scanner and kinematics systems operational

3. **Test Environment:** Tests are designed to run in SIL (Software-in-the-Loop) mode with simulation components.

4. **Timeout Adjustments:** The weld object calibration test uses longer timeouts (30 seconds) for the automatic calibration calculation phase.

## Troubleshooting

### Common Issues
1. **"Legacy calibration manager not available" errors:** This is expected behavior - the legacy system has been removed
2. **Timeout on WeldObjectCalResult:** Ensure the scanner and kinematics systems are properly initialized
3. **Joint geometry errors:** Make sure `SetJointGeometry()` is called before starting calibration

### Debug Tips
- Enable verbose logging to see all message exchanges
- Check that the configuration files don't reference removed calibration YAML files
- Verify that the v2 calibration system (CalibrationManagerV2Impl) is properly initialized

## Migration Checklist

- [x] Update message factory with v2 calibration messages
- [x] Convert laser torch calibration test to use get/set API
- [x] Convert weld object calibration test to use multi-step touch API
- [x] Add legacy calibration removal verification test
- [x] Document deprecated message functions
- [x] Remove references to deleted configuration files
- [x] Update test timeouts for v2 calibration process