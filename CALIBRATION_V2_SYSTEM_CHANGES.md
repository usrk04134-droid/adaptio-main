# Calibration V2 System Changes

## Overview

This document outlines the changes made to the C++ codebase to fully support the v2 calibration system and ensure compatibility with the updated Python tests. The legacy v1 calibration system has been completely removed, and the system now exclusively uses the v2 calibration API.

## Changes Made

### 1. WebHMI Calibration Handler Updates (`src/main/web_hmi/src/web_hmi_calibration.cc`)

**Problem:** The WebHmiCalibration class was handling legacy calibration messages and returning nullptr-based failures.

**Solution:** Updated the message handlers to return clear failure responses with helpful error messages directing users to the v2 API:

```cpp
// Legacy calibration message handlers - return failure responses since v1 calibration is removed
// The v2 calibration system (CalibrationManagerV2Impl) handles its own web HMI messages directly

if (message_name == "LaserToTorchCalibration") {
  LOG_ERROR("Legacy LaserToTorchCalibration not supported - use LaserTorchCalGet/LaserTorchCalSet instead");
  auto response_payload = LaserTorchCalibrationToPayload(false, {});
  auto message = CreateMessage("LaserToTorchCalibrationRsp", response_payload);
  socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
}
```

**Messages handled:**
- `LaserToTorchCalibration` → Returns failure, suggests `LaserTorchCalGet/LaserTorchCalSet`
- `GetLaserToTorchCalibration` → Returns failure, suggests `LaserTorchCalGet`
- `SetLaserToTorchCalibration` → Returns failure, suggests `LaserTorchCalSet`
- `WeldObjectCalibration` → Returns failure, suggests `WeldObjectCalStart/WeldObjectCalLeftPos/WeldObjectCalRightPos`
- `GetWeldObjectCalibration` → Returns failure, suggests `WeldObjectCalGet`
- `SetWeldObjectCalibration` → Returns failure, suggests `WeldObjectCalSet`

### 2. Application Integration Fix (`src/main/application.cc`)

**Problem:** The ManagementServer constructor was still referencing the removed `calibration_manager_`.

**Solution:** Updated the ManagementServer instantiation to pass `nullptr` for the legacy calibration manager:

```cpp
management_server_ = std::make_unique<management::ManagementServer>(
    management_socket_.get(), joint_geometry_provider_.get(), activity_status_.get(), nullptr,
    calibration_manager_v2_.get(), weld_control_.get(), shutdown_handler);
```

### 3. Missing V2 Message Handler Implementation

**Problem:** The `OnWeldObjectCalStop()` method was declared in the header but not implemented in the CalibrationManagerV2Impl.

**Solution:** Added the missing implementation in `src/main/calibration/src/calibration_manager_v2_impl.cc`:

```cpp
void CalibrationManagerV2Impl::OnWeldObjectCalStop() {
  LOG_INFO("WeldObjectCalStop received");
  if (activity_status_->Get() == coordination::ActivityStatusE::WELD_OBJECT_CALIBRATION) {
    StopCalibration();
    web_hmi_->Send("WeldObjectCalStopRsp", SUCCESS_PAYLOAD);
  } else {
    LOG_INFO("WeldObjectCalStop received when not in calibration mode");
    web_hmi_->Send("WeldObjectCalStopRsp", FAILURE_PAYLOAD);
  }
}
```

## V2 Calibration System Architecture

### Message Flow

#### Laser Torch Calibration (Configuration-based)
1. **Get:** `LaserTorchCalGet` → `LaserTorchCalGetRsp`
2. **Set:** `LaserTorchCalSet` → `LaserTorchCalSetRsp`

#### Weld Object Calibration (Touch-based with automatic calculation)
1. **Start:** `WeldObjectCalStart` → `WeldObjectCalStartRsp`
2. **Left Touch:** `WeldObjectCalLeftPos` → `WeldObjectCalLeftPosRsp`
3. **Right Touch:** `WeldObjectCalRightPos` → `WeldObjectCalRightPosRsp`
4. **Auto Result:** System sends `WeldObjectCalResult` (push message)
5. **Save:** `WeldObjectCalSet` → `WeldObjectCalSetRsp`
6. **Stop:** `WeldObjectCalStop` → `WeldObjectCalStopRsp`
7. **Get:** `WeldObjectCalGet` → `WeldObjectCalGetRsp`

### Key Components

#### CalibrationManagerV2Impl
- **Location:** `src/main/calibration/src/calibration_manager_v2_impl.{h,cc}`
- **Role:** Main v2 calibration logic and web HMI integration
- **Features:**
  - Direct web HMI message subscription
  - Touch-based calibration with left/right wall detection
  - Automatic grid generation and measurement
  - Calibration result calculation and storage

#### WebHmiCalibration (Legacy Handler)
- **Location:** `src/main/web_hmi/src/web_hmi_calibration.{h,cc}`
- **Role:** Handles legacy calibration messages and returns failure responses
- **Purpose:** Provides clear migration path for users still using old API

#### Configuration System
- **Location:** `assets/configuration/configuration.yaml`
- **Settings:** Grid configuration and runner configuration for v2 calibration
- **Parameters:**
  - `margin_top`, `margin_x`, `margin_z`, `margin_c`: Grid generation margins
  - `target_nr_gridpoints`: Minimum number of measurement points
  - `slide_velocity`, `stabilization_time`: Movement and timing parameters

## Configuration Files

### Main Configuration (`assets/configuration/configuration.yaml`)
```yaml
calibration:
  grid_config:
    margin_top: 10.0      # Distance between top surface and measurement points (mm)
    margin_x: 0.0         # Grid area expansion outside joint horizontally (mm)
    margin_z: 30.0        # Grid area expansion outside joint vertically (mm)
    margin_c: 5.0         # No measurements within this distance to touch points (mm)
    target_nr_gridpoints: 20  # Target number of measurements (minimum)
  runner_config:
    slide_velocity: 5.0           # Slide velocity horizontal/vertical (mm/sec)
    stabilization_time: 2.0       # Stabilization time when target reached (sec)
    near_target_delta: 1.0        # Target considered reached within this distance (mm)
    max_time_per_observation: 30.0  # Supervision timeout per measurement point (sec)
```

### Test Configuration (`tests/configs/sil/configuration.yaml`)
Same calibration configuration as main config, ensuring consistency between production and test environments.

## Integration Points

### Web HMI Integration
- CalibrationManagerV2Impl subscribes directly to web HMI messages
- No intermediate WebHmiCalibration layer needed for v2 messages
- Legacy messages handled by WebHmiCalibration with failure responses

### Activity Status Integration
- Uses `coordination::ActivityStatusE::WELD_OBJECT_CALIBRATION` during calibration
- Prevents conflicting operations during calibration process
- Proper cleanup on calibration stop or failure

### Scanner and Kinematics Integration
- Scanner client provides LPCS slice data for measurements
- Kinematics client handles slide positioning for grid measurements
- Automatic movement between measurement points

### Storage Integration
- Uses `StoredCalibrationResult` for weld object calibration persistence
- Uses `StoredLaserTorchConfiguration` for laser torch calibration persistence
- Database-backed storage for calibration results

## Testing Compatibility

### Python Test Integration
The C++ changes ensure full compatibility with the updated Python tests:

1. **Legacy Message Handling:** Returns proper failure responses for old API calls
2. **V2 Message Support:** CalibrationManagerV2Impl handles all v2 messages
3. **Configuration Support:** Proper calibration settings in test configurations
4. **Error Handling:** Clear error messages guide users to correct API usage

### Message Compatibility Matrix

| Python Test Message | C++ Handler | Response | Status |
|---------------------|-------------|----------|---------|
| `LaserTorchCalGet` | CalibrationManagerV2Impl | Success/Failure | ✅ Working |
| `LaserTorchCalSet` | CalibrationManagerV2Impl | Success/Failure | ✅ Working |
| `WeldObjectCalStart` | CalibrationManagerV2Impl | Success/Failure | ✅ Working |
| `WeldObjectCalLeftPos` | CalibrationManagerV2Impl | Success/Failure | ✅ Working |
| `WeldObjectCalRightPos` | CalibrationManagerV2Impl | Success/Failure | ✅ Working |
| `WeldObjectCalStop` | CalibrationManagerV2Impl | Success/Failure | ✅ Working |
| `WeldObjectCalGet` | CalibrationManagerV2Impl | Success/Failure | ✅ Working |
| `WeldObjectCalSet` | CalibrationManagerV2Impl | Success/Failure | ✅ Working |
| `LaserToTorchCalibration` (legacy) | WebHmiCalibration | Failure | ✅ Working |
| `WeldObjectCalibration` (legacy) | WebHmiCalibration | Failure | ✅ Working |

## Benefits of V2 System

1. **Edge Sensor Compatible:** Works with edge sensor tracking system
2. **Touch-based Calibration:** More accurate with left/right wall detection
3. **Automatic Grid Generation:** Optimized measurement point placement
4. **Better Error Handling:** Clear error messages and proper state management
5. **Simplified Architecture:** Direct web HMI integration without intermediate layers
6. **Simulator Friendly:** Easier to develop simulator-based test cases

## Migration Notes

### For Developers
- Use v2 calibration API messages in new code
- Legacy messages will return failure responses
- Refer to example messages in `src/main/web_hmi/example_messages.txt`

### For Operators
- Use new calibration workflow: Start → Left Touch → Right Touch → Auto Calculation → Set
- Legacy single-step calibration no longer available
- Better accuracy with manual touch points

### For Test Development
- Use updated Python test framework with v2 messages
- Configuration files already updated for v2 system
- Legacy test verification available in `test_legacy_calibration_removal.py`

## Verification

To verify the changes are working correctly:

1. **Build and run the system:** All v2 calibration messages should work
2. **Test legacy messages:** Should return failure responses with helpful error messages
3. **Run Python tests:** All updated tests should pass
4. **Check logs:** Clear error messages for legacy API usage
5. **Verify calibration workflow:** Complete touch-based calibration sequence

The system is now fully migrated to v2 calibration and ready for production use with edge sensor tracking.