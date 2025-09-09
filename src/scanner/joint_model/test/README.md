# BigSnake::Parse() Unit Tests

This directory contains comprehensive unit tests for the `BigSnake::Parse()` function, migrated and enhanced from the original adaptio-core repository tests.

## Overview

The tests validate image processing functionality for different joint geometries and scanner configurations, ensuring that the ABW (Adaptive Welding Bead) point detection works correctly across various scenarios.

## Test Structure

### Test Files

- `big_snake_parse_test.cc` - Main unit test implementation using doctest framework
- `test_data_set.yaml` - Test configuration defining images, scanner configs, and test cases
- `test_data/` - Directory containing test images (TIFF format)

### Test Data

The test data includes:

1. **Test Images** (`test_data/` directory):
   - `1755001276997.tiff` - Valid joint image for processing
   - `1754561083373.tiff` - Black/empty image (expected to fail)
   - `1738232679592.tiff` - Additional test image
   - `1738243625597.tiff` - Additional test image

2. **Scanner Configurations**:
   - HIL scanner configuration
   - LX31624160019 scanner configuration  
   - LX31624160053 scanner configuration

3. **Test Cases**:
   - Each image is tested with each scanner configuration
   - Both success and failure cases are covered
   - Different tolerance levels for ABW point comparison

## Test Implementation

### Key Test Features

1. **Comprehensive Coverage**: Tests all available images with all scanner configurations
2. **Expected Results**: Compares actual ABW points with expected values (when available)
3. **Error Handling**: Validates proper failure handling for invalid images
4. **Performance**: Checks processing time constraints
5. **Edge Cases**: Tests with empty images, null inputs, etc.

### Test Cases

The main test suite `"BigSnake Parse Tests"` includes:

- **Comprehensive Image Processing Tests**: Processes all image/scanner combinations
- **Edge Cases**: Tests error conditions and boundary cases
- **Median Profile Tests**: Tests processing with previous results as input

## Usage

### Building and Running Tests

```bash
# Build the project with tests
mkdir build && cd build
cmake .. -DBUILD_TESTING=ON
make

# Run all BigSnake tests
./src/unit_tests --test-suite="BigSnake Parse Tests"

# Run specific test case
./src/unit_tests --test-case="BigSnake::Parse - Comprehensive Image Processing Tests"
```

### Validating Test Data

Use the validation script to check test data integrity:

```bash
python3 scripts/run_bigsnake_tests.py
```

## Test Data Configuration

The `test_data_set.yaml` file defines:

```yaml
test_data:
  images:
    - name: "image_name.tiff"
      path: "./path/to/image"
      expected_abw_points:
        abw0: {x: 0.047, y: -0.280}
        # ... abw1-abw6
      joint_geometry:
        upper_joint_width: 50.0
        # ... other parameters
      expect_failure: false  # optional

  scanner_configurations:
    - name: "scanner_name"
      path: "path/to/config.yaml"
      config:
        gray_minimum_wall: 16
        # ... other parameters

  test_cases:
    - image: "image_name.tiff"
      scanner: "scanner_name"
      tolerance: 0.001
      expect_failure: false  # optional
```

## Expected Results

In a production environment with annotated ground truth data, the tests would:

1. Load expected ABW point coordinates from annotations
2. Compare actual results with expected values within tolerance
3. Validate joint area calculations
4. Check processing performance metrics

Currently, the tests focus on:
- Successful processing of valid images
- Proper error handling for invalid images
- Reasonable processing times
- Valid output structure

## Migration from adaptio-core

This test suite replaces the original adaptio-core test structure:
- Original: `@ADAPTIO_CORE/tests/data_set/data_set.yaml`
- New: `src/scanner/joint_model/test/test_data_set.yaml`

Key improvements:
- Uses doctest framework for better integration
- Comprehensive scanner configuration testing
- Better error handling and edge case coverage
- Performance validation
- Structured test data organization

## Adding New Tests

To add new test cases:

1. Add test images to `test_data/` directory
2. Update `test_data_set.yaml` with new image definitions
3. Add corresponding test cases
4. Update expected ABW points (when ground truth is available)
5. Run validation script to verify setup

## Notes

- Expected ABW points in the current configuration are placeholder values
- In a production environment, these should be replaced with actual annotated ground truth data
- The tests are designed to be robust against missing files and configuration errors
- Processing time limits are set to reasonable values (< 10 seconds per image)