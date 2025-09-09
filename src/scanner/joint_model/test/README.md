# BigSnake::Parse() Unit Tests

This directory contains unit tests for the `BigSnake::Parse()` method, which processes images to detect and calculate ABW (Adaptive Bead Welding) points from laser line data.

## Overview

The BigSnake algorithm analyzes laser line images to identify joint geometry and extract key welding points (ABW0-ABW14). These tests verify that the algorithm correctly processes various image types and produces accurate ABW point coordinates.

## Test Files

### Core Test Files

- **`big_snake_test.cc`**: Main unit test file with comprehensive test cases for BigSnake::Parse()
  - Tests normal image processing
  - Tests with median profiles
  - Tests with updated joint properties
  - Tests approximation mode
  - Tests with horizontal hints
  - Performance benchmarking

- **`big_snake_yaml_test.cc`**: YAML-driven test suite
  - Reads test cases from `data_set.yaml`
  - Supports batch processing of multiple images
  - Allows easy addition of new test cases without code changes

### Test Data

- **`test_data/data_set.yaml`**: Configuration file containing:
  - Test case definitions
  - Expected ABW points for each image
  - Joint properties and scanner configurations
  - Camera calibration parameters
  - Tolerance values for comparison

- **`test_data/*.tiff`**: Test images
  - `1755001276997.tiff`: Normal V-joint with clear laser line
  - `1754561083373.tiff`: Dark/black image (failure case)
  - Additional images can be added as needed

## Test Structure

Each test case includes:
1. **Input Image**: TIFF file with laser line data
2. **Joint Properties**: Geometric parameters of the expected joint
3. **Scanner Configuration**: Gray level thresholds and processing parameters
4. **Expected ABW Points**: Annotated coordinates for verification
5. **Tolerance**: Acceptable deviation in millimeters

## ABW Points

The algorithm identifies 15 ABW points (ABW0-ABW14):
- **ABW0**: Left top edge
- **ABW1**: Left wall upper
- **ABW2**: Left wall lower
- **ABW3**: Bottom left
- **ABW4**: Bottom center (root)
- **ABW5**: Bottom right
- **ABW6**: Right wall lower
- **ABW7**: Right wall upper
- **ABW8**: Right top edge
- **ABW9-14**: Reserved for future use

## Running the Tests

### Using the Test Script
```bash
./run_big_snake_tests.sh
```

### Using Nix
```bash
nix build .#adaptio-unit-tests
./result/bin/adaptio-unit-tests --test-case="*BigSnake*"
```

### Using CMake
```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make adaptio-unit-tests
./src/adaptio-unit-tests --test-case="*BigSnake*"
```

### Running Specific Test Suites
```bash
# Run only the main BigSnake tests
./adaptio-unit-tests --test-case="BigSnake::Parse*"

# Run only the YAML-driven tests
./adaptio-unit-tests --test-case="*YAML*"

# Run with verbose output
./adaptio-unit-tests --test-case="*BigSnake*" --success
```

## Adding New Test Cases

### Method 1: Add to YAML Configuration

1. Edit `test_data/data_set.yaml`
2. Add a new test case entry with:
   ```yaml
   - name: "Your test name"
     image: "your_image.tiff"
     description: "Description of the test"
     joint_properties:
       # ... joint parameters
     scanner_config:
       # ... scanner parameters
     expected_result:
       success: true/false
       tolerance: 0.5  # mm
       abw_points:
         ABW0: { x: -15.0, y: 8.0 }
         # ... other points
   ```

### Method 2: Add to C++ Test File

1. Edit `big_snake_test.cc`
2. Add a new TEST_CASE or SUBCASE
3. Follow the existing pattern for image loading and verification

## Test Coverage

The test suite covers:
- ✅ Normal V-joint processing
- ✅ Dark/black image handling (failure cases)
- ✅ Processing with median profiles
- ✅ Updated joint properties
- ✅ Approximation mode
- ✅ Horizontal hints
- ✅ Performance benchmarking
- ✅ Batch processing of multiple images

## Migration from adaptio-core

These tests were migrated from the original adaptio-core repository where they existed as integration tests. The migration involved:
1. Converting from integration tests to unit tests
2. Focusing on testing `BigSnake::Parse()` directly
3. Using doctest framework instead of custom test runners
4. Maintaining the same test data and expected results
5. Adding YAML-based configuration for easier test management

## Troubleshooting

### Missing Images
If test images are not found:
1. Ensure all TIFF files are in the `test_data/` directory
2. Check file permissions
3. Verify paths in `data_set.yaml`

### Tolerance Issues
If tests fail due to small differences:
1. Review the tolerance values in `data_set.yaml`
2. Consider if the differences are within acceptable engineering tolerances
3. Update expected values if the algorithm has been intentionally modified

### Build Errors
If compilation fails:
1. Ensure all dependencies are available (OpenCV, Eigen, doctest)
2. Check that CMakeLists.txt includes the test files
3. Verify that the scanner module is built before tests

## Future Improvements

- [ ] Add more diverse test images (different joint types, lighting conditions)
- [ ] Include edge cases (partial images, noisy data)
- [ ] Add regression tests for specific bug fixes
- [ ] Implement automated annotation tools for new test images
- [ ] Add visualization tools for debugging failed tests