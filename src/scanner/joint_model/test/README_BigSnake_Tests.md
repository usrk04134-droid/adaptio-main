# BigSnake::Parse() Unit Tests

This directory contains comprehensive unit tests for the `BigSnake::Parse()` function, migrated from the old adaptio-core repository. These tests validate image processing functionality and compare results with expected ABW (Automatic Bead Width) points.

## Overview

The tests were created to replace the old image processing tests from adaptio-core that used a `data_set.yaml` file with annotated ABW points. The new unit tests:

1. Process all available test images
2. Validate the `BigSnake::Parse()` function output
3. Compare results with expected ABW points (when available)
4. Test various scanner configurations
5. Validate performance and error handling

## Test Files

- `big_snake_parse_test.cc` - Main test file containing all test cases
- `test_abw_data.h` - Test data with expected ABW points for comparison
- `test_data/` - Directory containing test images (TIFF, BMP formats)

## Test Cases

### 1. Process All Test Images
- Loads all available test images from the test data directory
- Processes each image using `BigSnake::Parse()`
- Validates basic output properties (ABW points, processing time, etc.)
- Reports success/failure statistics

### 2. Test with Horizontal Cropping
- Tests image processing with horizontal cropping applied
- Validates that cropping doesn't break the parsing functionality

### 3. Test with Median Profile
- Tests parsing with a provided median profile
- Validates that median profile improves processing accuracy

### 4. Test Error Handling
- Tests error conditions (e.g., very small images)
- Validates that appropriate errors are returned

### 5. Performance Test
- Runs multiple iterations to measure processing time
- Validates that processing time is within acceptable limits

### 6. Compare with Expected ABW Points
- Compares actual results with expected ABW points (when available)
- Validates joint width and depth measurements
- Reports differences within tolerance

### 7. Test All Scanner Configurations
- Tests different scanner threshold configurations
- Validates that different settings produce valid results

## Running the Tests

### Using CMake
```bash
cd /workspace
cmake --build build --target unit_tests
./build/unit_tests --test-suite="BigSnake Parse Tests"
```

### Using the Test Script
```bash
cd /workspace
python3 scripts/run_big_snake_tests.py
```

## Test Configuration

The tests use the following configuration:

### Camera Properties
- Focal length: 50mm
- Principal point: (1750, 1250)
- Pixel size: 5μm
- FOV: 3500x2500 pixels

### Joint Properties
- Upper joint width: 10mm
- Surface angles: 45 degrees
- Joint angles: 30 degrees
- Groove depth: 5mm

### Scanner Configuration
- Gray minimum wall threshold: 16 (configurable)
- Multiple threshold values tested: 16, 32, 64

## Expected Results

The tests validate:

1. **ABW Points**: Exactly 7 points with valid coordinates
2. **Joint Width**: Reasonable width (0 < width < 100mm)
3. **Joint Depth**: Reasonable depth (0 ≤ depth < 50mm)
4. **Processing Time**: Less than 1 second per image
5. **Point Ordering**: ABW0 (leftmost) ≤ ABW6 (rightmost)

## Adding New Test Images

To add new test images:

1. Place images in `test_data/` directory
2. Supported formats: TIFF, BMP, PNG
3. Images should contain visible joint features
4. Update expected ABW points in `test_abw_data.h` if known

## Adding Expected ABW Points

To add expected ABW points for comparison:

1. Edit `test_abw_data.h`
2. Add expected points for specific images
3. Adjust tolerance if needed (default: 1mm)
4. Update test cases to use the new data

## Migration from adaptio-core

This test suite replaces the old adaptio-core tests that used:

- `@ADAPTIO_CORE/tests/data_set/data_set.yaml` - Test data configuration
- Multiple test tools and scripts
- Manual validation of results

The new tests provide:

- Automated validation of all test images
- Comprehensive error checking
- Performance monitoring
- Easy addition of new test cases
- Integration with the existing test framework

## Troubleshooting

### Common Issues

1. **No test images found**: Ensure images are in `test_data/` directory
2. **Build failures**: Check that all dependencies are installed
3. **Test failures**: Review test output for specific error messages
4. **Performance issues**: Check image sizes and processing parameters

### Debug Information

The tests provide detailed output including:
- Image processing results
- ABW point coordinates
- Joint measurements
- Processing times
- Error messages

## Future Enhancements

Potential improvements:

1. Add more test images with known ground truth
2. Implement regression testing for specific image sets
3. Add visualization of test results
4. Create automated test data generation
5. Add integration with CI/CD pipeline