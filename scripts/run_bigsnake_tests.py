#!/usr/bin/env python3
"""
Script to validate BigSnake::Parse() unit tests
This script helps validate the test setup and provides a summary of test coverage.
"""

import os
import sys
import yaml
from pathlib import Path

def validate_test_data():
    """Validate the test data configuration and files."""
    test_data_path = Path(__file__).parent.parent / "src/scanner/joint_model/test/test_data_set.yaml"
    
    if not test_data_path.exists():
        print(f"ERROR: Test data file not found: {test_data_path}")
        return False
    
    try:
        with open(test_data_path, 'r') as f:
            data = yaml.safe_load(f)
    except Exception as e:
        print(f"ERROR: Failed to load test data YAML: {e}")
        return False
    
    print("BigSnake::Parse() Test Data Validation")
    print("=" * 50)
    
    # Validate images
    images = data.get('test_data', {}).get('images', [])
    print(f"Images: {len(images)}")
    
    missing_images = []
    for img in images:
        img_path = Path(__file__).parent.parent / img['path']
        if not img_path.exists():
            missing_images.append(img['name'])
        else:
            # Check if image has expected failure flag
            expect_failure = img.get('expect_failure', False)
            status = "(expect failure)" if expect_failure else "(should process)"
            print(f"  ✓ {img['name']} {status}")
    
    if missing_images:
        print("Missing image files:")
        for img in missing_images:
            print(f"  ✗ {img}")
    
    # Validate scanner configurations
    scanners = data.get('test_data', {}).get('scanner_configurations', [])
    print(f"\nScanner Configurations: {len(scanners)}")
    
    for scanner in scanners:
        scanner_path = Path(__file__).parent.parent / scanner['path']
        if scanner_path.exists():
            print(f"  ✓ {scanner['name']}")
        else:
            print(f"  ✗ {scanner['name']} (file not found: {scanner['path']})")
    
    # Validate test cases
    test_cases = data.get('test_data', {}).get('test_cases', [])
    print(f"\nTest Cases: {len(test_cases)}")
    
    success_cases = [tc for tc in test_cases if not tc.get('expect_failure', False)]
    failure_cases = [tc for tc in test_cases if tc.get('expect_failure', False)]
    
    print(f"  Success cases: {len(success_cases)}")
    print(f"  Failure cases: {len(failure_cases)}")
    
    # Group by image
    image_groups = {}
    for tc in test_cases:
        img = tc['image']
        if img not in image_groups:
            image_groups[img] = []
        image_groups[img].append(tc['scanner'])
    
    print(f"\nTest Coverage:")
    for img, scanners in image_groups.items():
        print(f"  {img}: {len(scanners)} scanner configs")
    
    print(f"\nTotal test combinations: {len(test_cases)}")
    
    return len(missing_images) == 0

def print_usage():
    """Print usage information for running the actual tests."""
    print("\nTo run the actual BigSnake::Parse() unit tests:")
    print("1. Build the project with tests enabled")
    print("2. Run: ./build/src/unit_tests --test-case='BigSnake Parse Tests'")
    print("\nOr to run all scanner tests:")
    print("   ./build/src/unit_tests --test-suite='BigSnake Parse Tests'")

if __name__ == "__main__":
    success = validate_test_data()
    print_usage()
    
    if success:
        print("\n✓ Test data validation successful!")
        sys.exit(0)
    else:
        print("\n✗ Test data validation failed!")
        sys.exit(1)