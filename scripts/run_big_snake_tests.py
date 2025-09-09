#!/usr/bin/env python3
"""
Script to run BigSnake::Parse() unit tests and generate a test report.
This script helps process all test images and validate the results.
"""

import os
import sys
import subprocess
import json
import time
from pathlib import Path

def find_test_images():
    """Find all test images in the test data directory."""
    test_data_dir = Path("src/scanner/joint_model/test/test_data")
    image_extensions = ['.tiff', '.tif', '.bmp', '.png']
    
    images = []
    if test_data_dir.exists():
        for ext in image_extensions:
            images.extend(test_data_dir.glob(f"*{ext}"))
    
    return [str(img) for img in images]

def run_tests():
    """Run the BigSnake unit tests."""
    print("Running BigSnake::Parse() unit tests...")
    
    # Change to the workspace directory
    os.chdir("/workspace")
    
    # Build the tests
    print("Building tests...")
    build_result = subprocess.run([
        "cmake", "--build", "build", "--target", "unit_tests"
    ], capture_output=True, text=True)
    
    if build_result.returncode != 0:
        print("Build failed:")
        print(build_result.stderr)
        return False
    
    # Run the tests
    print("Running tests...")
    test_result = subprocess.run([
        "./build/unit_tests", "--test-suite=BigSnake Parse Tests"
    ], capture_output=True, text=True)
    
    print("Test output:")
    print(test_result.stdout)
    
    if test_result.stderr:
        print("Test errors:")
        print(test_result.stderr)
    
    return test_result.returncode == 0

def generate_test_report():
    """Generate a test report with image processing results."""
    print("Generating test report...")
    
    # Find test images
    images = find_test_images()
    
    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "test_images": images,
        "total_images": len(images),
        "test_cases": [
            "Process all test images",
            "Test with horizontal cropping", 
            "Test with median profile",
            "Test error handling",
            "Performance test",
            "Compare with expected ABW points",
            "Test all scanner configurations"
        ]
    }
    
    # Save report
    with open("big_snake_test_report.json", "w") as f:
        json.dump(report, f, indent=2)
    
    print(f"Test report saved to big_snake_test_report.json")
    print(f"Found {len(images)} test images:")
    for img in images:
        print(f"  - {img}")

def main():
    """Main function."""
    print("BigSnake::Parse() Unit Test Runner")
    print("=" * 40)
    
    # Generate test report
    generate_test_report()
    
    # Run tests
    success = run_tests()
    
    if success:
        print("\n✅ All tests passed!")
        return 0
    else:
        print("\n❌ Some tests failed!")
        return 1

if __name__ == "__main__":
    sys.exit(main())