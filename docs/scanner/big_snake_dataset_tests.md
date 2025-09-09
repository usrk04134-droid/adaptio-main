BigSnake dataset tests

Overview

Unit tests were added to validate `BigSnake::Parse()` against a dataset of images with annotated ABW points. The tests run with doctest as part of `adaptio-unit-tests`.

How it works

- Dataset-driven test: If a dataset YAML is present, the test will load each scanner calibration and each image and assert that all 7 ABW points match the expected coordinates within a configurable epsilon.
- Smoke test fallback: If no dataset is provided, the test parses all images in `src/scanner/joint_model/test/test_data` using all calibrations in `assets/scanner_calibration`, asserting that parsing succeeds and ABW x-coordinates are strictly increasing.

Dataset location

- Default: `tests/data_set/data_set.yaml`
- Or set environment variable `ADAPTIO_BIGSNAKE_DATASET=/absolute/path/to/data_set.yaml`

Dataset YAML schema

Root keys:

- `epsilon` (double, optional): absolute tolerance for ABW coordinate comparison. Default `1e-4`.
- `filtering` (optional):
  - `gray_minimum_top` (int)
  - `gray_minimum_wall` (int)
  - `gray_minimum_bottom` (int)
- `joint` (optional defaults for all scanners/images):
  - `upper_joint_width` (double)
  - `left_max_surface_angle` (double, rad)
  - `right_max_surface_angle` (double, rad)
  - `left_joint_angle` (double, rad)
  - `right_joint_angle` (double, rad)
  - `groove_depth` (double)
  - `upper_joint_width_tolerance` (double, mm)
  - `surface_angle_tolerance` (double, rad)
  - `groove_angle_tolerance` (double, rad)
  - `offset_distance` (double, mm)
- `scanners` (array, required): each entry has:
  - `calibration` (string, path to scanner calibration YAML)
  - `joint` (optional): overrides of joint for this scanner (same keys as above)
  - `images` (array, required): each entry has:
    - `file` (string, path to image file)
    - `expected_abw` (array of 7 objects, required): each object has `x` and `y` doubles specifying expected ABW0..ABW6 in LPCS.
    - `joint` (optional): overrides of joint for this image (same keys as above)

Notes

- Relative paths in the dataset are resolved relative to the dataset YAML location.
- Calibrations must be in the same YAML format as files under `assets/scanner_calibration/`.
- Images should be grayscale `.tiff`, `.tif` or `.bmp` that the OpenCV loader can read.

Running

Build tests:

```bash
./adaptio.sh --build-tests
```

Run unit tests:

```bash
./adaptio.sh --unit-tests
```

With a custom dataset path:

```bash
ADAPTIO_BIGSNAKE_DATASET=/path/to/data_set.yaml ./build/debug/src/adaptio-unit-tests
```

