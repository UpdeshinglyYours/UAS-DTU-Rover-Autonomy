
# Changelog
All notable changes to this project will be documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

### Changed

### Fixed

### Removed

## [1.5.5] - 2023.05.03

### Added
- [#48959] logging throttling interval option

### Fixed
- [#48959] Fix bug: frame mode separated in scan pattern causes the point cloud to freeze

## [1.5.4] - 2023.02.15

### Fixed
- [#45042] Fix scanlines overwriting in image creation with the scanline preserving only up

## [1.5.3] - 2022.11.24

### Changed
- [#39667] Range and intensity image is created based on strongest return 
- [#43779] Update the ComposableNodeContainer to use multi threading

### Fixed
- [#39667] Fix stripes in images generated for scanline preserving and angle preserving

## [1.5.2] - 2022.10.18

### Changed
- [#42724] Use COMPOSITION_BUILDING_DLL instead of MINIMAL_COMPOSITION_DLL and set node plugins

## [1.5.1] - 2022.07.05

### Fixed
- [#40410] Fix typos in README and add information for remap and point_id image options

## [1.5.0] - 2022.06.29

### Added
- [#37666] Added return id to point cloud fields if multi return is activated (returns_publishing_options: "all") 
- [#38464] Added publishing of point id image
- [#39345] Added meters per second sqaured as a unit option for imu acceleration

### Changed
- [#38022] update documentation
- [#38314] Update publishers to use unique pointers

## [1.4.0] - 2022.02.25

### Added
- [#35687] Added composable node implementation

### Fixed
- [#35687] Fixed remapping of output topics for python launch file

## [1.3.3] - 2022.01.24

### Fixed
- Fix intensity type

## [1.3.2] - 2022.01.21

### Changed
- [#35609] service names are configurable in live_scanner.launch.xml and driver_config.yaml files
- [#35909] Imu static tf service returns transformation as service response 

### Fixed
- Update readme regarding default DDS 

## [1.3.1] - 2021.10.28

### Fixed
- README formating

### Removed
- Support for ros Eloquent

## [1.3.0] - 2021.10.28

### Added
- [closes #30445] Integrate with ros blickfeld driver core
- [closes #36445] Add on device algorithm setup
- [closes #36445] publishing 2d point cloud projection based on range, intensity and ambient
- [closes #31485] Publish IMU data as ros IMU message
- [closes #31486] Publish IMU static data as static TF transform

### Changed
- Target names and class names have blickfeld as prefix
- [closes #32741] Create custom message for setScanPatternService call in ros core
- [closes #31485] Upgrade bsl version to 2.18.2
- [closes #31485] bsl version in ci_pipeline was changed to 2.18.2

### Removed
- Scanline ID was removed from point cloud published field options

## [1.2.4] - 2021.02.01

### Changed
- Updated documentation, switched Note with Default column in parameters table

## [1.2.3] - 2021.01.28

### Changed
- Updated documentation
- Enforce non-standalone BSL. Shows CMake error if BSL without required protocol files is configured.

## [1.2.2] - 2020.11.11

### Added
- Added documentation for ROS2 Foxy

## [1.2.1] - 2020.09.08

### Changed
- Fix azimuth angle for no return points

## [1.2.0] - 2020.07.17

### Added
- Enhance time debug information on RCLCPP debug - ticket [22666]
- Automatic ci release process - ticket [23492]

## [1.1.1] - 2020.06.25

### Changed
- Updated documentation

## [1.1.0] - 2020.04.26

### Added
- BSD 3-clause license
- Debug and warning info frame index

### Changed
- Replaced the row_id with scanline_id to be compliant with protobuf_protocol
- Replaced the column_id with scanline_point_index to represent the point index in a scanline
- Unified the usage publish_time_delta

## [1.0.0] - 2020.04.06

### Added
- Ability to publish explicit range of points
- Debug tool to view all point fields in the ROS message
- Ability to publish Blickfeld PointID (global ID in the frame), ScanLineID (= row_id) and column_id
- Publish diagnostics data
- Possibility to publish point measurement time as [`start_offset_ns`](http://enduser-software.pages.muc.blickfeld.com/blickfeld-scanner-lib/protobuf_protocol.html#point) as point field
- Ability to publish multi return points
- Ability to publish under canonical topic names (names ending with `_in`/`_out` for input/output topics)

### Changed
- Update the use of point cloud term - ticket [18362]
- ROS2 adaptation

## [0.3.0] - 2019.10.30

### Added
- Ability to publish intensities, ambient light level, and point without return
- Multiple return support
- Try to reconnect after connection loss

### Changed
- Adopt to new BSL API
- Publish PointCloud2::ConstPtr to make full use of nodelets

### Removed
- Burst mode
- Old launch files
- Read from dump since BSL does not support it, yet

