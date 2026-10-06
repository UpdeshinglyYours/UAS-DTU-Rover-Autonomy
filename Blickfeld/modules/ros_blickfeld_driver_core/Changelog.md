# Changelog

All notable changes to this project will be documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

### Changed

### Fixed

### Removed

## [0.3.3] - 2023.05.03

### Added

### Changed

### Fixed
- Fix bug: should not calculate the image size when the image publishing is turned off or not supported [#48959]

### Removed

## [0.3.2] - 2023.02.15

### Added

### Changed

### Fixed
- Fix scanlines overwriting in image creation with the scanline preserving only up [#45042]

### Removed
- Scanline ID was removed since release 0.1.0

## [0.3.1] - 2022.11.21

### Added

### Changed
- Use built-in Eigen method for computing static imu transform [#43302]
### Fixed
- Fix stripes in images generated for scanline preserving and angle preserving [#39667]

### Removed


## [0.3.0] - 2022.6.29

### Added
- Added return id to point cloud fields if multi return is activated (returns_publishing_options: "all") [#37666] 
- Added creation of point id image [#38464]
- Added meters per second sqaured as a unit option for imu acceleration [#39345]

### Changed
- Generalize pointers usage to be able to deal with unique pointers [#38314]

### Fixed

### Removed

## [0.2.4] - 2022.1.24

### Added

### Changed

### Fixed

- Fix intensity type

### Removed

## [0.2.3] - 2022.1.20

### Added

### Changed

- Passing by reference as inputs of publishImuStaticTF instead of pointer

### Fixed

### Removed

## [0.2.2] - 2021.10.28

### Added

- License file

### Changed

### Fixed

### Removed

## [0.2.1] - 2021.10.28

### Added

### Changed

### Fixed

- Catkin lint warnings

### Removed

## [0.2.0] - 2021.10.28

### Added

- [closes #30445] Supporting ros 2 driver
- [closes #31485] Publish IMU data as ros IMU message
- [closes #31486] Convert IMU acceleration to transform message stamp and publish it as static TF transform

### Changed

- Target names and class names have blickfeld_driver as prefix

### Fixed

### Removed

- [closes #30445] Setting message timestamps

## [0.1.1] - 2021.07.26

### Added

### Changed

### Fixed

- [closes #31131] Fix the "pure virtual method called" message

### Removed

## [0.1.0] - 2021.06.25

### Added

- [closes #30443] add the ros1_ros2 common functionalities
- [closes #30566] On device algorithm, background subtraction and nearest neighbor filter
- [closes #30446] Set scan pattern
- [closes #30587] Create range, ambient and intesity image
