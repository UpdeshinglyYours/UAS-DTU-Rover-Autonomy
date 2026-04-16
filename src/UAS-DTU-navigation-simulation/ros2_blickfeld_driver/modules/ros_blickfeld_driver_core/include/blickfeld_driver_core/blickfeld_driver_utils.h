/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Header for Blickfeld Driver Utilities
 */

#pragma once

#include <string>

#include <Eigen/Dense>

#include <blickfeld/scanner.h>

#include "blickfeld_driver/blickfeld_driver_ros_types.h"
#include "blickfeld_driver_core/blickfeld_driver_types.h"

namespace blickfeld {
namespace ros_interop {
namespace driver_utilities {

/**
 * @brief createRosImageMessage creates a ros image message from image data
 *
 * @tparam T image type supported type int32 and float
 * @param[in] image_data
 * @param[in] image_size
 * @param[in] frame_id
 * @param[in] encoding image encoding
 *
 * @return SensorMsgImagePtr
 */
template <typename T>
SensorMsgImagePtr createRosImageMessage(std::vector<T> &image_data, const cv::Size &image_size,
                                        const std::string &frame_id, const std::string &encoding);
/**
 * @brief getFlatPixelScanlineBased computes the pixel coordinate of a point in a scan pattern. The pixel
 * coordinate is in flatten row major order. This method can be used to create a picture of scan pattern in a sense
 * that each scanline is a row of an image
 *
 * @param[in] point_id
 * @param[in] scanline_index
 * @param[in] scan_pattern
 * @param[in] image_size
 *
 * @return int flat pixel or -1 in case the point is not in range
 */
int getFlatPixelScanlineBased(uint32_t point_id, int scanline_index, const ScanPatternT &scan_pattern,
                              const cv::Size &image_size);

/**
 * @brief getFlatPixelAngleBased computes the pixel coordinate of a projected point in a scan pattern based on
 * ray angles. The pixel coordinate is in flatten row major order. This method can be used to projects the points to
 * create an angle preserving picture.
 *
 * @param[in] point
 * @param[in] scan_pattern
 * @param[in] vertical_limit maximum and minimum vertically
 * @param[in] horizontal_limit maximum and minimum horizontally
 * @param[in] image_size
 *
 * @return int flat pixel or -1 in case the point is not in range
 */
int getFlatPixelAngleBased(const PointT &point, const ScanPatternT &scan_pattern,
                           std::pair<float, float> vertical_limit, std::pair<float, float> horizontal_limit,
                           const cv::Size &image_size);

/**
 * @brief getNumberOfScanlines returns number of scanlines of a scan pattern
 *
 * @param[in] scan_pattern
 * @return number of scanlines
 */
size_t getNumberOfScanlines(const ScanPatternT &scan_pattern);

/**
 * @brief getImageSize
 *
 * @param[in] scan_pattern
 * @param[in] projection_type
 * @return size of the image
 */
cv::Size getImageSize(const ScanPatternT &scan_pattern, const ProjectionType &projection_type);

/**
 * @brief getScanPatternHorizontalLimit gets the scan pattern 2D horizontal limit by projecting scan pattern on a 2D
 * plane
 *
 * @param[in] frame
 * @return std::pair<float, float> pair of minimum and maximum
 */
std::pair<float, float> getScanPatternHorizontalLimit(const FrameT &frame);

/**
 * @brief getScanPatternVerticalLimit gets the scan pattern 2D vertical limit by projecting scan pattern on a 2D plane
 *
 * @param[in] frame
 * @return std::pair<float, float> pair of minimum and maximum
 */
std::pair<float, float> getScanPatternVerticalLimit(const FrameT &frame);

/**
 * @brief imuAccelerationUnitFromString converts @p imu_acceleration_unit_string to ImuAccelerationUnit, it throws an
 * exception in case of undefined string
 *
 * @param[in] imu_acceleration_unit_string
 * @return ImuAccelerationUnit
 */
ImuAccelerationUnit imuAccelerationUnitFromString(const std::string &imu_acceleration_unit_string);

/**
 * @brief imuAccelerationUnitToString converts @p imu_acceleration_unit to string, it throws an exception in case
 * of undefined unit
 *
 * @param[in] imu_acceleration_unit
 * @return std::string
 */
std::string imuAccelerationUnitToString(const ImuAccelerationUnit &imu_acceleration_unit);

/**
 * @brief isImagePublishingSupported returns false if the image projection type is scanline preserving and the frame
 * mode is not supported for that otherwise returns true
 *
 * @param[in] frame
 * @param[in] image_options
 * @return bool
 */
bool isImagePublishingSupported(const FrameT &frame, const ImageOptions &image_options);

/**
 * @brief isPublishImage checks the @p image_options and return true in case at least one of the image publishing
 * options is true
 *
 * @param[in] image_options
 * @return true in case at least one of the image publishing options is true
 */
bool isPublishImage(const ImageOptions &image_options);

/**
 * @brief projectionTypeFromString converts @p projection_type_string to a ProjectionType, it would throw an exception
 * in case of undefined string
 *
 * @param[in] projection_type_string
 * @return projection_type
 */
ProjectionType projectionTypeFromString(const std::string &projection_type_string);

/**
 * @brief projectionTypeToString converts @p projection_type to a string, it would throw an exception in case of
 * undefined projection_type
 *
 * @param[in] projection_type
 * @return std::string
 */
std::string projectionTypeToString(const ProjectionType &projection_type);

/**
 * @brief returnOptionsFromString converts @p string to a ReturnOptions, it would throw an exception in case
 * of undefined string
 *
 * @param[in] string
 * @return return options
 */
ReturnOptions returnOptionsFromString(const std::string &string);

/**
 * @brief returnOptionsToString converts @p return_options to a string, it would throw an exception in case of
 * undefined projection_type
 *
 * @param[in] return_options
 * @return std::string
 */
std::string returnOptionsToString(const ReturnOptions &return_options);

}  // namespace driver_utilities
}  // namespace ros_interop
}  // namespace blickfeld
