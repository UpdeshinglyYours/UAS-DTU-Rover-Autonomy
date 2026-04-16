/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Implementation for Blickfeld Driver Utilities
 */

#include <math.h>

#include <cv_bridge/cv_bridge.h>

#include "blickfeld_driver_core/blickfeld_driver_utils.h"

namespace blickfeld {
namespace ros_interop {
namespace driver_utilities {

template <typename T>
SensorMsgImagePtr createRosImageMessage(std::vector<T> &image_data, const cv::Size &image_size,
                                        const std::string &frame_id, const std::string &encoding) {
  SensorMsgImage ros_image;
  MsgHeader header;
  header.frame_id = frame_id;
  const cv::Mat &cv_image = cv::Mat(image_size, cv_bridge::getCvType(encoding), image_data.data());
  cv_bridge::CvImage(header, encoding, cv_image).toImageMsg(ros_image);
  return SensorMsgImagePtr(new SensorMsgImage(ros_image));
}
template SensorMsgImagePtr createRosImageMessage<float>(std::vector<float> &image_data, const cv::Size &image_size,
                                                        const std::string &frame_id, const std::string &encoding);
template SensorMsgImagePtr createRosImageMessage<int32_t>(std::vector<int32_t> &image_data, const cv::Size &image_size,
                                                          const std::string &frame_id, const std::string &encoding);

int getFlatPixelScanlineBased(uint32_t point_id, int scanline_index, const ScanPatternT &scan_pattern,
                              const cv::Size &image_size) {
  size_t row = 0;
  const size_t vertical_center = std::floor(image_size.height / 2);
  const FrameModeT &frame_mode = scan_pattern.pulse().frame_mode();
  if (frame_mode == FrameModeT::ScanPattern_Pulse_FrameMode_ONLY_UP)
    ///  if even scanline: row =  vertical_center + (scanline_index / 2)
    ///  if odd scanline: row =  vertical_center - (scanline_index / 2 + scanline_index % 2)
    row = scanline_index % 2 == 0 ? vertical_center + (scanline_index / 2) : vertical_center - (scanline_index / 2) - 1;
  else if (frame_mode == FrameModeT::ScanPattern_Pulse_FrameMode_ONLY_DOWN)
    /// if even scanline: row = vertical_center + (vertical_center - scanline_index / 2) - 1
    /// if odd scanline: row = vertical_center - (vertical_center - (scanline_index / 2 + scanline_index % 2)) - 1;
    row = scanline_index % 2 == 0 ? (2 * vertical_center) - (scanline_index / 2) - 1 : scanline_index / 2;
  else
    return -1;

  /// Rotate the image only for visualization purpose
  row = image_size.height - 1 - row;

  const size_t unordered_image_column = point_id % image_size.width;
  const size_t column =
      scanline_index % 2 != 0 ? unordered_image_column : image_size.width - unordered_image_column - 1;

  return row * image_size.width + column;
}

int getFlatPixelAngleBased(const PointT &point, const ScanPatternT &scan_pattern,
                           std::pair<float, float> vertical_limit, std::pair<float, float> horizontal_limit,
                           const cv::Size &image_size) {
  const auto &direction = point.direction();
  const float x_coordinate = direction.azimuth() / static_cast<float>(scan_pattern.pulse().angle_spacing());
  const float y_coordinate =
      direction.elevation() / static_cast<float>(scan_pattern.vertical().fov()) / getNumberOfScanlines(scan_pattern);
  /// HINT: we have to check if coordinates are inside the image limit so we do not extra calculate the pixels
  const bool valid_x_coordinate = horizontal_limit.first <= x_coordinate && x_coordinate <= horizontal_limit.second;
  const bool valid_y_coordinate = vertical_limit.first <= y_coordinate && y_coordinate <= vertical_limit.second;
  if (valid_x_coordinate == true && valid_y_coordinate == true) {
    int x_pixel = std::floor(((x_coordinate - horizontal_limit.first) * (image_size.width)) /
                             ((horizontal_limit.second - horizontal_limit.first)));
    int y_pixel = std::floor(((y_coordinate - vertical_limit.first) * (image_size.height)) /
                             ((vertical_limit.second - vertical_limit.first)));
    /// HINT: actual image size is smaller than the image limits because we do not want to have holes inside our image
    const bool valid_x_pixel = 0 <= x_pixel && x_pixel < image_size.width;
    const bool valid_y_pixel = 0 <= y_pixel && y_pixel < image_size.height;
    /// HINT: Rotate the image only for visualization purpose
    y_pixel = image_size.height - 1 - y_pixel;
    if (valid_x_pixel == true && valid_y_pixel == true)
      return y_pixel * image_size.width + x_pixel;
    else
      return -1;
  } else
    return -1;
}

size_t getNumberOfScanlines(const ScanPatternT &scan_pattern) {
  const auto &frame_mode = scan_pattern.pulse().frame_mode();
  switch (frame_mode) {
    case FrameModeT::ScanPattern_Pulse_FrameMode_ONLY_UP:
      return scan_pattern.vertical().scanlines_up();
    case FrameModeT::ScanPattern_Pulse_FrameMode_ONLY_DOWN:
      return scan_pattern.vertical().scanlines_down();
    case FrameModeT::ScanPattern_Pulse_FrameMode_COMBINE_UP_DOWN:
      return scan_pattern.vertical().scanlines_up() + scan_pattern.vertical().scanlines_down();
    default:
      throw std::invalid_argument("Cannot compute a 2D image for this frame mode!");
  }
}

cv::Size getImageSize(const ScanPatternT &scan_pattern, const ProjectionType &projection_type) {
  const float horizontal_fov = static_cast<float>(scan_pattern.horizontal().fov());
  const float horizontal_resolution = static_cast<float>(scan_pattern.pulse().angle_spacing());
  const size_t maximum_number_of_points = static_cast<size_t>(std::floor(horizontal_fov / horizontal_resolution)) + 1;

  const size_t number_of_scanlines = getNumberOfScanlines(scan_pattern);

  if (projection_type == ProjectionType::AnglePreserving) {
    const float vertical_fov = static_cast<float>(scan_pattern.vertical().fov());
    const float vertical_resolution = vertical_fov / static_cast<float>(number_of_scanlines);

    size_t width_base_on_height = horizontal_fov / vertical_resolution;
    size_t height_base_on_width = vertical_fov / horizontal_resolution;
    if (vertical_resolution < horizontal_resolution)
      return cv::Size(maximum_number_of_points, height_base_on_width);
    else
      return cv::Size(width_base_on_height, number_of_scanlines);
  } else {
    return cv::Size(maximum_number_of_points, number_of_scanlines);
  }
}

std::pair<float, float> getScanPatternHorizontalLimit(const FrameT &frame) {
  const ScanPatternT &scan_pattern = frame.scan_pattern();
  const auto &frame_mode = scan_pattern.pulse().frame_mode();
  ScanlineT scanline;
  switch (frame_mode) {
    case FrameModeT::ScanPattern_Pulse_FrameMode_ONLY_UP:
    case FrameModeT::ScanPattern_Pulse_FrameMode_COMBINE_UP_DOWN:
      scanline = frame.scanlines(0);
      break;
    case FrameModeT::ScanPattern_Pulse_FrameMode_ONLY_DOWN:
      /// HINT: to match the order of point id increasing from up only (left to right or right to left)
      if (scan_pattern.vertical().scanlines_up() % 2 == 0)
        scanline = frame.scanlines(0);
      else
        scanline = frame.scanlines(1);
      break;
    default:
      throw std::invalid_argument("Cannot compute a 2D image for this frame mode!");
  }

  const auto angle_spacing = static_cast<float>(scan_pattern.pulse().angle_spacing());
  const float maximum = scanline.points(0).direction().azimuth() / angle_spacing;
  const size_t last_point_index = scanline.points_size() - 1;
  const float minimum = scanline.points(last_point_index).direction().azimuth() / angle_spacing;
  return std::make_pair(minimum, maximum);
}

std::pair<float, float> getScanPatternVerticalLimit(const FrameT &frame) {
  const ScanPatternT &scan_pattern = frame.scan_pattern();
  const auto &frame_mode = scan_pattern.pulse().frame_mode();
  const size_t last_scanline_index = frame.scanlines_size() - 1;
  size_t top_scanline_index, bottom_scanline_index;
  switch (frame_mode) {
    case FrameModeT::ScanPattern_Pulse_FrameMode_ONLY_UP:
      if (last_scanline_index % 2 == 0) {
        top_scanline_index = last_scanline_index;
        bottom_scanline_index = last_scanline_index - 1;
      } else {
        top_scanline_index = last_scanline_index - 1;
        bottom_scanline_index = last_scanline_index;
      }
      break;
    case FrameModeT::ScanPattern_Pulse_FrameMode_COMBINE_UP_DOWN:
      if (scan_pattern.vertical().scanlines_up() % 2 == 0) {
        top_scanline_index = scan_pattern.vertical().scanlines_up();
        bottom_scanline_index = scan_pattern.vertical().scanlines_up() - 1;
      } else {
        top_scanline_index = scan_pattern.vertical().scanlines_up() - 1;
        bottom_scanline_index = scan_pattern.vertical().scanlines_up();
      }
      break;
    case FrameModeT::ScanPattern_Pulse_FrameMode_ONLY_DOWN:
      if (scan_pattern.vertical().scanlines_up() % 2 == 0) {
        top_scanline_index = 0;
        bottom_scanline_index = 1;
      } else {
        top_scanline_index = 1;
        bottom_scanline_index = 0;
      }
      break;
    default:
      throw std::invalid_argument("Cannot compute a 2D image for this frame mode!");
  }

  const auto &top_scanline = frame.scanlines(top_scanline_index);
  size_t center_point_index = top_scanline.points_size() / 2;
  const auto vertical_fov = static_cast<float>(scan_pattern.vertical().fov());
  /// HINT: vertical angle spacing = vertical_fov / getNumberOfScanlines(scan_pattern)
  const float maximum = top_scanline.points(center_point_index).direction().elevation() / vertical_fov /
                        getNumberOfScanlines(scan_pattern);

  const auto &bottom_scanline = frame.scanlines(bottom_scanline_index);
  center_point_index = bottom_scanline.points_size() / 2;
  const float minimum = bottom_scanline.points(center_point_index).direction().elevation() / vertical_fov /
                        getNumberOfScanlines(scan_pattern);

  return std::make_pair(minimum, maximum);
}

ImuAccelerationUnit imuAccelerationUnitFromString(const std::string &imu_acceleration_unit_string) {
  if (imu_acceleration_unit_string.find("g") != std::string::npos) {
    return ImuAccelerationUnit::G;
  } else if (imu_acceleration_unit_string.find("meters_per_second_squared") != std::string::npos) {
    return ImuAccelerationUnit::MetersPerSecondSquared;
  } else {
    throw std::invalid_argument("imu acceleration unit unknown: " + imu_acceleration_unit_string);
  }
}

std::string imuAccelerationUnitToString(const ImuAccelerationUnit &imu_acceleration_unit) {
  switch (imu_acceleration_unit) {
    case ImuAccelerationUnit::G:
      return "g";
    case ImuAccelerationUnit::MetersPerSecondSquared:
      return "meters_per_second_squared";
    default:
      throw std::invalid_argument("imu acceleration unit unknown");
  }
}

ProjectionType projectionTypeFromString(const std::string &projection_type_string) {
  if (projection_type_string.find("angle_preserving") != std::string::npos) {
    return ProjectionType::AnglePreserving;
  } else if (projection_type_string == "scanline_preserving") {
    return ProjectionType::ScanlinePreserving;
  } else {
    throw std::invalid_argument("ProjectionType unknown: " + projection_type_string);
  }
}

std::string projectionTypeToString(const ProjectionType &projection_type) {
  switch (projection_type) {
    case ProjectionType::AnglePreserving:
      return "angle_preserving";
    case ProjectionType::ScanlinePreserving:
      return "scanline_preserving";
    default:
      throw std::invalid_argument("ProjectionType unknown");
  }
}

bool isImagePublishingSupported(const FrameT &frame, const ImageOptions &image_options) {
  /// HINT: #40318
  FrameModeT frame_mode = frame.scan_pattern().pulse().frame_mode();
  if (frame_mode == FrameModeT::ScanPattern_Pulse_FrameMode_SEPARATE ||
      (frame_mode == FrameModeT::ScanPattern_Pulse_FrameMode_COMBINE_UP_DOWN &&
       image_options.projection_type == ProjectionType::ScanlinePreserving))
    return false;
  else
    return true;
}

bool isPublishImage(const ImageOptions &image_options) {
  return image_options.range_image || image_options.ambient_image || image_options.intensity_image ||
         image_options.point_id_image;
}

ReturnOptions returnOptionsFromString(const std::string &return_option_string) {
  if (return_option_string.find("strongest") != std::string::npos) {
    return ReturnOptions::Strongest;
  } else if (return_option_string.find("closest") != std::string::npos) {
    return ReturnOptions::Closest;
  } else if (return_option_string.find("farthest") != std::string::npos) {
    return ReturnOptions::Farthest;
  } else if (return_option_string.find("all") != std::string::npos) {
    return ReturnOptions::All;
  } else {
    throw std::invalid_argument("return options unknown: " + return_option_string);
  }
}

std::string returnOptionsToString(const ReturnOptions &return_options) {
  switch (return_options) {
    case ReturnOptions::Strongest:
      return "strongest";
    case ReturnOptions::Closest:
      return "closest";
    case ReturnOptions::Farthest:
      return "farthest";
    case ReturnOptions::All:
      return "all";
    default:
      throw std::invalid_argument("returns option unknown");
  }
}
}  // namespace driver_utilities
}  // namespace ros_interop
}  // namespace blickfeld
