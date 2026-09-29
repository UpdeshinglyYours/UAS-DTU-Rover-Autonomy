/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Implementation for class BlickfeldDriverPointCloudParser
 */
#include <cmath>

#include <Eigen/Dense>

#include "blickfeld_driver_core/blickfeld_driver_point_cloud_parser.h"

namespace blickfeld {
namespace ros_interop {

BlickfeldDriverPointCloudParser::BlickfeldDriverPointCloudParser(const PointCloudOptions& point_cloud_options,
                                                                 const ImageOptions& image_options)
    : point_cloud_options_(point_cloud_options), point_size_(0), image_options_(image_options) {
  for (auto& cartesian_dimension : {"x", "y", "z"}) {
    point_fields_.push_back(createField(cartesian_dimension, point_size_, PointField::FLOAT32));
    point_size_ += sizeof(float);
  }

  if (point_cloud_options_.range == true) {
    point_fields_.push_back(createField("range", point_size_, PointField::FLOAT32));
    point_size_ += sizeof(float);
  }

  if (point_cloud_options_.angles == true) {
    point_fields_.push_back(createField("azimuth", point_size_, PointField::FLOAT32));
    point_size_ += sizeof(float);

    point_fields_.push_back(createField("elevation", point_size_, PointField::FLOAT32));
    point_size_ += sizeof(float);
  }

  if (point_cloud_options_.intensities == true) {
    point_fields_.push_back(createField("intensity", point_size_, PointField::UINT32));
    point_size_ += sizeof(uint32_t);
  }

  if (point_cloud_options_.ambient_light == true) {
    point_fields_.push_back(createField("ambient_light", point_size_, PointField::UINT32));
    point_size_ += sizeof(uint32_t);
  }

  if (point_cloud_options_.point_id == true) {
    point_fields_.push_back(createField("point_id", point_size_, PointField::UINT32));
    point_size_ += sizeof(uint32_t);
  }

  if (point_cloud_options_.point_time_offset == true) {
    point_fields_.push_back(createField("point_time_offset", point_size_, PointField::UINT32));
    point_size_ += sizeof(uint32_t);
  }

  if (point_cloud_options_.return_options == ReturnOptions::All) {
    point_fields_.push_back(createField("return_id", point_size_, PointField::UINT32));
    point_size_ += sizeof(uint32_t);
  }
}

FrameStreamOutput BlickfeldDriverPointCloudParser::parseFrame(const FrameT& frame, LogMessages& log_messages) {
  {
    const size_t dropped_frames = droppedFrames(frame.id());
    if (dropped_frames != 0) {
      log_messages[LogLevel::Warning] << "Dropped " << dropped_frames << " frame(s)!\n";
    }
  }

  bool publish_image = driver_utilities::isPublishImage(image_options_);
  /// HINT: #40318
  if (publish_image && driver_utilities::isImagePublishingSupported(frame, image_options_) == false) {
    publish_image = false;
    log_messages[LogLevel::Warning] << "Image publishing is not supported for this frame mode.\n";
  }
  cv::Size image_size(0, 0);
  std::pair<float, float> scan_pattern_vertical_limit, scan_pattern_horizontal_limit;
  if (publish_image == true) {
    image_size = driver_utilities::getImageSize(frame.scan_pattern(), image_options_.projection_type);
    resizeImageBuffer(image_size);
    /// HINT: we do not need to compute the scan pattern limits for every frame we need to compute it once when the
    /// scan pattern changed but since it does not affect the performance for now we kept it here
    scan_pattern_vertical_limit = driver_utilities::getScanPatternVerticalLimit(frame);
    scan_pattern_horizontal_limit = driver_utilities::getScanPatternHorizontalLimit(frame);
  }

  uint32_t number_of_returns;
  if (point_cloud_options_.no_return_points) {
    number_of_returns =
        frame.total_number_of_points() * frame.scan_pattern().filter().max_number_of_returns_per_point();
  } else {
    number_of_returns = frame.total_number_of_returns();
  }

  FrameStreamOutput frame_stream_output;
  initializeFrameStreamOutput(frame, number_of_returns, frame_stream_output);

  size_t point_index_counter = 0;
  for (int scanline_index = 0; scanline_index < frame.scanlines_size(); scanline_index++) {
    const auto& scanline = frame.scanlines(scanline_index);
    for (int point_index = 0; point_index < scanline.points_size(); point_index++) {
      const auto& point = scanline.points(point_index);
      const size_t number_of_returns = static_cast<size_t>(point.returns_size());

      if (publish_image == true)
        updateImageData(frame.scan_pattern(), point, image_size, scanline_index, scan_pattern_vertical_limit,
                        scan_pattern_horizontal_limit);

      if (number_of_returns > 0) {
        for (const PointReturnT& point_return : ReturnSelector(point.returns(), point_cloud_options_.return_options)) {
          assignField(frame_stream_output.point_cloud_msg, point_index_counter, "x", point_return.cartesian(0));
          assignField(frame_stream_output.point_cloud_msg, point_index_counter, "y", point_return.cartesian(1));
          assignField(frame_stream_output.point_cloud_msg, point_index_counter, "z", point_return.cartesian(2));

          if ((point_cloud_options_.range == true) && (point_return.has_range() == true)) {
            assignField(frame_stream_output.point_cloud_msg, point_index_counter, "range", point_return.range());
          }
          if ((point_cloud_options_.angles == true) && (point.has_direction() == true)) {
            assignField(frame_stream_output.point_cloud_msg, point_index_counter, "azimuth",
                        point.direction().azimuth());
            assignField(frame_stream_output.point_cloud_msg, point_index_counter, "elevation",
                        point.direction().elevation());
          }
          if ((point_cloud_options_.intensities == true) && (point_return.has_intensity() == true)) {
            assignField(frame_stream_output.point_cloud_msg, point_index_counter, "intensity",
                        point_return.intensity());
          }
          if ((point_cloud_options_.ambient_light == true) && (point.has_ambient_light_level() == true)) {
            assignField(frame_stream_output.point_cloud_msg, point_index_counter, "ambient_light",
                        point.ambient_light_level());
          }
          if ((point_cloud_options_.point_id == true) && point.has_id()) {
            /// Note that in case of multiple returns (i.e., returns_count > 1), this leads to non-unique IDs. This set
            /// of IDs only forms a primary key if the return_id is also considered.
            assignField(frame_stream_output.point_cloud_msg, point_index_counter, "point_id", point.id());
          }
          if ((point_cloud_options_.point_time_offset == true) && point.has_start_offset_ns()) {
            assert(point.start_offset_ns() <= UINT32_MAX);
            assignField<uint32_t>(frame_stream_output.point_cloud_msg, point_index_counter, "point_time_offset",
                                  point.start_offset_ns());
          }
          if (point_cloud_options_.return_options == ReturnOptions::All && point_return.has_id() == true) {
            assignField(frame_stream_output.point_cloud_msg, point_index_counter, "return_id", point_return.id());
          }
          point_index_counter++;
        }
      } else if ((point_cloud_options_.no_return_points == true) && (point.has_direction() == true)) {
        const auto& direction = point.direction();
        const float azimuth = direction.azimuth();
        const float elevation = direction.elevation();
        /// HINT: This will bring the dummy points to be in front of field of view
        const float x = point_cloud_options_.no_return_point_range * std::cos(elevation) *
                        static_cast<float>(std::cos(M_PI_2 - azimuth));
        const float y = point_cloud_options_.no_return_point_range * std::cos(elevation) *
                        static_cast<float>(std::sin(M_PI_2 - azimuth));
        const float z = point_cloud_options_.no_return_point_range * std::sin(elevation);

        assignField(frame_stream_output.point_cloud_msg, point_index_counter, "x", x);
        assignField(frame_stream_output.point_cloud_msg, point_index_counter, "y", y);
        assignField(frame_stream_output.point_cloud_msg, point_index_counter, "z", z);

        if (point_cloud_options_.angles == true) {
          assignField(frame_stream_output.point_cloud_msg, point_index_counter, "azimuth", azimuth);
          assignField(frame_stream_output.point_cloud_msg, point_index_counter, "elevation", elevation);
        }

        if (point_cloud_options_.intensities == true) {
          assignField(frame_stream_output.point_cloud_msg, point_index_counter, "intensity", 0.f);
        }

        if ((point_cloud_options_.ambient_light == true) && (point.has_ambient_light_level() == true)) {
          assignField(frame_stream_output.point_cloud_msg, point_index_counter, "ambient_light",
                      point.ambient_light_level());
        }

        if ((point_cloud_options_.point_id == true) && point.has_id()) {
          /// Note that in case of multiple returns (i.e., returns_count > 1), this leads to non-unique IDs. This set
          /// of IDs only forms a primary key if the return_id is also considered.
          assignField(frame_stream_output.point_cloud_msg, point_index_counter, "point_id", point.id());
        }

        if ((point_cloud_options_.point_time_offset == true) && point.has_start_offset_ns()) {
          assert(point.start_offset_ns() <= UINT32_MAX);
          assignField<uint32_t>(frame_stream_output.point_cloud_msg, point_index_counter, "point_time_offset",
                                point.start_offset_ns());
        }

        point_index_counter++;
      }
    }
  }

  if (publish_image == true) createRosImages(image_size, frame_stream_output);

  /// Truncate at the end
  frame_stream_output.point_cloud_msg->data.resize(frame_stream_output.point_cloud_msg->point_step *
                                                   point_index_counter);
  frame_stream_output.point_cloud_msg->is_dense = false;
  frame_stream_output.point_cloud_msg->height = 1;
  frame_stream_output.point_cloud_msg->width = static_cast<uint32_t>(point_index_counter);
  frame_stream_output.point_cloud_msg->row_step =
      frame_stream_output.point_cloud_msg->point_step * frame_stream_output.point_cloud_msg->width;

  return frame_stream_output;
}

DiagnosticStatusPtr BlickfeldDriverPointCloudParser::parseStatus(const blickfeld::protocol::data::Frame& frame,
                                                                 const std::string& host) {
  /// TODO: Ticket #30528
  auto scan_pattern = frame.scan_pattern();
  std::unordered_map<std::string, std::string> diagnostic_status;
  diagnostic_status["scan_pattern/horizontal/fov"] = std::to_string(scan_pattern.horizontal().fov() / M_PI * 180.f);
  diagnostic_status["scan_pattern/vertical/fov"] = std::to_string(scan_pattern.vertical().fov() / M_PI * 180.f);
  diagnostic_status["scan_pattern/vertical/scanlines_up"] = std::to_string(scan_pattern.vertical().scanlines_up());
  diagnostic_status["scan_pattern/vertical/scanlines_down"] = std::to_string(scan_pattern.vertical().scanlines_down());
  diagnostic_status["scan_pattern/pulse/frame_mode"] =
      scan_pattern.pulse().FrameMode_Name(scan_pattern.pulse().frame_mode()).c_str();
  diagnostic_status["scan_pattern/pulse/angle_spacing"] =
      std::to_string(scan_pattern.pulse().angle_spacing() / M_PI * 180.f);
  diagnostic_status["scan_pattern/frame_rate/target"] = std::to_string(scan_pattern.frame_rate().target());
  diagnostic_status["scan_pattern/frame_rate/maximum"] = std::to_string(scan_pattern.frame_rate().maximum());

  DiagnosticStatusPtr diagnostic_status_msg(new DiagnosticStatus);
  for (const auto& diagnostic : diagnostic_status) {
    DiagnosticKeyValue message;
    message.key = diagnostic.first;
    message.value = diagnostic.second;
    diagnostic_status_msg->values.push_back(message);
  }
  diagnostic_status_msg->name = host;
  diagnostic_status_msg->hardware_id = host;
  return diagnostic_status_msg;
}

PointCloudOptions BlickfeldDriverPointCloudParser::getPointCloudOptions() const { return point_cloud_options_; }

ImageOptions BlickfeldDriverPointCloudParser::getImageOptions() const { return image_options_; }

PointField BlickfeldDriverPointCloudParser::createField(const std::string& name, uint32_t offset, uint8_t datatype) {
  PointField field;
  field.name = name;
  field.count = 1;
  field.datatype = datatype;
  field.offset = offset;
  return field;
}

void BlickfeldDriverPointCloudParser::initializeFrameStreamOutput(const FrameT& frame, const size_t number_of_returns,
                                                                  FrameStreamOutput& frame_stream_output) {
  frame_stream_output.frame_time = frame.start_time_ns();
  frame_stream_output.point_cloud_msg = PointCloud2Ptr(new PointCloud2);
  frame_stream_output.point_cloud_msg->header.frame_id = point_cloud_options_.lidar_frame_id;
  frame_stream_output.point_cloud_msg->fields = point_fields_;
  frame_stream_output.point_cloud_msg->point_step = point_size_;
  frame_stream_output.point_cloud_msg->data.resize(frame_stream_output.point_cloud_msg->point_step * number_of_returns);
}

void BlickfeldDriverPointCloudParser::resizeImageBuffer(const cv::Size& image_size) {
  const size_t total_pixel = image_size.height * image_size.width;

  if (image_options_.range_image == true) range_image_sorted_pixels_.assign(total_pixel, 0);
  if (image_options_.ambient_image == true) ambient_image_sorted_pixels_.assign(total_pixel, 0);
  if (image_options_.intensity_image == true) intensity_image_sorted_pixels_.assign(total_pixel, 0);
  if (image_options_.point_id_image == true) point_id_image_sorted_pixels_.assign(total_pixel, -1);
}

void BlickfeldDriverPointCloudParser::updateImageData(const ScanPatternT scan_pattern, const PointT& point,
                                                      const cv::Size& image_size, size_t scanline_index,
                                                      const std::pair<float, float>& scan_pattern_vertical_limit,
                                                      const std::pair<float, float>& scan_pattern_horizontal_limit) {
  if (point.has_id() == false) return;
  int flat_pixel = 0;

  switch (image_options_.projection_type) {
    case ProjectionType::ScanlinePreserving:
      flat_pixel = driver_utilities::getFlatPixelScanlineBased(point.id(), scanline_index, scan_pattern, image_size);
      break;
    case ProjectionType::AnglePreserving:
      flat_pixel = driver_utilities::getFlatPixelAngleBased(point, scan_pattern, scan_pattern_vertical_limit,
                                                            scan_pattern_horizontal_limit, image_size);
      break;
    default:
      throw std::invalid_argument("Unknown Image projection type");
  }
  /// flat_pixel -1 is an invalid pixel
  if (flat_pixel == -1) return;

  if (image_options_.point_id_image == true) {
    point_id_image_sorted_pixels_[flat_pixel] = point.id();
  }
  if (image_options_.ambient_image == true && point.has_ambient_light_level() == true) {
    ambient_image_sorted_pixels_[flat_pixel] = point.ambient_light_level();
  }
  if ((image_options_.range_image == true || image_options_.intensity_image == true) &&
      static_cast<size_t>(point.returns_size()) > 0) {
    /// HINT: Range image and intensity image is created based on the strongest return for two reasons: 1) easier to
    /// interpret 2) strongest return is most of the time the return that customers are interested in
    const PointReturnT& point_return = *ReturnSelector(point.returns(), ReturnOptions::Strongest).begin();
    if (image_options_.range_image == true && point_return.has_range() == true) {
      range_image_sorted_pixels_[flat_pixel] = point_return.range();
    }
    if (image_options_.intensity_image == true && point_return.has_intensity() == true) {
      intensity_image_sorted_pixels_[flat_pixel] = point_return.intensity();
    }
  }
}

void BlickfeldDriverPointCloudParser::createRosImages(const cv::Size& image_size,
                                                      FrameStreamOutput& frame_stream_output) {
  if (image_options_.range_image == true) {
    frame_stream_output.range_image = driver_utilities::createRosImageMessage(
        range_image_sorted_pixels_, image_size, point_cloud_options_.lidar_frame_id, image_encoding_32_bit_float_);
  }
  if (image_options_.intensity_image == true) {
    frame_stream_output.intensity_image = driver_utilities::createRosImageMessage(
        intensity_image_sorted_pixels_, image_size, point_cloud_options_.lidar_frame_id, image_encoding_32_bit_float_);
  }
  if (image_options_.ambient_image == true) {
    frame_stream_output.ambient_image = driver_utilities::createRosImageMessage(
        ambient_image_sorted_pixels_, image_size, point_cloud_options_.lidar_frame_id, image_encoding_32_bit_float_);
  }

  if (image_options_.point_id_image == true) {
    frame_stream_output.point_id_image = driver_utilities::createRosImageMessage(
        point_id_image_sorted_pixels_, image_size, point_cloud_options_.lidar_frame_id, image_encoding_32_bit_int_);
  }
}

size_t BlickfeldDriverPointCloudParser::droppedFrames(const size_t current_frame_index) {
  size_t dropped_frames = 0;
  if (last_frame_index_ != 0) dropped_frames = current_frame_index - last_frame_index_ - 1;
  last_frame_index_ = current_frame_index;
  return dropped_frames;
}

ReturnSelector::ReturnSelector(const PointReturnsT& returns, const ReturnOptions& return_options) {
  auto range_comparison = [](const PointReturnT& point1, const PointReturnT& point2) {
    if (point1.has_range() && point2.has_range()) {
      return point1.range() < point2.range();
    }
    assert("Returned points did not have range!");
    return false;
  };
  switch (return_options) {
    case ReturnOptions::Strongest:
      current_iterator = returns.begin();
      end_iterator = returns.begin() + 1;
      break;
    case ReturnOptions::All:
      current_iterator = returns.begin();
      end_iterator = returns.end();
      break;
    case ReturnOptions::Closest:
      current_iterator = std::min_element(returns.begin(), returns.end(), range_comparison);
      end_iterator = current_iterator + 1;
      break;
    case ReturnOptions::Farthest:
      current_iterator = std::max_element(returns.begin(), returns.end(), range_comparison);
      end_iterator = current_iterator + 1;
      break;
    default:
      throw std::invalid_argument("Unknown Return Option!");
      break;
  }
}

const ReturnSelector& ReturnSelector::begin() const { return *this; }

const ReturnSelector& ReturnSelector::end() const { return *this; }

bool ReturnSelector::operator!=(const ReturnSelector&) const { return current_iterator != end_iterator; }

void ReturnSelector::operator++() { current_iterator++; }

PointReturnT ReturnSelector::operator*() const { return *current_iterator; }

}  // namespace ros_interop
}  // namespace blickfeld
