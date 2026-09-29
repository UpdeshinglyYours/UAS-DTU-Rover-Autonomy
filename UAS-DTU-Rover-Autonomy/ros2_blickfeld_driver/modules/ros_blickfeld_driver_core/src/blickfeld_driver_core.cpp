/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Implementation for class BlickfeldDriverCore
 */

#include <chrono>
#include <numeric>
#include <ratio>
#include <stdexcept>

#include "blickfeld_driver_core/blickfeld_driver_core.h"

namespace blickfeld {
namespace ros_interop {

BlickfeldDriverCore::~BlickfeldDriverCore() {
  point_cloud_stream_.reset();
  imu_stream_.reset();
  std::unique_lock<std::mutex> lock(point_cloud_scanner_mutex_);
  point_cloud_scanner_.reset();
  lock.unlock();
  imu_scanner_.reset();
}

void BlickfeldDriverCore::init(const std::string& host, const PointCloudOptions& point_cloud_options,
                               const DeviceAlgorithmOptions& device_algorithm_options,
                               const ImageOptions& image_options, const ImuOptions& imu_options) {
  host_ = host;
  point_cloud_parser_ = BlickfeldDriverPointCloudParser(point_cloud_options, image_options);
  imu_parser_ = BlickfeldDriverImuParser(imu_options.imu_acceleration_unit, imu_options.lidar_frame_id);
  imu_options_ = imu_options;
  setupOnDeviceAlgorithms(device_algorithm_options);
}

void BlickfeldDriverCore::start() {
  /// HINT: Taking the thread initialization out of the constructor avoids race conditions
  is_running_ = true;
  point_cloud_grabber_thread_ = std::thread(&BlickfeldDriverCore::pointCloudSpinner, this);
  if (imu_options_.imu_stream == true) {
    imu_grabber_thread_ = std::thread(&BlickfeldDriverCore::imuSpinner, this);
  }
  if (imu_options_.imu_static_tf == true) {
    publishImuStaticTF(*getStaticTF());
  }
}

void BlickfeldDriverCore::stop() {
  /// HINT: Taking the thread destruction out of the destructor to avoid race conditions
  is_running_ = false;
  if (point_cloud_grabber_thread_.joinable() == true) {
    point_cloud_grabber_thread_.join();
  }
  if (imu_grabber_thread_.joinable() == true) {
    imu_grabber_thread_.join();
  }
}

bool BlickfeldDriverCore::isRunning() const { return is_running_; }

void BlickfeldDriverCore::setupOnDeviceAlgorithms(const DeviceAlgorithmOptions& device_algorithm_options) {
  /// enable on-device algorithms for point cloud stream
  if (device_algorithm_options.use_background_subtraction == true) {
    auto background_subtraction = subscription_.add_algorithms()->mutable_background_subtraction();
    background_subtraction->set_initialization_frames(
        device_algorithm_options.background_subtraction_num_initialization_frames);
    background_subtraction->set_exponential_decay_rate(
        device_algorithm_options.background_subtraction_exponential_decay_rate);
  }
  if (device_algorithm_options.use_neighbor_filter == true) {
    subscription_.add_algorithms()->mutable_neighbor_filter();
  }
}

SetScanPatternResponse BlickfeldDriverCore::setScanPattern(const SetScanPatternRequest& service_request) {
  SetScanPatternResponse service_response;
  blickfeld::protocol::config::ScanPattern scan_pattern;

  scan_pattern.mutable_pulse()->set_angle_spacing(service_request.angle_spacing / 180.f * M_PI);
  scan_pattern.mutable_horizontal()->set_fov(service_request.fov_horizontal / 180.f * M_PI);
  scan_pattern.mutable_vertical()->set_fov(service_request.fov_vertical / 180.f * M_PI);
  scan_pattern.mutable_vertical()->set_scanlines_up(service_request.scanlines_up);
  scan_pattern.mutable_vertical()->set_scanlines_down(service_request.scanlines_down);

  FrameModeT requested_frame_mode;
  std::string requested_frame_mode_string = service_request.frame_mode;
  if (scan_pattern.pulse().FrameMode_Parse(requested_frame_mode_string, &requested_frame_mode) == true) {
    scan_pattern.mutable_pulse()->set_frame_mode(requested_frame_mode);
  } else {
    service_response.success = false;
    service_response.message = "Unknown frame mode.";
    return service_response;
  }

  blickfeld::protocol::config::ScanPattern::Pulse::Type requested_type;
  std::string requested_pulse_type_string = service_request.pulse_type;
  if (scan_pattern.pulse().Type_Parse(requested_pulse_type_string, &requested_type) == true) {
    scan_pattern.mutable_pulse()->set_type(requested_type);
  } else {
    service_response.success = false;
    service_response.message = "Unknown pulse type.";
    return service_response;
  }

  try {
    std::unique_lock<std::mutex> lock(point_cloud_scanner_mutex_);
    scan_pattern = point_cloud_scanner_->fill_scan_pattern(scan_pattern);
    point_cloud_scanner_->set_scan_pattern(scan_pattern);
    lock.unlock();

  } catch (const std::exception& e) {
    service_response.success = false;
    service_response.message = e.what();
    return service_response;
  }

  service_response.success = true;
  service_response.message = "Ok.";
  return service_response;
}

TransformMsgStampedPtr BlickfeldDriverCore::getStaticTF() {
  std::shared_ptr<blickfeld::scanner> scanner;
  while (scanner == nullptr) {
    try {
      scanner = blickfeld::scanner::connect(host_);
    } catch (std::exception const& e) {
      std::cerr << "Connecting to scanner yields: " << e.what() << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  return imu_parser_.parseImuStatic(scanner->get_status().imu().static_state().acceleration());
}

void BlickfeldDriverCore::pointCloudSpinner() {
  DebuggingTimes debugging_times;
  while (isRunning() == true) {
    if (isPointCloudStreaming() == false) {
      createPointCloudStream();
    } else {
      try {
        debugging_times.frame_receive_start_time = std::chrono::high_resolution_clock::now();
        const blickfeld::protocol::data::Frame& frame = point_cloud_stream_->recv_frame();
        debugging_times.frame_receive_end_time = std::chrono::high_resolution_clock::now();

        /// HINT: in case of any performance issue we can allocate memory at the start and update it with every frame
        /// (apply to images and point cloud)
        FrameStreamOutput frame_output = point_cloud_parser_.parseFrame(frame, log_messages_);
        debugging_times.parse_frame_end_time = std::chrono::high_resolution_clock::now();

        DiagnosticStatusPtr status = point_cloud_parser_.parseStatus(frame, host_);
        debugging_times.parse_status_end_time = std::chrono::high_resolution_clock::now();

        addDebugMessage(frame, debugging_times);

        publishPointCloud(frame_output.point_cloud_msg, frame_output.frame_time);
        publishStatus(status);
        if (driver_utilities::isPublishImage(point_cloud_parser_.getImageOptions()) == true &&
            driver_utilities::isImagePublishingSupported(frame, point_cloud_parser_.getImageOptions()) == true) {
          if (point_cloud_parser_.getImageOptions().range_image == true) {
            publishRangeImage(frame_output.range_image, frame_output.frame_time);
          }
          if (point_cloud_parser_.getImageOptions().intensity_image == true) {
            publishIntensityImage(frame_output.intensity_image, frame_output.frame_time);
          }
          if (point_cloud_parser_.getImageOptions().ambient_image == true) {
            publishAmbientImage(frame_output.ambient_image, frame_output.frame_time);
          }
          if (point_cloud_parser_.getImageOptions().point_id_image == true) {
            publishPointIdImage(frame_output.point_id_image, frame_output.frame_time);
          }
        }

        printLogMessages(log_messages_);
        log_messages_.clear();
      } catch (std::exception const& e) {
        std::cerr << "Exception while receiving point cloud frame: " << e.what() << std::endl;
        point_cloud_stream_.reset();
        std::unique_lock<std::mutex> lock(point_cloud_scanner_mutex_);
        point_cloud_scanner_.reset();
        lock.unlock();
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
  }
}

void BlickfeldDriverCore::createPointCloudStream() {
  /// if connected, disconnect
  point_cloud_stream_.reset();
  /// HINT: we need the mutex because set scan pattern also uses this connection
  std::unique_lock<std::mutex> lock(point_cloud_scanner_mutex_);
  point_cloud_scanner_.reset();
  lock.unlock();
  try {
    std::unique_lock<std::mutex> lock(point_cloud_scanner_mutex_);
    point_cloud_scanner_ = blickfeld::scanner::connect(host_);
    lock.unlock();
    point_cloud_stream_ = point_cloud_scanner_->get_point_cloud_stream(subscription_);
  } catch (const std::exception& e) {
    std::cerr << "Connecting to scanner yields: " << e.what() << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void BlickfeldDriverCore::imuSpinner() {
  while (isRunning() == true) {
    if (isImuStreaming() == false) {
      createImuStream();
    } else {
      try {
        std::vector<std::pair<SensorMsgImuPtr, time_t>> imu_messages;
        blickfeld::protocol::data::IMU imu_data = imu_stream_->recv_burst();
        imu_messages = imu_parser_.parseImuSamples(imu_data);

        for (size_t i = 0; i < imu_messages.size(); ++i) {
          publishImu(imu_messages[i].first, imu_messages[i].second);
        }
      } catch (std::exception const& e) {
        std::cerr << "Exception while receiving imu stream: " << e.what() << std::endl;
        imu_stream_.reset();
        imu_scanner_.reset();
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
  }
}

void BlickfeldDriverCore::createImuStream() {
  /// if connected, disconnect
  imu_stream_.reset();
  imu_scanner_.reset();
  try {
    imu_scanner_ = blickfeld::scanner::connect(host_);
    imu_stream_ = imu_scanner_->get_imu_stream();
  } catch (std::exception const& e) {
    std::cerr << "Connecting to scanner yields: " << e.what() << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void BlickfeldDriverCore::addDebugMessage(const blickfeld::protocol::data::Frame& frame,
                                          const DebuggingTimes& debugging_times) {
  log_messages_[LogLevel::Debug] << "Publishing frame #" << frame.id() << "\n";
  log_messages_[LogLevel::Debug] << "Maximum frame rate for scan pattern is "
                                 << 1.f / frame.scan_pattern().frame_rate().maximum() << "[s]\n";

  double target_frame_rate = (frame.scan_pattern().frame_rate().target() == 0)
                                 ? 1.f / frame.scan_pattern().frame_rate().maximum()
                                 : 1.f / frame.scan_pattern().frame_rate().target();
  log_messages_[LogLevel::Debug] << "Target frame rate for scan pattern is " << target_frame_rate << "[s]\n";

  log_messages_[LogLevel::Debug] << "Receiving frame took "
                                 << std::chrono::duration<double>(debugging_times.frame_receive_end_time -
                                                                  debugging_times.frame_receive_start_time)
                                        .count()
                                 << "[s]\n";
  log_messages_[LogLevel::Debug] << "Parsing frame took "
                                 << std::chrono::duration<double>(debugging_times.parse_frame_end_time -
                                                                  debugging_times.frame_receive_end_time)
                                        .count()
                                 << "[s]\n";
  log_messages_[LogLevel::Debug] << "Parsing status took "
                                 << std::chrono::duration<double>(debugging_times.parse_status_end_time -
                                                                  debugging_times.parse_frame_end_time)
                                        .count()
                                 << "[s]\n";
  log_messages_[LogLevel::Debug] << "Frame start time is " << frame.start_time_ns() / 1e9 << "[s]\n";
  log_messages_[LogLevel::Debug] << "Frame receive time is "
                                 << std::chrono::system_clock::to_time_t(debugging_times.frame_receive_end_time)
                                 << "[s]\n";
  auto frame_chrono_time = std::chrono::system_clock::from_time_t(frame.start_time_ns());
  if (last_frame_start_time_ != std::chrono::high_resolution_clock::time_point::max()) {
    /// time from start of the frame till we receive the frame
    auto end_to_end_duration =
        std::chrono::system_clock::to_time_t(debugging_times.frame_receive_end_time) - (frame.start_time_ns() / 1e9);
    auto delta_last_frame_start = (std::chrono::system_clock::to_time_t(frame_chrono_time) -
                                   std::chrono::system_clock::to_time_t(last_frame_start_time_)) /
                                  1e9;
    /// processing duration is the additional time needed to process a frame and transmit it to client
    auto processing_duration = (end_to_end_duration - delta_last_frame_start);

    log_messages_[LogLevel::Debug] << "End to end duration is " << end_to_end_duration << "[s]\n";
    log_messages_[LogLevel::Debug]
        << "Delta from last frame receive time is "
        << std::chrono::duration<double>(debugging_times.frame_receive_end_time - last_frame_receive_time_).count()
        << "[s]\n";
    log_messages_[LogLevel::Debug] << "Delta from last frame start time is " << delta_last_frame_start << "[s]\n";
    log_messages_[LogLevel::Debug] << "Frame process and transmission duration is " << processing_duration << "[s]\n";

    if (last_processing_durations_.size() >= 1000) {
      last_processing_durations_.pop_front();
    }
    last_processing_durations_.push_back(processing_duration);

    auto average_time =
        std::accumulate(std::begin(last_processing_durations_), std::end(last_processing_durations_), 0.0) /
        last_processing_durations_.size();

    log_messages_[LogLevel::Debug] << "Average processing and transmission duration of last "
                                   << last_processing_durations_.size() << " frames is " << average_time << "[s]\n";

    log_messages_[LogLevel::Debug] << "Receiving and processing IMU streaming took "
                                   << std::chrono::duration<double>(debugging_times.imu_stream_end_time -
                                                                    debugging_times.imu_stream_start_time)
                                          .count()
                                   << "[s]\n";

    log_messages_[LogLevel::Debug] << "Receiving and processing IMU static state took "
                                   << std::chrono::duration<double>(debugging_times.imu_static_end_time -
                                                                    debugging_times.imu_static_start_time)
                                          .count()
                                   << "[s]\n";
  }
  last_frame_receive_time_ = debugging_times.frame_receive_end_time;
  last_frame_start_time_ = frame_chrono_time;
}

bool BlickfeldDriverCore::isPointCloudStreaming() { return point_cloud_stream_ && point_cloud_scanner_; }

bool BlickfeldDriverCore::isImuStreaming() { return imu_stream_ && imu_scanner_; }

}  // namespace ros_interop
}  // namespace blickfeld
