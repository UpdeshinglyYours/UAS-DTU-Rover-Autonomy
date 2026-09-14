// MIT License
#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <optional>
#include <sophus/so3.hpp>
#include <string>
#include <vector>

namespace genz_icp::imu {

constexpr double kStandardGravity = 9.80665;

struct Sample {
    double stamp = 0.0;
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
    // Unit specific-force direction in the IMU frame. For a ROS FLU IMU at
    // rest this points opposite physical gravity. It can be obtained without
    // using absolute yaw as q_world_imu.inverse() * UnitZ().
    Eigen::Vector3d gravity_direction = Eigen::Vector3d::UnitZ();
    bool gravity_direction_valid = false;
};

struct StationaryCalibrationConfig {
    double duration_seconds = 3.0;
    size_t min_samples = 200;
    double max_gyro_norm = 0.05;
    double accel_g_tolerance = 0.75;
    // Variance of ||linear_acceleration|| in (m/s^2)^2.
    double max_accel_variance = 0.05;
    double gravity_magnitude = kStandardGravity;
    double trim_fraction = 0.1;
};

struct StationaryCalibrationResult {
    bool success = false;
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity_specific_force =
        Eigen::Vector3d(0.0, 0.0, kStandardGravity);
    double duration_seconds = 0.0;
    double accel_magnitude_variance = 0.0;
    size_t accepted_samples = 0;
    size_t rejected_samples = 0;
};

class StationaryCalibrator {
public:
    explicit StationaryCalibrator(StationaryCalibrationConfig config = {});

    std::optional<StationaryCalibrationResult> AddSample(const Sample &sample);
    void ResetCandidate();
    bool Ready() const { return result_.success; }
    const StationaryCalibrationResult &Result() const { return result_; }
    size_t RejectedSamples() const { return rejected_samples_; }

private:
    StationaryCalibrationConfig config_;
    std::vector<Sample> candidate_samples_;
    double candidate_start_ = 0.0;
    double last_stamp_ = 0.0;
    bool have_candidate_start_ = false;
    size_t rejected_samples_ = 0;
    StationaryCalibrationResult result_;
};

struct IntegrationConfig {
    double max_gap_seconds = 0.06;
    double max_acceleration = 5.0;
    double max_velocity = 5.0;
    double gravity_magnitude = kStandardGravity;
    double gravity_correction_gain = 1.0;
    bool use_gravity_constraint = false;
    bool integrate_translation = false;
};

struct State {
    Sophus::SO3d orientation;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
};

struct Trajectory {
    std::vector<double> stamps;
    std::vector<State> states;
    double max_observed_gap = 0.0;
};

Sophus::SO3d AlignGravity(const Sophus::SO3d &orientation,
                          const Eigen::Vector3d &gravity_direction_body,
                          double gain = 1.0);

std::optional<Sample> InterpolateSample(const Sample &first,
                                        const Sample &second,
                                        double stamp);

std::optional<Trajectory> Integrate(const std::vector<Sample> &samples,
                                    double start_time,
                                    double end_time,
                                    const State &initial_state,
                                    const Eigen::Vector3d &gyro_bias,
                                    const Eigen::Vector3d &accel_bias,
                                    const IntegrationConfig &config,
                                    std::string *reason = nullptr);

State InterpolateState(const Trajectory &trajectory, double stamp);

Eigen::Vector3d DeskewPoint(const Eigen::Vector3d &point,
                            const State &point_state,
                            const State &reference_state,
                            bool enable_translation);

Eigen::Vector3d CorrectVelocityFromLidar(const Eigen::Vector3d &previous_position,
                                         const Eigen::Vector3d &current_position,
                                         double dt,
                                         double max_velocity);

}  // namespace genz_icp::imu
