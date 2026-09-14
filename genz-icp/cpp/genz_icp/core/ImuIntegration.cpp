// MIT License
#include "ImuIntegration.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

namespace {

constexpr double kTimeEpsilon = 1.0e-9;

bool IsFinite(const genz_icp::imu::Sample &sample) {
    return std::isfinite(sample.stamp) && sample.angular_velocity.allFinite() &&
           sample.linear_acceleration.allFinite() &&
           (!sample.gravity_direction_valid || sample.gravity_direction.allFinite());
}

Eigen::Vector3d TrimmedMean(const std::vector<Eigen::Vector3d> &values,
                            double trim_fraction) {
    if (values.empty()) return Eigen::Vector3d::Zero();
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    trim_fraction = std::clamp(trim_fraction, 0.0, 0.45);
    for (Eigen::Index axis = 0; axis < 3; ++axis) {
        std::vector<double> components;
        components.reserve(values.size());
        for (const auto &value : values) components.push_back(value[axis]);
        std::sort(components.begin(), components.end());
        const size_t trim = static_cast<size_t>(
            trim_fraction * static_cast<double>(components.size()));
        const size_t begin = std::min(trim, components.size() - 1);
        const size_t end = std::max(begin + 1, components.size() - trim);
        const double sum = std::accumulate(components.begin() + static_cast<std::ptrdiff_t>(begin),
                                           components.begin() + static_cast<std::ptrdiff_t>(end),
                                           0.0);
        mean[axis] = sum / static_cast<double>(end - begin);
    }
    return mean;
}

Eigen::Vector3d ClampNorm(const Eigen::Vector3d &value, double maximum) {
    if (!(maximum > 0.0) || value.norm() <= maximum) return value;
    return value.normalized() * maximum;
}

}  // namespace

namespace genz_icp::imu {

StationaryCalibrator::StationaryCalibrator(StationaryCalibrationConfig config)
    : config_(std::move(config)) {
    config_.duration_seconds = std::max(0.0, config_.duration_seconds);
    config_.min_samples = std::max<size_t>(1, config_.min_samples);
    config_.max_gyro_norm = std::max(0.0, config_.max_gyro_norm);
    config_.accel_g_tolerance = std::max(0.0, config_.accel_g_tolerance);
    config_.max_accel_variance = std::max(0.0, config_.max_accel_variance);
    config_.gravity_magnitude = std::max(1.0, config_.gravity_magnitude);
}

void StationaryCalibrator::ResetCandidate() {
    candidate_samples_.clear();
    candidate_start_ = 0.0;
    have_candidate_start_ = false;
}

std::optional<StationaryCalibrationResult> StationaryCalibrator::AddSample(
    const Sample &sample) {
    if (result_.success) return result_;

    const double accel_magnitude = sample.linear_acceleration.norm();
    const bool monotonic = !have_candidate_start_ || sample.stamp > last_stamp_;
    const bool candidate = IsFinite(sample) && monotonic &&
                           sample.angular_velocity.norm() <= config_.max_gyro_norm &&
                           std::abs(accel_magnitude - config_.gravity_magnitude) <=
                               config_.accel_g_tolerance;
    if (!candidate) {
        ++rejected_samples_;
        ResetCandidate();
        last_stamp_ = sample.stamp;
        return std::nullopt;
    }

    if (!have_candidate_start_) {
        candidate_start_ = sample.stamp;
        have_candidate_start_ = true;
    }
    last_stamp_ = sample.stamp;
    candidate_samples_.push_back(sample);

    const double duration = sample.stamp - candidate_start_;
    if (duration + kTimeEpsilon < config_.duration_seconds ||
        candidate_samples_.size() < config_.min_samples) {
        return std::nullopt;
    }

    std::vector<Eigen::Vector3d> gyros;
    std::vector<Eigen::Vector3d> accelerations;
    std::vector<Eigen::Vector3d> gravity_directions;
    std::vector<double> magnitudes;
    gyros.reserve(candidate_samples_.size());
    accelerations.reserve(candidate_samples_.size());
    gravity_directions.reserve(candidate_samples_.size());
    magnitudes.reserve(candidate_samples_.size());
    for (const auto &entry : candidate_samples_) {
        gyros.push_back(entry.angular_velocity);
        accelerations.push_back(entry.linear_acceleration);
        magnitudes.push_back(entry.linear_acceleration.norm());
        if (entry.gravity_direction_valid && entry.gravity_direction.norm() > 1.0e-6) {
            gravity_directions.push_back(entry.gravity_direction.normalized());
        }
    }

    const double magnitude_mean =
        std::accumulate(magnitudes.begin(), magnitudes.end(), 0.0) /
        static_cast<double>(magnitudes.size());
    double magnitude_variance = 0.0;
    for (const double magnitude : magnitudes) {
        const double residual = magnitude - magnitude_mean;
        magnitude_variance += residual * residual;
    }
    magnitude_variance /= static_cast<double>(magnitudes.size());
    if (magnitude_variance > config_.max_accel_variance) {
        ++rejected_samples_;
        ResetCandidate();
        return std::nullopt;
    }

    const Eigen::Vector3d acceleration_mean =
        TrimmedMean(accelerations, config_.trim_fraction);
    Eigen::Vector3d gravity_direction = acceleration_mean.normalized();
    if (!gravity_directions.empty()) {
        const Eigen::Vector3d expected_direction_mean =
            TrimmedMean(gravity_directions, config_.trim_fraction);
        if (expected_direction_mean.norm() > 1.0e-6) {
            gravity_direction = expected_direction_mean.normalized();
        }
    }

    result_.success = true;
    result_.gyro_bias = TrimmedMean(gyros, config_.trim_fraction);
    result_.gravity_specific_force = config_.gravity_magnitude * gravity_direction;
    // Preserve gravity: only the residual between the measured stationary
    // specific force and the expected +g direction is accelerometer bias.
    result_.accel_bias = acceleration_mean - result_.gravity_specific_force;
    result_.duration_seconds = duration;
    result_.accel_magnitude_variance = magnitude_variance;
    result_.accepted_samples = candidate_samples_.size();
    result_.rejected_samples = rejected_samples_;
    return result_;
}

Sophus::SO3d AlignGravity(const Sophus::SO3d &orientation,
                          const Eigen::Vector3d &gravity_direction_body,
                          double gain) {
    if (!orientation.matrix().allFinite() || !gravity_direction_body.allFinite() ||
        gravity_direction_body.norm() < 1.0e-9) {
        return orientation;
    }
    gain = std::clamp(gain, 0.0, 1.0);
    const Eigen::Vector3d measured_world =
        (orientation * gravity_direction_body.normalized()).normalized();
    Eigen::Quaterniond correction =
        Eigen::Quaterniond::FromTwoVectors(measured_world, Eigen::Vector3d::UnitZ());
    correction.normalize();
    const Sophus::SO3d correction_so3(correction);
    // FromTwoVectors has a horizontal rotation axis here, so it corrects
    // roll/pitch while adding no absolute-yaw observation.
    return Sophus::SO3d::exp(gain * correction_so3.log()) * orientation;
}

std::optional<Sample> InterpolateSample(const Sample &first,
                                        const Sample &second,
                                        double stamp) {
    const double dt = second.stamp - first.stamp;
    if (!IsFinite(first) || !IsFinite(second) || !(dt > 0.0) ||
        stamp < first.stamp - kTimeEpsilon || stamp > second.stamp + kTimeEpsilon) {
        return std::nullopt;
    }
    const double alpha = std::clamp((stamp - first.stamp) / dt, 0.0, 1.0);
    Sample result;
    result.stamp = stamp;
    result.angular_velocity =
        first.angular_velocity + alpha * (second.angular_velocity - first.angular_velocity);
    result.linear_acceleration = first.linear_acceleration +
                                 alpha * (second.linear_acceleration - first.linear_acceleration);
    result.gravity_direction_valid =
        first.gravity_direction_valid && second.gravity_direction_valid;
    if (result.gravity_direction_valid) {
        result.gravity_direction = first.gravity_direction +
                                   alpha * (second.gravity_direction - first.gravity_direction);
        if (result.gravity_direction.norm() < 1.0e-9) {
            result.gravity_direction_valid = false;
        } else {
            result.gravity_direction.normalize();
        }
    }
    return result;
}

std::optional<Trajectory> Integrate(const std::vector<Sample> &samples,
                                    double start_time,
                                    double end_time,
                                    const State &initial_state,
                                    const Eigen::Vector3d &gyro_bias,
                                    const Eigen::Vector3d &accel_bias,
                                    const IntegrationConfig &config,
                                    std::string *reason) {
    if (!std::isfinite(start_time) || !std::isfinite(end_time) ||
        end_time < start_time || !initial_state.orientation.matrix().allFinite() ||
        !initial_state.velocity.allFinite() || !initial_state.position.allFinite() ||
        !gyro_bias.allFinite() || !accel_bias.allFinite()) {
        if (reason) *reason = "non-finite state or invalid integration interval";
        return std::nullopt;
    }
    if (samples.size() < 2) {
        if (reason) *reason = "not enough IMU samples buffered";
        return std::nullopt;
    }

    const auto after_start = std::upper_bound(
        samples.begin(), samples.end(), start_time,
        [](double stamp, const Sample &sample) { return stamp < sample.stamp; });
    const auto at_or_after_end = std::lower_bound(
        samples.begin(), samples.end(), end_time,
        [](const Sample &sample, double stamp) { return sample.stamp < stamp; });
    if (after_start == samples.begin()) {
        if (reason) *reason = "IMU buffer does not cover interval start";
        return std::nullopt;
    }
    if (after_start == samples.end()) {
        if (reason) *reason = "IMU buffer has no sample after interval start";
        return std::nullopt;
    }
    if (at_or_after_end == samples.end()) {
        if (reason) *reason = "IMU buffer does not cover interval end";
        return std::nullopt;
    }

    const size_t first_index =
        static_cast<size_t>(std::distance(samples.begin(), after_start) - 1);
    const size_t last_index =
        static_cast<size_t>(std::distance(samples.begin(), at_or_after_end));
    std::vector<Sample> bounded;
    bounded.reserve(last_index - first_index + 2);
    const auto start_sample =
        InterpolateSample(samples[first_index], samples[first_index + 1], start_time);
    if (!start_sample) {
        if (reason) *reason = "could not interpolate IMU at interval start";
        return std::nullopt;
    }
    bounded.push_back(*start_sample);
    for (size_t index = first_index + 1; index <= last_index; ++index) {
        if (samples[index].stamp > start_time + kTimeEpsilon &&
            samples[index].stamp < end_time - kTimeEpsilon) {
            bounded.push_back(samples[index]);
        }
    }
    if (end_time > start_time + kTimeEpsilon) {
        const size_t end_left = last_index == 0 ? 0 : last_index - 1;
        const auto end_sample =
            InterpolateSample(samples[end_left], samples[last_index], end_time);
        if (!end_sample) {
            if (reason) *reason = "could not interpolate IMU at interval end";
            return std::nullopt;
        }
        bounded.push_back(*end_sample);
    }

    Trajectory trajectory;
    trajectory.stamps.reserve(bounded.size());
    trajectory.states.reserve(bounded.size());
    State state = initial_state;
    if (config.use_gravity_constraint && bounded.front().gravity_direction_valid) {
        state.orientation = AlignGravity(state.orientation,
                                         bounded.front().gravity_direction,
                                         config.gravity_correction_gain);
    }
    trajectory.stamps.push_back(start_time);
    trajectory.states.push_back(state);

    for (size_t index = 0; index + 1 < bounded.size(); ++index) {
        const double dt = bounded[index + 1].stamp - bounded[index].stamp;
        if (!(dt > 0.0)) {
            if (reason) *reason = "IMU timestamps are not strictly increasing";
            return std::nullopt;
        }
        trajectory.max_observed_gap = std::max(trajectory.max_observed_gap, dt);
        if (config.max_gap_seconds > 0.0 && dt > config.max_gap_seconds) {
            if (reason) *reason = "IMU gap exceeds configured maximum";
            return std::nullopt;
        }

        const Eigen::Vector3d omega_mid =
            0.5 * (bounded[index].angular_velocity +
                   bounded[index + 1].angular_velocity) -
            gyro_bias;
        if (!omega_mid.allFinite()) {
            if (reason) *reason = "non-finite bias-corrected angular velocity";
            return std::nullopt;
        }

        Sophus::SO3d mid_orientation =
            state.orientation * Sophus::SO3d::exp(omega_mid * (0.5 * dt));
        Eigen::Vector3d gravity_direction_mid = Eigen::Vector3d::UnitZ();
        const bool gravity_valid = bounded[index].gravity_direction_valid &&
                                   bounded[index + 1].gravity_direction_valid;
        if (gravity_valid) {
            gravity_direction_mid =
                (bounded[index].gravity_direction +
                 bounded[index + 1].gravity_direction).normalized();
            if (config.use_gravity_constraint) {
                mid_orientation = AlignGravity(mid_orientation,
                                               gravity_direction_mid,
                                               config.gravity_correction_gain);
            }
        }

        if (config.integrate_translation) {
            const Eigen::Vector3d specific_force_mid =
                0.5 * (bounded[index].linear_acceleration +
                       bounded[index + 1].linear_acceleration) -
                accel_bias;
            // ROS/MAVROS FLU convention observed in the Sept2 bag: a stationary
            // sensor reports +g along its body-up direction. Rotate specific
            // force into odom and subtract world +g to obtain linear accel.
            Eigen::Vector3d linear_acceleration_world =
                mid_orientation * specific_force_mid -
                Eigen::Vector3d(0.0, 0.0, config.gravity_magnitude);
            linear_acceleration_world =
                ClampNorm(linear_acceleration_world, config.max_acceleration);
            state.position += state.velocity * dt +
                              0.5 * linear_acceleration_world * dt * dt;
            state.velocity = ClampNorm(state.velocity + linear_acceleration_world * dt,
                                       config.max_velocity);
        }

        state.orientation =
            state.orientation * Sophus::SO3d::exp(omega_mid * dt);
        if (config.use_gravity_constraint &&
            bounded[index + 1].gravity_direction_valid) {
            state.orientation = AlignGravity(state.orientation,
                                             bounded[index + 1].gravity_direction,
                                             config.gravity_correction_gain);
        }
        if (!state.orientation.matrix().allFinite() || !state.velocity.allFinite() ||
            !state.position.allFinite()) {
            if (reason) *reason = "non-finite integrated IMU state";
            return std::nullopt;
        }
        trajectory.stamps.push_back(bounded[index + 1].stamp);
        trajectory.states.push_back(state);
    }
    return trajectory;
}

State InterpolateState(const Trajectory &trajectory, double stamp) {
    if (trajectory.states.empty()) return State{};
    if (stamp <= trajectory.stamps.front()) return trajectory.states.front();
    if (stamp >= trajectory.stamps.back()) return trajectory.states.back();
    const auto upper =
        std::upper_bound(trajectory.stamps.begin(), trajectory.stamps.end(), stamp);
    const size_t index =
        static_cast<size_t>(std::distance(trajectory.stamps.begin(), upper) - 1);
    const double dt = trajectory.stamps[index + 1] - trajectory.stamps[index];
    const double alpha = (stamp - trajectory.stamps[index]) / dt;
    State state;
    const Sophus::SO3d delta = trajectory.states[index].orientation.inverse() *
                               trajectory.states[index + 1].orientation;
    state.orientation = trajectory.states[index].orientation *
                        Sophus::SO3d::exp(alpha * delta.log());
    state.velocity = trajectory.states[index].velocity +
                     alpha * (trajectory.states[index + 1].velocity -
                              trajectory.states[index].velocity);
    state.position = trajectory.states[index].position +
                     alpha * (trajectory.states[index + 1].position -
                              trajectory.states[index].position);
    return state;
}

Eigen::Vector3d DeskewPoint(const Eigen::Vector3d &point,
                            const State &point_state,
                            const State &reference_state,
                            bool enable_translation) {
    Eigen::Vector3d corrected =
        reference_state.orientation.inverse() * point_state.orientation * point;
    if (enable_translation) {
        corrected += reference_state.orientation.inverse() *
                     (point_state.position - reference_state.position);
    }
    return corrected;
}

Eigen::Vector3d CorrectVelocityFromLidar(const Eigen::Vector3d &previous_position,
                                         const Eigen::Vector3d &current_position,
                                         double dt,
                                         double max_velocity) {
    if (!previous_position.allFinite() || !current_position.allFinite() ||
        !std::isfinite(dt) || !(dt > kTimeEpsilon)) {
        return Eigen::Vector3d::Zero();
    }
    return ClampNorm((current_position - previous_position) / dt, max_velocity);
}

}  // namespace genz_icp::imu
