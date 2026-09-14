#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <optional>
#include <sophus/so3.hpp>
#include <string>
#include <vector>

#include "genz_icp/core/ImuIntegration.hpp"
#include "genz_icp/core/Registration.hpp"
#include "genz_icp/pipeline/GenZICP.hpp"

namespace {

using genz_icp::imu::IntegrationConfig;
using genz_icp::imu::Sample;
using genz_icp::imu::State;
using genz_icp::imu::Trajectory;

std::vector<Sample> Samples(double duration,
                            double dt,
                            const Eigen::Vector3d &omega,
                            const Eigen::Vector3d &acceleration,
                            const Eigen::Vector3d &gravity_direction =
                                Eigen::Vector3d::UnitZ()) {
    std::vector<Sample> samples;
    for (double stamp = 0.0; stamp < duration + 0.5 * dt; stamp += dt) {
        Sample sample;
        sample.stamp = std::min(stamp, duration);
        sample.angular_velocity = omega;
        sample.linear_acceleration = acceleration;
        sample.gravity_direction = gravity_direction;
        sample.gravity_direction_valid = true;
        samples.push_back(sample);
    }
    return samples;
}

Trajectory IntegrateOrFail(const std::vector<Sample> &samples,
                           const State &initial,
                           const IntegrationConfig &config,
                           const Eigen::Vector3d &gyro_bias = Eigen::Vector3d::Zero(),
                           const Eigen::Vector3d &accel_bias = Eigen::Vector3d::Zero()) {
    std::string reason;
    const auto result = genz_icp::imu::Integrate(
        samples, samples.front().stamp, samples.back().stamp, initial,
        gyro_bias, accel_bias, config, &reason);
    EXPECT_TRUE(result.has_value()) << reason;
    return result.value_or(Trajectory{});
}

TEST(ImuIntegration, StationaryImuHasNearZeroVelocityDrift) {
    IntegrationConfig config;
    config.integrate_translation = true;
    const auto trajectory = IntegrateOrFail(
        Samples(2.0, 0.01, Eigen::Vector3d::Zero(),
                Eigen::Vector3d(0.0, 0.0, genz_icp::imu::kStandardGravity)),
        State{}, config);
    EXPECT_LT(trajectory.states.back().velocity.norm(), 1.0e-9);
    EXPECT_LT(trajectory.states.back().position.norm(), 1.0e-9);
}

TEST(ImuIntegration, ConstantAccelerationProducesExpectedDeltaPosition) {
    IntegrationConfig config;
    config.integrate_translation = true;
    config.max_acceleration = 10.0;
    const auto trajectory = IntegrateOrFail(
        Samples(1.0, 0.01, Eigen::Vector3d::Zero(),
                Eigen::Vector3d(1.0, 0.0, genz_icp::imu::kStandardGravity)),
        State{}, config);
    EXPECT_NEAR(trajectory.states.back().velocity.x(), 1.0, 1.0e-6);
    EXPECT_NEAR(trajectory.states.back().position.x(), 0.5, 1.0e-6);
}

TEST(ImuIntegration, ConstantYawRateProducesExpectedRotation) {
    IntegrationConfig config;
    const auto trajectory = IntegrateOrFail(
        Samples(1.0, 0.01, Eigen::Vector3d(0.0, 0.0, 0.5),
                Eigen::Vector3d(0.0, 0.0, genz_icp::imu::kStandardGravity)),
        State{}, config);
    EXPECT_NEAR(trajectory.states.back().orientation.log().z(), 0.5, 1.0e-6);
}

TEST(ImuIntegration, GravityOnlySpecificForceProducesNoTranslation) {
    IntegrationConfig config;
    config.integrate_translation = true;
    config.use_gravity_constraint = true;
    const auto trajectory = IntegrateOrFail(
        Samples(1.0, 0.01, Eigen::Vector3d::Zero(),
                Eigen::Vector3d(0.0, 0.0, genz_icp::imu::kStandardGravity)),
        State{}, config);
    EXPECT_LT(trajectory.states.back().position.norm(), 1.0e-9);
}

TEST(ImuIntegration, TiltedStationaryImuRemovesGravityCorrectly) {
    const Sophus::SO3d tilted =
        Sophus::SO3d::exp(Eigen::Vector3d(0.25, -0.17, 0.0));
    const Eigen::Vector3d gravity_body =
        tilted.inverse() * Eigen::Vector3d::UnitZ();
    State initial;
    initial.orientation = tilted;
    IntegrationConfig config;
    config.integrate_translation = true;
    config.use_gravity_constraint = true;
    const auto trajectory = IntegrateOrFail(
        Samples(1.0, 0.01, Eigen::Vector3d::Zero(),
                genz_icp::imu::kStandardGravity * gravity_body, gravity_body),
        initial, config);
    EXPECT_LT(trajectory.states.back().velocity.norm(), 1.0e-8);
    EXPECT_LT(trajectory.states.back().position.norm(), 1.0e-8);
}

TEST(ImuCalibration, EstimatesAccelerometerBiasWithoutRemovingGravity) {
    genz_icp::imu::StationaryCalibrationConfig config;
    config.duration_seconds = 0.5;
    config.min_samples = 40;
    config.max_accel_variance = 0.1;
    genz_icp::imu::StationaryCalibrator calibrator(config);
    const Eigen::Vector3d expected_bias(0.12, -0.08, 0.05);
    std::optional<genz_icp::imu::StationaryCalibrationResult> result;
    for (int index = 0; index <= 60; ++index) {
        Sample sample;
        sample.stamp = 0.01 * static_cast<double>(index);
        sample.linear_acceleration =
            Eigen::Vector3d(0.0, 0.0, genz_icp::imu::kStandardGravity) + expected_bias;
        sample.gravity_direction = Eigen::Vector3d::UnitZ();
        sample.gravity_direction_valid = true;
        result = calibrator.AddSample(sample);
        if (result) break;
    }
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->accel_bias.x(), expected_bias.x(), 1.0e-9);
    EXPECT_NEAR(result->accel_bias.y(), expected_bias.y(), 1.0e-9);
    EXPECT_NEAR(result->accel_bias.z(), expected_bias.z(), 1.0e-9);
    EXPECT_NEAR(result->gravity_specific_force.norm(),
                genz_icp::imu::kStandardGravity, 1.0e-9);
}

TEST(ImuCalibration, EstimatesGyroscopeBias) {
    genz_icp::imu::StationaryCalibrationConfig config;
    config.duration_seconds = 0.5;
    config.min_samples = 40;
    genz_icp::imu::StationaryCalibrator calibrator(config);
    const Eigen::Vector3d expected_bias(0.001, -0.002, 0.003);
    std::optional<genz_icp::imu::StationaryCalibrationResult> result;
    for (int index = 0; index <= 60; ++index) {
        Sample sample;
        sample.stamp = 0.01 * static_cast<double>(index);
        sample.angular_velocity = expected_bias;
        sample.linear_acceleration.z() = genz_icp::imu::kStandardGravity;
        sample.gravity_direction_valid = true;
        result = calibrator.AddSample(sample);
        if (result) break;
    }
    ASSERT_TRUE(result.has_value());
    EXPECT_LT((result->gyro_bias - expected_bias).norm(), 1.0e-12);
}

TEST(ImuCalibration, RejectsNonMonotonicTimestamp) {
    genz_icp::imu::StationaryCalibrator calibrator;
    Sample sample;
    sample.linear_acceleration.z() = genz_icp::imu::kStandardGravity;
    sample.stamp = 1.0;
    EXPECT_FALSE(calibrator.AddSample(sample).has_value());
    EXPECT_FALSE(calibrator.AddSample(sample).has_value());
    EXPECT_EQ(calibrator.RejectedSamples(), 1U);
}

TEST(ImuIntegration, RejectsLargeImuGap) {
    auto samples = Samples(1.0, 0.5, Eigen::Vector3d::Zero(),
                           Eigen::Vector3d(0.0, 0.0,
                                           genz_icp::imu::kStandardGravity));
    IntegrationConfig config;
    config.max_gap_seconds = 0.1;
    std::string reason;
    EXPECT_FALSE(genz_icp::imu::Integrate(
        samples, 0.0, 1.0, State{}, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), config, &reason));
    EXPECT_NE(reason.find("gap"), std::string::npos);
}

TEST(ImuIntegration, GravityAlignmentKeepsQuaternionNormalized) {
    const Sophus::SO3d aligned = genz_icp::imu::AlignGravity(
        Sophus::SO3d::exp(Eigen::Vector3d(0.4, -0.2, 0.7)),
        Eigen::Vector3d(0.2, -0.3, 0.93).normalized());
    EXPECT_NEAR(aligned.unit_quaternion().norm(), 1.0, 1.0e-12);
}

TEST(GroundConstraint, PlanarModeRemovesZRollPitchIncrement) {
    genz_icp::RegistrationDofConstraintConfig constraint;
    constraint.enabled = true;
    constraint.lock_z = true;
    constraint.lock_roll_pitch = true;
    Eigen::Matrix<double, 6, 1> increment;
    increment << 1.0, 2.0, 3.0, 0.1, 0.2, 0.3;
    const auto constrained = genz_icp::ApplyLockedDofMask(increment, constraint);
    EXPECT_DOUBLE_EQ(constrained[0], 1.0);
    EXPECT_DOUBLE_EQ(constrained[1], 2.0);
    EXPECT_DOUBLE_EQ(constrained[2], 0.0);
    EXPECT_DOUBLE_EQ(constrained[3], 0.0);
    EXPECT_DOUBLE_EQ(constrained[4], 0.0);
    EXPECT_DOUBLE_EQ(constrained[5], 0.3);
}

TEST(ImuIntegration, IcpCorrectionResetsPredictionVelocity) {
    const Eigen::Vector3d corrected = genz_icp::imu::CorrectVelocityFromLidar(
        Eigen::Vector3d::Zero(), Eigen::Vector3d(0.3, 0.0, 0.0), 0.1, 5.0);
    EXPECT_NEAR(corrected.x(), 3.0, 1.0e-12);
    EXPECT_LT(corrected.norm(), 5.0 + 1.0e-12);
    const Eigen::Vector3d reset_on_bad_dt = genz_icp::imu::CorrectVelocityFromLidar(
        Eigen::Vector3d::Zero(), Eigen::Vector3d(100.0, 0.0, 0.0), -1.0, 5.0);
    EXPECT_TRUE(reset_on_bad_dt.isZero(0.0));
}

TEST(ImuIntegration, InterpolatesAtLidarTimestamp) {
    Sample first;
    first.stamp = 10.0;
    Sample second;
    second.stamp = 12.0;
    second.angular_velocity.x() = 2.0;
    second.linear_acceleration.y() = 4.0;
    second.gravity_direction_valid = true;
    first.gravity_direction_valid = true;
    const auto midpoint = genz_icp::imu::InterpolateSample(first, second, 11.0);
    ASSERT_TRUE(midpoint.has_value());
    EXPECT_DOUBLE_EQ(midpoint->angular_velocity.x(), 1.0);
    EXPECT_DOUBLE_EQ(midpoint->linear_acceleration.y(), 2.0);
}

TEST(ImuIntegration, TranslationalDeskewDisabledPreservesRotationOnlyBehavior) {
    State point_state;
    point_state.orientation = Sophus::SO3d::exp(Eigen::Vector3d(0.0, 0.0, 0.2));
    point_state.position = Eigen::Vector3d(3.0, 4.0, 5.0);
    const Eigen::Vector3d point(1.0, 0.0, 0.0);
    const Eigen::Vector3d expected = point_state.orientation * point;
    EXPECT_LT((genz_icp::imu::DeskewPoint(point, point_state, State{}, false) -
               expected).norm(), 1.0e-12);
}

TEST(ImuIntegration, ImuTranslationDisabledPreservesInitialPosition) {
    State initial;
    initial.position = Eigen::Vector3d(1.0, 2.0, 3.0);
    initial.velocity = Eigen::Vector3d(4.0, 0.0, 0.0);
    IntegrationConfig config;
    config.integrate_translation = false;
    config.use_gravity_constraint = false;
    const auto trajectory = IntegrateOrFail(
        Samples(1.0, 0.01, Eigen::Vector3d::Zero(),
                Eigen::Vector3d(100.0, 0.0, 100.0)),
        initial, config);
    EXPECT_LT((trajectory.states.back().position - initial.position).norm(), 1.0e-12);
}

TEST(Compatibility, ImuDisabledUsesLegacyRegisterFramePath) {
    genz_icp::pipeline::GenZConfig config;
    config.voxel_size = 0.1;
    config.desired_num_voxelized_points = 1000;
    config.enable_registration_quality_gate = false;
    genz_icp::pipeline::GenZICP legacy(config);
    genz_icp::pipeline::GenZICP optional_prediction(config);

    std::vector<Eigen::Vector3d> frame;
    for (int x = -5; x <= 5; ++x) {
        for (int y = -5; y <= 5; ++y) {
            frame.emplace_back(0.2 * x, 0.2 * y, 0.01 * x * y);
        }
    }

    legacy.RegisterFrame(frame);
    optional_prediction.RegisterFrameWithPrediction(frame, std::nullopt);
    legacy.RegisterFrame(frame);
    optional_prediction.RegisterFrameWithPrediction(frame, std::nullopt);

    ASSERT_EQ(legacy.poses().size(), optional_prediction.poses().size());
    ASSERT_FALSE(legacy.poses().empty());
    EXPECT_LT((legacy.poses().back().matrix() -
               optional_prediction.poses().back().matrix()).norm(),
              1.0e-12);
}

}  // namespace
