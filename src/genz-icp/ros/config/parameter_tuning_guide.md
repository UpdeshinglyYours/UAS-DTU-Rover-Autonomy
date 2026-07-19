# GenZ-ICP Parameter Tuning Guide

GenZ-ICP is designed to perform well across diverse environments. For optimal performance on each dataset, parameter tuning is recommended. 
This section provides tips for tuning GenZ-ICP's parameters.

## :star2: Key Parameters

### `voxel_size`
: Voxel size of local map (default: `0.3`)
+ Larger **_voxel_size_** speed up processing but reduce accuracy.
+ It is recommended to adjust **_voxel_size_** proportionally to the **scale** of the environment: **larger** values for wide **outdoor** spaces and **smaller** values for narrow **indoor** spaces.

### `max_points_per_voxel`
: Maximum points in a single voxel (default: `1`)
+ Lower **_max_points_per_voxel_** speed up processing but reduce accuracy.
+ Similar to **_voxel_size_**, this parameter should also be adjusted proportionally to the **scale** of the environment.

### `planarity_threshold`
: Threshold for planarity classification (default: `0.2`)
+ Lower **_planarity_threshold_** classifies planars pair more strictly.
+ In narrow **indoor** environments, a smaller **_max_points_per_voxel_** is typically used, which reduces the number of neighboring points available for covariance calculation during planarity classification. As a result, even planar surfaces can exhibit relatively high local surface variation.
  To prevent rejecting valid planar pairs due to this, a relatively higher **_planarity_threshold_** is recommended for indoor environments.
+ Conversely, in wide **outdoor** environments, a larger **_max_points_per_voxel_** increases the number of neighboring points, resulting in more reliable local surface variation values.
  Therefore, in outdoor environments, lowering the planarity_threshold is recommended to achieve stricter planarity classification.

### `desired_num_voxelized_points`
: Desired number of points in a voxelized scan (default: `2000`)
+ If this value is too large, it can cause CPU overload, while a value too small may lead to inaccurate results.
+ This value should be set proportionally to the **scale** of the environment: **larger** values for wide **outdoor** spaces and **smaller** values for narrow **indoor** spaces.
+ Based on this value, the voxel filter size is adaptively adjusted to perform **adaptive voxelization**.

### `max_num_iterations`
: Maximum number of iterations for the ICP loop (default: `100`)
+ Higher **_max_num_iterations_** can improve accuracy but increases CPU load.

### `enable_registration_quality_gate`
: Enables a commit-time quality gate for the final ICP candidate (default: `true`)
+ The gate lets ICP compute a full candidate pose first, then rejects the whole candidate if its final RMSE, correspondence count, pose jump, or numerical validity is suspicious.
+ Rejected candidates do not update odometry state, the local map, or the accepted-RMSE moving average.
+ The RMSE baseline is an exponential moving average controlled by **_registration_rmse_ema_alpha_** and is updated only on accepted registrations.
+ During startup, the gate uses finite checks, correspondence count, and pose-jump limits immediately, but waits for a few accepted registrations before applying the adaptive RMSE limit.
+ After **_max_consecutive_registration_rejections_**, recovery mode keeps odometry frozen and continues skipping map updates until a candidate passes the normal gate again. This is conservative: it avoids poisoning the map during aggressive yaw at the cost of temporarily holding the last valid pose.

Related parameters:
+ **_min_registration_correspondences_**: minimum final correspondence count, default `300`.
+ **_registration_rmse_reject_ratio_**: adaptive RMSE multiplier over the accepted-frame EMA, default `2.5`.
+ **_absolute_registration_rmse_limit_**: floor for the adaptive RMSE limit, default `1.0`.
+ **_max_registration_translation_per_frame_**: maximum candidate translation jump in meters, default `1.0`.
+ **_max_registration_rotation_per_frame_deg_**: maximum candidate rotation jump in degrees, default `45.0`.

### `enable_imu_motion_prediction`
: Uses IMU gyro integration to improve ICP's initial yaw/rotation guess (default: `false`)
+ This does not replace ICP and does not bypass the registration quality gate. It only changes the starting pose used by ICP.
+ The node subscribes to **_imu_topic_**, ignores IMU orientation and linear acceleration, scales **_angular_velocity_** by **_imu_angular_velocity_scale_**, subtracts the calibrated or manual gyro bias, and integrates the gyro between the last accepted scan reference time and the current scan reference time.
+ Only the rotational part of GenZ-ICP's motion model is replaced by the IMU delta. The translation prior remains GenZ-ICP's existing prediction; acceleration is not integrated.
+ After a rejected registration, the last accepted pose and scan reference time are not advanced. The next frame's IMU prediction therefore integrates from the last accepted scan to the current scan, which helps prevent rejection cascades during fast yaw.
+ If IMU prediction is unavailable because of missing point timing, calibration, missing IMU coverage, or an excessive IMU gap, GenZ-ICP falls back to its normal initial guess and the quality gate still evaluates the final ICP candidate.

Related parameters:
+ **_imu_topic_**: IMU topic, default `/bf_lidar/imu_out`.
+ **_imu_prediction_point_time_field_**: point time field name or `auto`, default `auto`.
+ **_imu_prediction_point_time_unit_**: `seconds`, `milliseconds`, `microseconds`, or `nanoseconds`, default `nanoseconds`.
+ **_imu_prediction_cloud_stamp_location_**: whether the cloud header is scan `start`, `middle`, or `end`, default `end`.
+ **_imu_prediction_deskew_reference_**: scan reference convention, default `middle`.
+ **_imu_angular_velocity_scale_**: multiply raw gyro by this before use. For Blickfeld gyro values published in deg/s, use `0.017453292519943295`.
+ **_enable_imu_prediction_gyro_bias_calibration_**, **_imu_prediction_gyro_bias_calibration_seconds_**, **_imu_prediction_gyro_bias_min_samples_**, **_imu_prediction_gyro_bias_**: automatic or manual gyro bias handling.
+ **_imu_prediction_max_gap_seconds_**: reject prediction over IMU gaps larger than this, default `0.06`.
+ **_imu_prediction_max_age_seconds_**: reject prediction intervals older/longer than this, default `0.25`.
+ **_imu_prediction_max_rejected_frame_age_seconds_**: larger prediction age allowed while one or more registration candidates have been rejected, default `1.0`. This keeps IMU prediction available when the last accepted scan reference is stale during a rejection cascade.
+ **_imu_prediction_debug_**: prints prediction availability, integrated rotation, and initial guess source.

### `enable_yaw_search_initializer`
: Searches a small set of yaw offsets around the current initial guess before running the real ICP (default: `false`)
+ Yaw search runs after the normal/IMU prediction and only chooses the initial guess for the real ICP solve.
+ It does not update the local map, pose history, odometry output, adaptive threshold, or quality-gate state.
+ Each candidate transforms the current scan by `initial_guess * yaw_delta`, queries correspondences in the existing local map, and scores the candidate using RMSE or weighted RMSE.
+ The final ICP candidate still must pass **_enable_registration_quality_gate_**; yaw search is only a local-minimum avoidance initializer.
+ For rover yaw failures, enable this in combination with IMU prediction so the search is centered on the gyro-predicted orientation.

Related parameters:
+ **_yaw_search_degrees_**: yaw offsets to test, default `[-12.0, -8.0, -4.0, 0.0, 4.0, 8.0, 12.0]`.
+ **_yaw_search_score_max_correspondence_distance_**: correspondence radius for cheap scoring, default `1.5`.
+ **_yaw_search_min_correspondences_**: minimum scoring correspondences required for a candidate, default `500`.
+ **_yaw_search_use_weighted_rmse_**: use robust weighted RMSE for candidate score when available, default `true`.
+ **_yaw_search_vertical_axis_**: LiDAR-frame axis for yaw offsets, usually `z`.
+ **_yaw_search_debug_**: prints every candidate score and the selected yaw offset.

### `enable_motion_prior`
: Adds a soft pose prior inside the ICP normal equations (default: `false`)
+ The prior mean is the final initial guess after the normal motion model, optional IMU rotation prediction, and optional yaw-search initializer.
+ It penalizes the ICP solution for moving far away from that predicted pose, but it is not a hard clamp. Scan residuals can still overcome the prior when the map evidence is strong.
+ The residual is `log(T_prior.inverse() * T_current)` in the same `[translation, rotation]` tangent ordering used by the registration solver.
+ Translation, vertical motion, roll/pitch, and yaw have separate sigmas. Use a relatively large yaw sigma for rovers, because fast yaw is real motion; the prior is mainly meant to suppress fake lateral/vertical translation.
+ The registration quality gate still judges the final ICP result. The prior only makes bad local minima less attractive before the gate has to reject them.

Related parameters:
+ **_motion_prior_translation_sigma_**: x/y translation sigma in meters, default `0.35`.
+ **_motion_prior_z_sigma_**: z translation sigma in meters, default `0.12`.
+ **_motion_prior_roll_pitch_sigma_deg_**: roll/pitch sigma in degrees, default `6.0`.
+ **_motion_prior_yaw_sigma_deg_**: yaw sigma in degrees, default `30.0`.
+ **_motion_prior_weight_**: multiplier on the prior information matrix, default `1.0`.
+ **_motion_prior_apply_during_recovery_**: keep the prior active during rejection/recovery cascades, default `true`.
+ **_motion_prior_debug_**: prints prior settings and final deviation from the prior.

### `enable_map_update_quality_gate`
: Separates odometry acceptance from local-map insertion quality (default: `false`)
+ Registration acceptance still decides whether the pose is committed and published.
+ The map-update gate runs only after a registration has been accepted. If it fails, odometry is still accepted, but the current scan is not inserted into the local map.
+ This helps reduce duplicated or blurred map features from borderline accepted frames without freezing the trajectory.
+ The weighted-RMSE ratio uses the accepted-frame weighted-RMSE EMA as its baseline.

Related parameters:
+ **_map_update_max_weighted_rmse_ratio_**: maximum weighted RMSE over accepted-frame weighted-RMSE EMA, default `1.25`.
+ **_map_update_max_rmse_**: absolute RMSE map insertion limit, default `0.35`.
+ **_map_update_max_translation_delta_**: maximum accepted-frame translation delta for map insertion, default `0.75`.
+ **_map_update_max_rotation_delta_deg_**: maximum accepted-frame rotation delta for map insertion, default `10.0`.
+ **_map_update_min_correspondences_**: minimum correspondences for map insertion, default `2500`.
+ **_map_update_debug_**: prints map-update accepted/skipped decisions.

## :zap: Minor Parameters

### `deskew`
: Enables or disables deskewing of LiDAR scans (default: `false`)
+ When the platform exhibits aggressive motion, enabling deskewing can lead to inaccuracies.
+ Additionally, the effect of deskewing diminishes as the platform's speed decreases.
+ Therefore, for platforms like hand-held devices or quadruped robots that exhibit slow or aggressive motion, setting **_deskew_** to `false` is recommended.
+ Conversely, for platforms with high-speed and smooth motion, such as vehicles used in datasets like KITTI or MulRan, setting **_deskew_** to `true` is recommended.

### `map_cleanup_radius`
: Radius of local map (default: `max_range`)
+ The default value for **_map_cleanup_radius_** is equal to the LiDAR's **_max_range_**.
+ In spaces larger than the LiDAR's **_max_range_**, where the platform revisits previously visited areas, it is recommended to increase the **_map_cleanup_radius_**.
+ However, excessively high values can consume a lot of memory and may lead to inaccurate results. Therefore, it is recommended to keep the value below `300`.

## ROS 2 Config File Note

When `config_file` is provided to `odometry.launch.py`, scalar values from that YAML are loaded by the node after parameter declaration and can override launch-file defaults. Put registration quality gate values in the YAML config when using `config_file`, or pass an empty `config_file` and set the parameters directly through launch arguments/`--ros-args`.
