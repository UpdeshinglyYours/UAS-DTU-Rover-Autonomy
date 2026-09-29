# Rule & Guide: SLAM Toolbox LIDAR Tuning, Hardware Realities & Modular Patches

## 1. Hardware & Physical Realities
- **Sensor Rigidity Invariant**: 2D SLAM assumes a rigid-body transform $\mathbf{T}_{\text{base\_link} \to \text{laser}} = \text{const}$. 
- **Loose / Vibrating Mount Phenomenon**:
  - If a LiDAR vibrates on the chassis, scan matching acts as an optical stabilizer, aligning laser scans and producing a crisp map.
  - However, SLAM assumes the sensor is rigid, so it attributes the physical wobble to robot odometry drift and pushes the rotational delta into $\mathbf{T}_{\text{map} \to \text{odom}}$.
  - Over a $30\text{m} \sim 50\text{m}$ trajectory, integrating small angular wiggles creates a $1\text{m} \sim 10\text{m}$ translation offset in $\mathbf{T}_{\text{map} \to \text{odom}}$, which collapses back down to $\sim 0.5\text{m}$ when returning to the starting point.
  - *Fix*: Stiffen the hardware mount; tune `correlation_search_space_smear_deviation: 0.05` to absorb residual jitter.

---

## 2. FoV Invariants (Narrow FoV < 180° / 70° Solid-State vs 360°)
1. **Disable Scan Barycenter (`use_scan_barycenter: false`)**:
   - For narrow FoV, all points lie forward, placing the barycenter meters ahead. In-place rotation causes large barycenter swing ($\Delta \mathbf{b} \approx R \cdot \Delta \theta$), breaking KD-tree spatial indexing and graph linking. Always set `use_scan_barycenter: false`.
2. **Keyframe Overlap Thresholds**:
   - Set `minimum_travel_heading: 0.03` to `0.1` ($\approx 1.7^\circ \sim 5.7^\circ$) to preserve $>80\%$ scan overlap on narrow FoV LiDARs.
3. **Occupancy Grid Generation Guardrails**:
   - Set `min_laser_range: 0.1` (never leave at 1.5m to avoid dropping obstacles near the rover).
   - Set `min_pass_through: 0` or `1` so rotating scans immediately register obstacle cells.

---

## 3. Dual Gatekeeper Architecture & Threshold Synchronization
- **Gatekeeper 1 (ROS Node `shouldProcessScan`)**: Pre-filters incoming scans and resets its reference pose.
- **Gatekeeper 2 (Karto SDK `HasMovedEnough`)**: Enforces `1.0 * min_heading` and `1.0 * min_distance`.
- **Dead-Zone Trap**: If Gatekeeper 1 triggers at 0.8x and resets its accumulator before Karto sees 1.0x, rotation scans are dropped in an infinite loop.
- **Fix**: Set `enable_synchronized_intake_thresholds: true` (or upstream `check_min_dist_and_heading_precisely: true`) to enforce synchronized 1.0x thresholds across both layers.

---

## 4. Parameter Guardrails & Failure Modes
1. **`distance_variance_penalty` Denominator Guard**:
   - Formula: $\text{Penalty} = 1.0 - 0.2 \frac{\Delta x^2 + \Delta y^2}{\mathbf{distance\_variance\_penalty}}$.
   - *Never lower below 0.3* (keep at default `0.5`). Lowering it chokes the denominator, collapses `bestResponse`, and explodes covariance ($\Sigma \propto 1/\text{bestResponse}$), causing Ceres Solver's Information Matrix $\Omega \to 0$ and warping both map and TF.
2. **`correlation_search_space_dimension` Clipping Guard**:
   - *Never reduce to 0.2* (keep at default `0.5`). A dimension of $0.2\text{m}$ limits the search window to $\pm 10\text{ cm}$. Any displacement $> 10\text{ cm}$ falls outside the box and clips scans against the boundary wall, inducing systematic drift.
3. **`correlation_search_space_smear_deviation`**:
   - Set to `0.03` to `0.05` ($3\text{ cm} \sim 5\text{ cm}$).

---

## 5. Modular C++ Source Patches for Narrow FoV (6 Toggles)

All 6 changes are strictly modular, non-destructive (original code preserved in comments), and default to `false` for 100% backward compatibility:

1. **`enable_heading_scan_intake` (Change 1)** ([src/slam_toolbox_common.cpp](file:///home/vortex/uas_nav/src/UAS-DTU-navigation-simulation/slam_toolbox/src/slam_toolbox_common.cpp#L198)):
   - Evaluates `(dist2 < threshold_mult * min_dist2 && deltaHeading < threshold_mult * min_heading)`.
   - Prevents dropping scans during stationary in-place turns.

2. **`enable_narrow_fov_valid_points` (Change 2)** ([lib/karto_sdk/src/Mapper.cpp](file:///home/vortex/uas_nav/src/UAS-DTU-navigation-simulation/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L1113)):
   - In `ScanMatcher::FindValidPoints()`, bypasses the $360^\circ$ winding ray polygon filter when `!GetIs360Laser()`.

3. **`enable_coupled_covariance` (Change 3)** ([lib/karto_sdk/src/Mapper.cpp](file:///home/vortex/uas_nav/src/UAS-DTU-navigation-simulation/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L948)):
   - Computes cross-terms ($\Sigma_{x\theta}, \Sigma_{y\theta}$) bounded by Cauchy-Schwarz limit: $\pm 0.75 \sqrt{\Sigma_{xx}\Sigma_{\theta\theta}}$.

4. **`enable_anisotropic_penalty` (Change 4)** ([lib/karto_sdk/src/Mapper.cpp](file:///home/vortex/uas_nav/src/UAS-DTU-navigation-simulation/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L670)):
   - In `ScanMatcher::operator()`, computes local frame coordinate displacement and weights lateral error 3x: `squaredDistance = localX^2 + 3.0 * localY^2`.

5. **`enable_synchronized_intake_thresholds` (Change 5)** ([src/slam_toolbox_common.cpp](file:///home/vortex/uas_nav/src/UAS-DTU-navigation-simulation/slam_toolbox/src/slam_toolbox_common.cpp#L204)):
   - When TRUE: Aligns intake thresholds to `1.0x` (`min_dist2` and `min_heading`), exactly matching Karto SDK `HasMovedEnough` and eliminating quantization dead-zones.
   - When FALSE: Preserves stock `0.8x` multiplier.

6. **`enable_heading_radian_units` (Change 6)** ([src/slam_toolbox_common.cpp](file:///home/vortex/uas_nav/src/UAS-DTU-navigation-simulation/slam_toolbox/src/slam_toolbox_common.cpp#L207)):
   - When TRUE: Converts Karto SDK's `getParamMinimumTravelHeading()` (which returns degrees) to radians via `karto::math::DegreesToRadians()`, allowing radians-to-radians comparison against `deltaHeading`.
   - When FALSE: Preserves raw getter return value.

---

## 6. Upstream Robustness & Architecture Reference
1. **Stack Overflow Protection**: Large pose graph optimizations require expanded stack (`setrlimit(RLIMIT_STACK, 40000000)`).
2. **Scan Matcher Input Validation**: Guards against $0$ or negative search parameters ($\in [10^{-6}, 10^6]$).
3. **Ceres Modernization**: Ceres $\ge 2.2$ uses `Manifold` (`AngleManifold`) instead of deprecated `LocalParameterization`.

---

## 7. Non-Destructive Code Patching & Build Invariants
1. **Comment-Out Policy**: Never delete original lines or author comments. Wrap them in `/* [ORIGINAL SLAM TOOLBOX CODE - PRESERVED] */`.
2. **Clear Labeling**: Label all additions with `// [MODULAR NARROW FOV OPTIMIZATION ...]` headers.
3. **Workspace Build Rule**: Never run `colcon build` inside package source subfolders (`src/slam_toolbox/`). Always run from workspace root (`/home/vortex/uas_nav`).
4. **Sourcing Rule**: Always source `source /home/vortex/uas_nav/install/setup.bash` to avoid executing system Debian binaries from `/opt/ros/humble/`.
