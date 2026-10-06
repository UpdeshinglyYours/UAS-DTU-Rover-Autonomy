#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from mavros_msgs.msg import OverrideRCIn
from nav_msgs.msg import Odometry

from collections import deque

import numpy as np

from rclpy.qos import qos_profile_sensor_data
import yaml
from pathlib import Path

class OmegaController(Node):

    def __init__(self):

        super().__init__("omega_controller")

        #########################################################
        # USER PARAMETERS
        #########################################################
        
        self.target = 0.0
        self.target_sign = 1
        self.target_mag = 0.0
        self.control_state = 0
# 0 = inside band
# 1 = correcting high omega
# -1 = correcting low omega
        # ROS convention is +angular.z = counter-clockwise/left.
        # This rover's steering convention was observed to be reversed,
        # so invert incoming cmd_vel yaw commands before control.
        self.declare_parameter("cmd_angular_sign", 1.0)
        self.cmd_angular_sign = float(
            self.get_parameter("cmd_angular_sign").value
        )
        self.last_target = self.target
        self.band = 0.02

        self.have_cmd = False

        self.current_pwm = 1500

        self.throttle_pwm = 1500

        #########################################################
        # Linear velocity feedback -- same architecture as the
        # angular (omega) controller: a feedforward lookup table
        # gives an initial PWM guess, then a dead-time-aware,
        # gain-scheduled bang-bang loop with hysteresis trims it
        # in closed loop. Replaces the old free-running PI +
        # slew-rate approach, which did not share the angular
        # controller's dead-time / settle-detection logic and was
        # prone to misbehaving the same way the old omega PI did.
        #########################################################

        # Deadband (m/s) below which no feedback correction is applied.
        # Combined with a proportional term at runtime, same as
        # self.band for omega (max(min_band, fraction * |target|)).
        self.declare_parameter("linear_band", 0.03)
        self.linear_band = float(self.get_parameter("linear_band").value)

        self.declare_parameter("linear_pwm_min", 1100)
        self.linear_pwm_min = int(self.get_parameter("linear_pwm_min").value)

        self.declare_parameter("linear_pwm_max", 1900)
        self.linear_pwm_max = int(self.get_parameter("linear_pwm_max").value)

        # Dead-time to ignore feedback for after a fresh feedforward
        # command, mirroring self.ignore_gradient_time for omega.
        self.declare_parameter("linear_dead_time", 1.6)
        self.linear_ignore_gradient_time = float(
            self.get_parameter("linear_dead_time").value
        )

        # Settle threshold on dv/dt (m/s^2), mirrors gradient_threshold.
        self.declare_parameter("linear_gradient_threshold", 0.01)
        self.linear_gradient_threshold = float(
            self.get_parameter("linear_gradient_threshold").value
        )

        # Safety timeout on the adaptive wait, mirrors max_wait.
        self.declare_parameter("linear_max_wait", 2.0)
        self.linear_max_wait = float(
            self.get_parameter("linear_max_wait").value
        )

        # Target change (m/s) big enough to trigger a fresh
        # feedforward re-init, mirrors the 0.02 rad/s threshold
        # used for target_mag in cmd_callback.
        self.declare_parameter("linear_retrigger", 0.03)
        self.linear_retrigger = float(
            self.get_parameter("linear_retrigger").value
        )

        self.target_linear = 0.0
        self.target_linear_sign = 1
        self.target_linear_mag = 0.0

        self.linear_velocity = 0.0
        self.linear_buffer = deque(maxlen=8)
        self.linear_history = deque(maxlen=25)

        self.linear_control_state = 0
        # 0 = inside band
        # 1 = correcting high linear velocity
        # -1 = correcting low linear velocity

        self.linear_ff_initialized = False
        self.linear_last_change_time = -100.0
        self.linear_command_start_time = 0.0
        self.linear_current_gain = 0.005

        self.linear_prev_pwm = None
        self.linear_prev_v = None

       # self.step = 1
        
        self.current_gain = 0.005
        self.command_start_time = 0.0
        self.pending_learning = False

        #########################################################
# Online learning
        #########################################################

        self.prev_pwm = None
        self.prev_omega = None

        self.last_learning_target = None
        #########################################################
# Dead-time compensation
#########################################################

        self.ignore_gradient_time = 1.6     # seconds

     #   self.learning_alpha = 0.05
                #########################################################
                # Adaptive waiting
                #########################################################

        self.gradient_threshold = 0.01      # rad/s²

        self.max_wait = 2.0                 # seconds

       
       #########################################################
        # Adaptive feedback gain scheduling
        #########################################################

        self.min_pwm_step = 1

        self.max_pwm_step = 8

        # Prevent division by tiny gains
        self.min_gain = 0.001

        # Scale factor (can tune later)
        self.feedback_scale = 0.5

       
       
        #########################################################
        # Feedforward table
        #########################################################

        # self.ff_table = {
        #     0.00: 1500,
        #     0.20: 1435,        # 0.20: 1468,
        #     0.40: 1417,
        #     0.60: 1400,           #  0.40: 1435, 
        #     0.80: 1385,            # 0.70: 1400,
        #   #  1.00: 1365,
        #   #  1.30: 1330,
        # }

    #     self.ff_table = {

    # # omega :
    # #   pwm  -> feedforward PWM
    # #   gain -> dω/dPWM (initial estimate)

    #         0.00: {
    #             "pwm": 1500,
    #             "gain": 0.010,
    #         },

    #         0.20: {
    #             "pwm": 1435,
    #             "gain": 0.008,
    #         },

    #         0.40: {
    #             "pwm": 1417,
    #             "gain": 0.006,
    #         },

    #         0.60: {
    #             "pwm": 1400,
    #             "gain": 0.004,
    #         },

    #         0.80: {
    #             "pwm": 1385,
    #             "gain": 0.003,
    #         },
    #     }
         
        self.ff_file = Path.home() / "omega_feedforward.yaml"

        if self.ff_file.exists():

            with open(self.ff_file, "r") as f:
                self.ff_table = yaml.safe_load(f)

        else:
        
            self.ff_table = {
            
                0.00: {"pwm":1500,"gain":0.010,"settle":0.60,"samples":0},

                0.20: {"pwm":1435,"gain":0.008,"settle":0.80,"samples":0},

                0.40: {"pwm":1417,"gain":0.006,"settle":1.00,"samples":0},

                0.60: {"pwm":1400,"gain":0.004,"settle":1.30,"samples":0},

                0.80: {"pwm":1395,"gain":0.003,"settle":1.60,"samples":0},
            }

         #########################################################
        # Linear feedforward table (m/s -> PWM), same shape/lookup
        # as the omega table above. Values supplied by the user;
        # symmetric about 1500 (reverse uses the same 3000-pwm
        # mirror trick as negative-sign omega targets), so only the
        # forward (positive) side needs to be tabulated. "gain" is
        # an initial dv/dPWM estimate (finite-differenced from the
        # table itself); "settle" values are rough starting guesses
        # -- tune both from the [LINEAR LEARN] log output the same
        # way the omega table gets tuned.
        #########################################################

        self.linear_ff_file = Path.home() / "linear_feedforward.yaml"

        if self.linear_ff_file.exists():

            with open(self.linear_ff_file, "r") as f:
                self.linear_ff_table = yaml.safe_load(f)

        else:

            self.linear_ff_table = {

                0.00: {"pwm":1500,"gain":0.0014,"settle":0.70,"samples":0},

                0.15: {"pwm":1605,"gain":0.0037,"settle":0.90,"samples":0},

                0.36: {"pwm":1640,"gain":0.0057,"settle":1.10,"samples":0},

                0.60: {"pwm":1685,"gain":0.0057,"settle":1.40,"samples":0},

                0.90: {"pwm":1735,"gain":0.0055,"settle":1.70,"samples":0},

                1.15: {"pwm":1785,"gain":0.0050,"settle":2.00,"samples":0},
            }

         #########################################################
       
        self.last_change_time = -100.0

        self.omega = 0.0

        self.omega_buffer = deque(maxlen=4)

        self.history = deque(maxlen=25)

        # FIX: initialise flag here, not via hasattr() in control loop
        self.ff_initialized = False

        #########################################################

        self.pub = self.create_publisher(
            OverrideRCIn,
            "/mavros/rc/override",
            10,
        )

        self.sub = self.create_subscription(
            Imu,
            "/mavros/imu/data",
            self.imu_callback,
            qos_profile_sensor_data,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self.odom_callback,
            qos_profile_sensor_data,
        )

        self.timer = self.create_timer(
            1.0 / 30.0,
            self.control_loop,
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_callback,
            10,
            )       

    #########################################################
    # IMU CALLBACK
    #########################################################

    def imu_callback(self, msg):

        now = self.get_clock().now().nanoseconds * 1e-9

        # Keep the existing IMU sign convention used by this controller.
        # cmd_vel sign correction is handled separately in cmd_callback().
        self.omega = -msg.angular_velocity.z

        self.omega_buffer.append(self.omega)

        self.history.append((now, self.omega))

    #########################################################
    # Moving average
    #########################################################

    def filtered_omega(self):

        if len(self.omega_buffer) == 0:
            return 0.0

        return sum(self.omega_buffer) / len(self.omega_buffer)

    #########################################################
    # ODOM CALLBACK (linear velocity feedback)
    #########################################################

    def odom_callback(self, msg):

        now = self.get_clock().now().nanoseconds * 1e-9

        # local_position/odom reports twist in the body frame,
        # so linear.x is forward speed -- exactly what we need
        # to close the loop on throttle.
        self.linear_velocity = msg.twist.twist.linear.x

        self.linear_buffer.append(self.linear_velocity)

        self.linear_history.append((now, self.linear_velocity))

    def filtered_linear(self):

        if len(self.linear_buffer) == 0:
            return 0.0

        return sum(self.linear_buffer) / len(self.linear_buffer)

    #########################################################
    # Linear velocity control (feedforward table + gain-
    # scheduled feedback) -- same architecture as the omega
    # controller: feedforward_lookup gives a starting PWM,
    # then a dead-time-aware wait/settle check and a
    # hysteresis-based bang-bang loop trim it in closed loop.
    #########################################################

    def update_linear_control(self):

        now = self.get_clock().now().nanoseconds * 1e-9

        # No forward/reverse command -> go neutral and reset all
        # linear state so it doesn't carry over into the next
        # command, exactly like the omega side resets on a new
        # ff_initialized cycle.
        if abs(self.target_linear_mag) < 1e-3:
            self.throttle_pwm = 1500
            self.linear_control_state = 0
            self.linear_ff_initialized = False
            return

        # Need enough history before doing anything, same gate
        # used for omega (len(self.history) < 8).
        if len(self.linear_history) < 8:
            return

        v_meas   = self.filtered_linear()
        gradient = self.compute_gradient(self.linear_history)

        #####################################################
        # One-shot feedforward initialisation
        #####################################################

        if not self.linear_ff_initialized:

            offset_pwm, self.linear_current_gain = (
                self.linear_feedforward_lookup(self.target_linear_mag)
            )

            if self.target_linear_sign > 0:
                self.throttle_pwm = offset_pwm
            else:
                self.throttle_pwm = 3000 - offset_pwm

            self.throttle_pwm = int(np.clip(
                self.throttle_pwm,
                self.linear_pwm_min,
                self.linear_pwm_max,
            ))

            self.linear_last_change_time = now
            self.linear_ff_initialized = True
            self.linear_command_start_time = now

            self.get_logger().info(
                f"[LINEAR FF INIT] target={self.target_linear_mag:.3f} "
                f"PWM={self.throttle_pwm}"
            )

            return

        #####################################################
        # Adaptive waiting -- hold correction while still
        # settling from the last change, same as omega.
        #####################################################

        elapsed = now - self.linear_last_change_time
        feedback_period = 0.15

        if elapsed < feedback_period:
            return

        command_elapsed = now - self.linear_command_start_time

        target_signed = self.target_linear_sign * self.target_linear_mag
        error = target_signed - v_meas

        #########################################################
        # Ignore all feedback during actuator dead time
        #########################################################

        if command_elapsed < self.linear_ignore_gradient_time:

            self.get_logger().info(
                f"[LINEAR DEADTIME] "
                f"t={command_elapsed:.2f}s "
                f"v={v_meas:.3f} "
                f"PWM={self.throttle_pwm}"
            )

            return

        if self.linear_prev_pwm is None:
            self.linear_prev_pwm = self.throttle_pwm
            self.linear_prev_v = v_meas

        prev_error = target_signed - self.linear_prev_v

        moving_toward_target = (
            abs(error) < abs(prev_error)
            and
            np.sign(error) == np.sign(prev_error)
        )

        settled = abs(gradient) < self.linear_gradient_threshold
        timeout = elapsed >= self.linear_max_wait

        if (
            moving_toward_target
            and
            (not settled)
            and
            (not timeout)
        ):
            self.get_logger().info(
                f"[LINEAR WAIT]  v={v_meas:.3f}  "
                f"grad={gradient:.4f}  "
                f"error={error:.3f}  "
                f"PWM={self.throttle_pwm}"
            )

            self.linear_prev_v = v_meas
            self.linear_prev_pwm = self.throttle_pwm

            return

        #####################################################
        # Gain-scheduled feedback with hysteresis (Schmitt
        # trigger), same structure as the omega control_state
        # machine.
        #####################################################

        band = max(
            self.linear_band,
            0.08 * abs(self.target_linear_mag),
        )

        enter_band = band
        exit_band = 0.25 * band

        if self.linear_control_state == 0:

            if v_meas > target_signed + enter_band:
                self.linear_control_state = 1

            elif v_meas < target_signed - enter_band:
                self.linear_control_state = -1

        elif self.linear_control_state == 1:

            if v_meas < target_signed + exit_band:
                self.linear_control_state = 0

        elif self.linear_control_state == -1:

            if v_meas > target_signed - exit_band:
                self.linear_control_state = 0

        #direction = self.linear_control_state

        


        gain = max(self.linear_current_gain, self.min_gain)

        delta_pwm = self.feedback_scale * abs(error) / gain
        delta_pwm = int(round(delta_pwm))

        # Always initialize direction from the hysteresis state
        direction = self.linear_control_state

        if abs(error) < band:
            direction = 0

        elif abs(error) < 2 * band:
            direction = self.linear_control_state
            delta_pwm = 1

        elif abs(error) < 4 * band:
            direction = self.linear_control_state
            delta_pwm = 2

        else:
            direction = self.linear_control_state
            delta_pwm = min(delta_pwm, self.max_pwm_step)

        delta_pwm = min(self.max_pwm_step, delta_pwm)
        changed = False

        # gain = max(self.linear_current_gain, self.min_gain)
        # delta_pwm = self.feedback_scale * abs(error) / gain
        # delta_pwm = int(round(delta_pwm))
# 
        # if abs(error) < band:
            # direction = 0
# 
        # elif abs(error) < 2 * band:
            # NOTE: uses self.linear_control_state (the value just
            # computed above), not an undefined "previous_direction"
            # -- the omega version of this branch references a name
            # that's never assigned anywhere and will raise
            # NameError the first time this branch is hit; fixed
            # here rather than carried over.
            # direction = self.linear_control_state
            # delta_pwm = 1

        # elif abs(error) < 4 * band:
            # delta_pwm = 2
# 
        # else:
            # delta_pwm = min(delta_pwm, self.max_pwm_step)
# 
        # delta_pwm = min(self.max_pwm_step, delta_pwm)
# 
        # changed = False

        # if direction != 0:
        #     self.throttle_pwm += direction * delta_pwm
        #     changed = True

        if direction != 0:
            if direction == 1:
                # Too fast → reduce throttle
                self.throttle_pwm -= delta_pwm
            elif direction == -1:
                # Too slow → increase throttle
                self.throttle_pwm += delta_pwm

            changed = True


        if changed:

            self.throttle_pwm = int(np.clip(
                self.throttle_pwm,
                self.linear_pwm_min,
                self.linear_pwm_max,
            ))

            self.linear_last_change_time = now

        self.linear_prev_v = v_meas
        self.linear_prev_pwm = self.throttle_pwm

        self.get_logger().info(
            f"[LINEAR CTRL]  v={v_meas:.3f}  "
            f"grad={gradient:.4f}  "
            f"PWM={self.throttle_pwm}"
        )

    def save_feedforward_table(self):

        with open(self.ff_file, "w") as f:

            yaml.safe_dump(self.ff_table, f, sort_keys=True)

        with open(self.linear_ff_file, "w") as f:

            yaml.safe_dump(self.linear_ff_table, f, sort_keys=True)
    



    #########################################################
    ## CMD_VEL CALLBACK
    ##########################################################
    #
    #def cmd_callback(self, msg):
    #
    #    # Desired yaw rate (rad/s)
    #    self.target = abs(msg.angular.z)
    #
    #    # Optional:
        # Convert linear velocity command into throttle PWM.
        # Replace this mapping with your own if desired.
        #
        # self.throttle_pwm = int(np.clip(
        #     1500 + msg.linear.x * 250,
        #     1500,
        #     1900,
        # ))
    
    #########################################################
# CMD_VEL CALLBACK
#########################################################

   # def cmd_callback(self, msg):
   # 
   #     # Desired yaw rate
   #     self.target = abs(msg.angular.z)
   # 
   #   #  # Throttle
   #   #  if msg.linear.x > 0.0:
   #   #      self.throttle_pwm = 1680
   #   #  else:
   #   #      self.throttle_pwm = 1500
##
   #   #  self.target = abs(msg.angular.z)
#
   #     if msg.linear.x > 0.0:
   #         self.throttle_pwm = 1680
   #     else:
   #         self.throttle_pwm = 1500
#
   #     self.have_cmd = True
#
   #     new_target = abs(msg.angular.z)
#
   #     if abs(new_target - self.target) > 0.02:
   #         self.ff_initialized = False
#
   #     self.target = new_target

  #  def cmd_callback(self, msg):
#
  #       new_target = msg.angular.z
  #       # Reinitialize feedforward if target changes significantly
  #       if abs(new_target - self.target) > 0.02:
  #           self.ff_initialized = False
  #       self.target = new_target
  #       # Throttle
  #       if msg.linear.x > 0.0:
  #           self.throttle_pwm = 1680
  #       else:
  #           self.throttle_pwm = 1500
  #       self.have_cmd = True

    # def cmd_callback(self, msg):

    #     new_target = msg.angular.z

    #     # Reinitialize feedforward if target changes
    #     if abs(new_target - self.target) > 0.02:
    #         self.ff_initialized = False

    #     self.target = new_target

    #     if msg.linear.x > 0.0:
    #         self.throttle_pwm = 1680
    #     else:
    #         self.throttle_pwm = 1500

    #     self.have_cmd = True

    def cmd_callback(self, msg):

        # Flip the incoming ROS yaw-rate sign to match this rover's
        # steering/RC convention. Default cmd_angular_sign is -1.0.
        commanded_omega = self.cmd_angular_sign * msg.angular.z

        if commanded_omega >= 0.0:
            new_sign = 1
        else:
            new_sign = -1

        new_mag = abs(commanded_omega)
    
        if (
            abs(new_mag - self.target_mag) > 0.02
            or new_sign != self.target_sign
        ):
            self.ff_initialized = False
    
        self.target_sign = new_sign
        self.target_mag = new_mag
    
        # Keep this for compatibility with the rest of the code
        self.target = self.target_mag
    
        # Continuous target (m/s) for the linear controller.
        # Mirrors the omega sign/magnitude split above so the same
        # feedforward-table + mirror-about-1500 trick applies to
        # forward vs. reverse.
        if msg.linear.x >= 0.0:
            new_linear_sign = 1
        else:
            new_linear_sign = -1

        new_linear_mag = abs(msg.linear.x)

        if (
            abs(new_linear_mag - self.target_linear_mag) > self.linear_retrigger
            or new_linear_sign != self.target_linear_sign
        ):
            self.linear_ff_initialized = False

        self.target_linear_sign = new_linear_sign
        self.target_linear_mag = new_linear_mag

        # Keep for compatibility/logging
        self.target_linear = msg.linear.x

        self.have_cmd = True

    #########################################################
    # Least-squares gradient estimate  (rad/s²)
    #########################################################

    def compute_gradient(self, history=None):

        # Defaults to the omega history for backward compatibility;
        # pass self.linear_history to get the linear equivalent.
        if history is None:
            history = self.history

        if len(history) < 8:
            return 0.0

        t = np.array([p[0] for p in history])
        w = np.array([p[1] for p in history])

        t = t - t[0]

        A = np.vstack([t, np.ones(len(t))]).T

        slope, _ = np.linalg.lstsq(A, w, rcond=None)[0]

        return slope

    #########################################################
    # Feedforward: interpolate PWM from table
    #########################################################

    # def feedforward_pwm(self, target):

    #     keys = sorted(self.ff_table.keys())

    #     if target <= keys[0]:
    #         return self.ff_table[keys[0]]

    #     if target >= keys[-1]:
    #         return self.ff_table[keys[-1]]

    #     for i in range(len(keys) - 1):

    #         if keys[i] <= target <= keys[i + 1]:

    #             w1 = keys[i]
    #             w2 = keys[i + 1]

    #             p1 = self.ff_table[w1]
    #             p2 = self.ff_table[w2]

    #             ratio = (target - w1) / (w2 - w1)

    #             return int(round(p1 + ratio * (p2 - p1)))

    #     # Should never reach here, but return neutral as safe fallback
    #     return 1500

    def feedforward_lookup(self, target):

        keys = sorted(self.ff_table.keys())

        if target <= keys[0]:
            data = self.ff_table[keys[0]]
            return data["pwm"], data["gain"]

        if target >= keys[-1]:
            data = self.ff_table[keys[-1]]
            return data["pwm"], data["gain"]

        for i in range(len(keys) - 1):

            if keys[i] <= target <= keys[i + 1]:

                w1 = keys[i]
                w2 = keys[i + 1]

                d1 = self.ff_table[w1]
                d2 = self.ff_table[w2]

                ratio = (target - w1) / (w2 - w1)

                pwm = d1["pwm"] + ratio * (d2["pwm"] - d1["pwm"])

                gain = d1["gain"] + ratio * (d2["gain"] - d1["gain"])

                return int(round(pwm)), gain

        return 1500, 0.005

    def linear_feedforward_lookup(self, target):

        keys = sorted(self.linear_ff_table.keys())

        if target <= keys[0]:
            data = self.linear_ff_table[keys[0]]
            return data["pwm"], data["gain"]

        if target >= keys[-1]:
            data = self.linear_ff_table[keys[-1]]
            return data["pwm"], data["gain"]

        for i in range(len(keys) - 1):

            if keys[i] <= target <= keys[i + 1]:

                w1 = keys[i]
                w2 = keys[i + 1]

                d1 = self.linear_ff_table[w1]
                d2 = self.linear_ff_table[w2]

                ratio = (target - w1) / (w2 - w1)

                pwm = d1["pwm"] + ratio * (d2["pwm"] - d1["pwm"])

                gain = d1["gain"] + ratio * (d2["gain"] - d1["gain"])

                return int(round(pwm)), gain

        return 1500, 0.005



    # def update_feedforward(self, target, pwm, gain):

    #     keys = sorted(self.ff_table.keys())

    #     nearest = min(keys, key=lambda k: abs(k - target))

    #     self.ff_table[nearest]["pwm"] = (
    #         (1.0 - self.learning_alpha)
    #         * self.ff_table[nearest]["pwm"]
    #         + self.learning_alpha
    #         * pwm
    #     )

    #     self.ff_table[nearest]["gain"] = (
    #         (1.0 - self.learning_alpha)
    #         * self.ff_table[nearest]["gain"]
    #         + self.learning_alpha
    #         * gain
    #     )

    # def update_feedforward(
    #     self,
    #     target,
    #     pwm,
    #     gain,
    #     settle_time,
    # ):
    
    #     keys = sorted(self.ff_table.keys())
    
    #     nearest = min(
    #         keys,
    #         key=lambda k: abs(k - target)
    #     )
    
    #     entry = self.ff_table[nearest]
    
    #     n = entry["samples"]
    
    #     entry["samples"] = n + 1
    
    #     entry["pwm"] += (
    #         pwm - entry["pwm"]
    #     ) / (n + 1)
    
    #     entry["gain"] += (
    #         gain - entry["gain"]
    #     ) / (n + 1)
    
    #     entry["settle"] += (
    #         settle_time - entry["settle"]
    #     ) / (n + 1)
    
    #########################################################
    # Publish RC override
    #########################################################

    def send_pwm(self):

        msg = OverrideRCIn()

        msg.channels = [65535] * 18

        msg.channels[0] = self.current_pwm      # steering / yaw

        msg.channels[1] = self.throttle_pwm     # throttle

        self.pub.publish(msg)

    #########################################################
    # CONTROL LOOP  (30 Hz)
    #########################################################

    def control_loop(self):

        if not self.have_cmd:
            return
        
        

        now = self.get_clock().now().nanoseconds * 1e-9

        #####################################################
        # Linear velocity feedback runs every cycle,
        # independent of the yaw controller's own state
        # machine (deadtime / wait / ff-init), so throttle
        # keeps closing the loop even while yaw is waiting.
        #####################################################

        self.update_linear_control()

        #####################################################
        # Need enough history before doing anything
        #####################################################

        if len(self.history) < 8:
            self.send_pwm()
            return

        omega    = self.filtered_omega()
        gradient = self.compute_gradient(self.history)

        #####################################################
        # One-shot feedforward initialisation
        #####################################################

        # if not self.ff_initialized:

        #     self.current_pwm      = self.feedforward_pwm(self.target)
        #     self.last_change_time = now
        #     self.ff_initialized   = True

        #     self.send_pwm()

        #     self.get_logger().info(
        #         f"[FF INIT] target={self.target:.3f} "
        #         f"PWM={self.current_pwm}"
        #     )

        #     return

        if not self.ff_initialized:

            offset_pwm, self.current_gain = self.feedforward_lookup(
            self.target_mag
        )

            if self.target_sign > 0:
                self.current_pwm = offset_pwm
            else:
                self.current_pwm = 3000 - offset_pwm

            self.last_change_time = now
            self.ff_initialized = True

            self.command_start_time = now
            self.send_pwm()

            self.get_logger().info(
                f"[FF INIT] target={self.target:.3f} "
                f"PWM={self.current_pwm}"
            )
            target_signed = self.target_sign * self.target

            self.get_logger().info(
                f"target={target_signed:.3f} "
                f"omega={omega:.3f}"
            )
            return


        #####################################################
        # Adaptive waiting
        #
        # Hold correction while the rover is already
        # moving toward the target and hasn't settled yet.
        # A safety timeout prevents waiting forever.
        #####################################################

        elapsed             = now - self.last_change_time
        feedback_period = 0.15      # 150 ms

        if elapsed < feedback_period:
            self.send_pwm()
            return
        command_elapsed = now - self.command_start_time
        
        # error = self.target - self.filtered_omega()
        # omega = self.filtered_omega()

        omega = self.filtered_omega()

        target_signed = self.target_sign * self.target

        error = target_signed - omega
        #########################################################
# Ignore all feedback during actuator dead time
#########################################################

        if command_elapsed < self.ignore_gradient_time:
        
            self.send_pwm()

            self.get_logger().info(
                f"[DEADTIME] "
                f"t={command_elapsed:.2f}s "
                f"omega={omega:.3f} "
                f"PWM={self.current_pwm}"
            )

            return



        if self.prev_pwm is None:

            self.prev_pwm = self.current_pwm
            self.prev_omega = omega
        
        # moving_toward_target = (error * gradient) > 0
        # settled              = abs(gradient) < self.gradient_threshold
        # timeout              = elapsed >= self.max_wait


        #moving_toward_target = (error * gradient) > 0

        prev_error = target_signed - self.prev_omega

        moving_toward_target = (
            abs(error) < abs(prev_error)
            and
            np.sign(error) == np.sign(prev_error)
        )

        # Ignore gradient during actuator dead time
        # if command_elapsed < self.ignore_gradient_time:
        
        #     settled = False

        # else:
        
        #     settled = abs(gradient) < self.gradient_threshold

        settled = abs(gradient) < self.gradient_threshold

        timeout = elapsed >= self.max_wait

        if (
    moving_toward_target
    and
    (not settled)
    and
    (not timeout)
):
            self.get_logger().info(
                f"[WAIT]  omega={omega:.3f}  "
                f"grad={gradient:.4f}  "
                f"error={error:.3f}  "
                f"PWM={self.current_pwm}"
            )

            self.send_pwm()

            return

        #####################################################
        # Bang-bang feedback with deadband
        #
        # Increments current_pwm by ±self.step (default 1)
        # per control cycle → effective rate = step * 30 PWM/s
        #
        # Direction convention:
        #   current_pwm < 1500  →  turning one way
        #   current_pwm > 1500  →  turning the other way
        # Moving toward 1500 always reduces |omega|.
        #
        # Edge case: if current_pwm == 1500 exactly and
        # omega is still out of band, nothing changes this
        # cycle. This is acceptable; it resolves next cycle
        # via the feedforward re-init path if omega is large.
        #####################################################

        # upper = self.target + self.band
        # lower = self.target - self.band
       # target_signed = self.target_sign * self.target
        self.band = max(
                0.04,
                0.08 * abs(self.target_mag)
            )
        upper = target_signed + self.band
        lower = target_signed - self.band
        changed = False

        # if omega > upper:

        #     # Turning too fast → move PWM toward 1500
        #     if self.current_pwm < 1500:
        #         self.current_pwm += self.step
        #     elif self.current_pwm > 1500:
        #         self.current_pwm -= self.step

        #     changed = True

        # elif omega < lower:

        #     # Turning too slow → move PWM away from 1500
        #     if self.current_pwm < 1500:
        #         self.current_pwm -= self.step
        #     elif self.current_pwm > 1500:
        #         self.current_pwm += self.step

        #     changed = True

    #     if omega > upper:

    # # Turning too fast
    #         if self.target_sign > 0:
    #             self.current_pwm += self.step
    #         else:
    #             self.current_pwm -= self.step
        
    #         changed = True
        
    #     elif omega < lower:
        
    #         # Turning too slow
    #         if self.target_sign > 0:
    #             self.current_pwm -= self.step
    #         else:
    #             self.current_pwm += self.step
        
    #         changed = True 


            #########################################################
            # Gain-scheduled feedback
            #########################################################

        enter_band = self.band
        exit_band = 0.25 * self.band

        if self.control_state == 0:
        
            if omega > target_signed + enter_band:
                self.control_state = 1

            elif omega < target_signed - enter_band:
                self.control_state = -1

        elif self.control_state == 1:
        
            if omega < target_signed + exit_band:
                self.control_state = 0

        elif self.control_state == -1:
        
            if omega > target_signed - exit_band:
                self.control_state = 0

        direction = self.control_state
        
        
        
        
        gain = max(self.current_gain, self.min_gain)

        delta_pwm = self.feedback_scale * abs(error) / gain
        delta_pwm = int(round(delta_pwm))

        # IMPORTANT:
        # Always initialize direction first.
        direction = self.control_state

        if abs(error) < self.band:
            direction = 0

        elif abs(error) < 2 * self.band:
            # Keep the hysteresis state and use the smallest correction.
            direction = self.control_state
            delta_pwm = 1

        elif abs(error) < 4 * self.band:
            # Keep the existing control-state direction.
            direction = self.control_state
            delta_pwm = 2

        else:
            # Large error: use normal gain-scheduled correction.
            direction = self.control_state
            delta_pwm = min(delta_pwm, self.max_pwm_step)

        delta_pwm = min(self.max_pwm_step, delta_pwm)
        changed = False
                # gain = max(self.current_gain, self.min_gain)
        # delta_pwm = self.feedback_scale * abs(error) / gain
        # delta_pwm = int(round(delta_pwm))
        # delta_pwm = max(self.min_pwm_step, delta_pwm)
        # if abs(error) < self.band:
            # direction = 0
        
        # elif abs(error) < 2*self.band:
           # direction = previous_direction
            # delta_pwm = 1

        # elif abs(error) < 4*self.band:
        
            # delta_pwm = 2

        # else:
        # 
            # delta_pwm = min(delta_pwm,self.max_pwm_step)
        # delta_pwm = min(self.max_pwm_step, delta_pwm)
        # changed = False
        # if omega > upper:
        # 
            # Turning too fast
            # if self.target_sign > 0:
                # self.current_pwm += delta_pwm
            # else:
                # self.current_pwm -= delta_pwm
            # changed = True
        # elif omega < lower:
        # 
            # Turning too slow
            # if self.target_sign > 0:
                # self.current_pwm -= delta_pwm
            # else:
                # self.current_pwm += delta_pwm
            # changed = True

       # target_signed = self.target_sign * self.target
        # if omega > upper:

        #     direction = 1
        
        # elif omega < lower:
        
        #     direction = -1
        
        # else:
        
        #     direction = 0

    

        # NOTE: the PWM->omega slope has the SAME sign on both sides of ---> wrong comment maybe
        # 1500 (the feedforward mirror "3000 - offset_pwm" is an
        # odd-symmetric reflection about (1500, 0), and the derivative
        # of an odd-symmetric function is even -> same-sign slope
        # everywhere). So the correction direction must NOT be
        # multiplied by target_sign -- doing so inverted the
        # correction for negative-sign targets, which is what was
        # causing the runaway "switches direction and turns
        # aggressively" behavior: instead of correcting error, it was
        # driving omega further from target until PWM hit a rail.


            # PWM->omega slope is negative on both sides of 1500:
#
#   PWM decreases -> omega becomes more positive
#   PWM increases -> omega becomes more negative
#
# Therefore the correction direction must account for the
# sign of the requested omega.
#
# direction:
#   +1 = omega too high
#   -1 = omega too low
#
# target_sign:
#   +1 = positive omega
#   -1 = negative omega

# if direction != 0:
    # self.current_pwm += self.target_sign * direction * delta_pwm
    # changed = True
# 

        if direction != 0:
            self.current_pwm += self.target_sign * direction * delta_pwm
            changed = True

        
        # if changed:

        #     self.last_change_time = now

        #     self.learning_pwm_before = self.prev_pwm
        #     self.learning_pwm_after = self.current_pwm

        #     self.learning_omega_before = self.prev_omega
        #     self.pending_learning = True

        if changed:

            self.current_pwm = int(np.clip(
                self.current_pwm,
                1100,
                1900,
            ))

            self.last_change_time = now

            self.learning_pwm_before = self.prev_pwm
            self.learning_pwm_after = self.current_pwm

            self.learning_omega_before = self.prev_omega

            self.pending_learning = True

    
        # self.get_logger().info(
        #     f"[DEBUG] settled={settled} "
        #     f"pending={self.pending_learning} "
        #     f"changed={changed}"
        #     )    
        # 
        self.get_logger().info(
         f"[DEBUG] "
         f"grad={gradient:.4f} "
         f"thr={self.gradient_threshold:.4f} "
         f"settled={settled} "
         f"pending={self.pending_learning} "
         f"changed={changed}"
            )   
        if settled and self.pending_learning:

            delta_pwm = (
                self.learning_pwm_after -
                self.learning_pwm_before
            )

            delta_omega = (
                omega -
                self.learning_omega_before
            )
            


            if abs(delta_pwm) > 0:
            
                # estimated_gain = abs(delta_omega / delta_pwm)

                # self.update_feedforward(
                #     self.target_mag,
                #     self.current_pwm,
                #     estimated_gain,
                # # )
                # if abs(delta_pwm) < 2:
                #     return
               
               
                # estimated_gain = abs(
                #     delta_omega / delta_pwm
                # )
                
                # # settle_time = (
                # #     now -
                # #     self.last_change_time
                # # )
                # settle_time = now - self.command_start_time
                
                # self.update_feedforward(
                
                #     self.target_mag,
                
                #     self.current_pwm,
                
                #     estimated_gain,
                
                #     settle_time,
                
                # )

                # # self.get_logger().info(
                # #     f"[LEARN] "
                # #     f"gain={estimated_gain:.5f}"
                # # )

                # entry = self.ff_table[
                #     min(
                #         self.ff_table.keys(),
                #         key=lambda k: abs(k-self.target_mag)
                #     )
                # ]
                
                # self.get_logger().info(
                
                #     f"[LEARN] "
                
                #     f"PWM={entry['pwm']:.1f} "
                
                #     f"Gain={entry['gain']:.5f} "
                
                #     f"Settle={entry['settle']:.2f}s "
                
                #     f"Samples={entry['samples']}"
                
                # )

                # self.prev_pwm = self.current_pwm
                # self.prev_omega = omega
                
                # self.learning_pwm_before = self.current_pwm
                # self.learning_pwm_after = self.current_pwm
                # self.learning_omega_before = omega
                # self.pending_learning = False

                if abs(delta_pwm) < 2:

                    self.pending_learning = False

                else:
                
                    estimated_gain = abs(delta_omega / delta_pwm)

                   # estimated_gain = delta_omega / delta_pwm

                    settle_time = now - self.command_start_time

                    # self.update_feedforward(
                    #     self.target_mag,
                    #     self.current_pwm,
                    #     estimated_gain,
                    #     settle_time,
                    # )

                    entry = self.ff_table[
                        min(
                            self.ff_table.keys(),
                            key=lambda k: abs(k-self.target_mag)
                        )
                    ]

                    self.get_logger().info(
                    
                        f"[LEARN] "

                        f"PWM={entry['pwm']:.1f} "

                        f"Gain={entry['gain']:.5f} "

                        f"Settle={entry['settle']:.2f}s "

                        f"Samples={entry['samples']}"

                    )
                    self.prev_pwm = self.current_pwm
                    self.prev_omega = omega
                    self.learning_pwm_before = self.current_pwm
                    self.learning_pwm_after = self.current_pwm
                    self.learning_omega_before = omega
                    self.pending_learning = False

        # Track this cycle's state for next cycle's
        # moving_toward_target comparison. This must happen every
        # cycle (not just inside the settled/learning branch above),
        # otherwise prev_omega goes stale for many cycles, the WAIT
        # state machine misjudges whether we're still converging,
        # and corrections end up firing late and oversized -- which
        # is what was causing the omega dither between commands.
        self.prev_omega = omega
        self.prev_pwm = self.current_pwm

        self.send_pwm()

        # changed = False

        # if abs(error) > self.band:
        
        #     if error > 0:
        #         # Need more positive yaw
        #         if self.current_pwm > 1500:
        #             self.current_pwm -= self.step
        #         else:
        #             self.current_pwm -= self.step
        
        #     else:
        #         # Need more negative yaw
        #         if self.current_pwm < 1500:
        #             self.current_pwm += self.step
        #         else:
        #             self.current_pwm += self.step
        
        #     changed = True

        self.get_logger().info(
            f"[CTRL]  omega={omega:.3f}  "
            f"grad={gradient:.4f}  "
            f"PWM={self.current_pwm}"
        )


#########################################################
# MAIN
#########################################################

def main(args=None):

    rclpy.init(args=args)

    node = OmegaController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.save_feedforward_table()
    
        node.get_logger().info("Feedforward table saved.")
    
        node.destroy_node()
    
        rclpy.shutdown()

if __name__ == "__main__":
    main()
