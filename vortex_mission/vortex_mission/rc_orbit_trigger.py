#!/usr/bin/env python3
"""rc_orbit_trigger: an RC switch flick starts the orbit (testing stand-in for the model script).

This is also the TEMPLATE for the model script. Its whole contract with the rover is:
  1. subscribe to /rover_state (latched)                      -> know when we're in STANDBY
  2. when state == STANDBY: do the work (here: wait for a flick; later: run the models)
  3. call /casualty_manager/start_orbit {casualty_id: <same id>}

Parameters
  orbit_channel  1-based RC channel of a free switch (0 = disabled). Avoid CH1-2 (driven by
                 new_omega_fixed5.py RC override), CH5 (MODE_CH) and channels with an
                 RCx_OPTION function. Find it with:  ros2 topic echo /mavros/rc/in
  skip_channel   optional 1-based channel whose flick calls ~/skip_current (0 = off)
"""

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from std_srvs.srv import Trigger

from mavros_msgs.msg import RCIn
from vortex_interfaces.msg import RoverState
from vortex_interfaces.srv import StartOrbit

from vortex_mission.rc_edge import RcEdgeDetector

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)


class RcOrbitTrigger(Node):

    def __init__(self):
        super().__init__('rc_orbit_trigger')
        self.orbit_channel = self.declare_parameter('orbit_channel', 0).value
        self.skip_channel = self.declare_parameter('skip_channel', 0).value
        high = self.declare_parameter('high_us', 1700).value
        low = self.declare_parameter('low_us', 1300).value
        debounce = self.declare_parameter('debounce_s', 0.3).value
        self.orbit_edge = RcEdgeDetector(high, low, debounce)
        self.skip_edge = RcEdgeDetector(high, low, debounce)
        self.state = None
        cb = ReentrantCallbackGroup()

        # ---- the model script needs exactly these two things ----
        self.create_subscription(RoverState, '/rover_state', self.on_state, LATCHED)
        self.start_orbit = self.create_client(StartOrbit, '/casualty_manager/start_orbit',
                                              callback_group=cb)
        # ---------------------------------------------------------

        self.skip = self.create_client(Trigger, '/casualty_manager/skip_current',
                                       callback_group=cb)
        self.create_subscription(RCIn, '/mavros/rc/in', self.on_rc, qos_profile_sensor_data)
        if self.orbit_channel <= 0:
            self.get_logger().error('orbit_channel is not set: flicks will be ignored. '
                                    'Set it to a free RC switch channel (1-based).')
        else:
            self.get_logger().info(f'flick RC channel {self.orbit_channel} in STANDBY to orbit')

    def on_state(self, msg: RoverState):
        if msg.state == RoverState.STANDBY and (self.state is None or
                                                self.state.state != RoverState.STANDBY):
            self.get_logger().info(f'STANDBY at casualty {msg.casualty_id}: '
                                   f'flick channel {self.orbit_channel} to start the orbit')
        self.state = msg

    def on_rc(self, msg: RCIn):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._flicked(self.orbit_edge, msg, self.orbit_channel, now):
            self.request_orbit()
        if self._flicked(self.skip_edge, msg, self.skip_channel, now):
            self.get_logger().info('skip flick')
            self.skip.call_async(Trigger.Request())

    @staticmethod
    def _flicked(edge: RcEdgeDetector, msg: RCIn, channel: int, now: float) -> bool:
        if channel <= 0 or channel > len(msg.channels):
            return False
        return edge.update(int(msg.channels[channel - 1]), now)

    def request_orbit(self):
        if self.state is None or self.state.state != RoverState.STANDBY:
            self.get_logger().warn('flick ignored: rover is not in STANDBY')
            return
        if not self.start_orbit.service_is_ready():
            self.get_logger().error('/casualty_manager/start_orbit not available')
            return
        req = StartOrbit.Request()
        req.casualty_id = self.state.casualty_id
        future = self.start_orbit.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'start_orbit({req.casualty_id}): accepted={f.result().accepted} '
                f'{f.result().message}'))


def main(args=None):
    rclpy.init(args=args)
    node = RcOrbitTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
