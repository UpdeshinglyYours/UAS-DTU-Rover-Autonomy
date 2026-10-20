#!/usr/bin/env python3
"""rover_console2: rover_console + TELEOP / AUTONOMY drive-mode keys for new_omega_fixed6.py.

  python3 ~/vortex_ws/vortex_mission/vortex_mission/rover_console2.py   # interactive
  (rover_console / estop are unchanged; this copy is run by path, no setup.py entry)

Keys (no Enter needed):
  SPACE or h  E-STOP: ArduRover HOLD + abort mission + cancel all goals + zero /cmd_vel
  o           start the orbit at the casualty the rover is in STANDBY at (= RC flick)
  k           skip the current casualty
  p           pause after the current casualty
  g           go / resume the mission
  t           TELEOP: new_omega_fixed6 releases CH1/CH2, transmitter sticks drive (stays MANUAL)
  a a         AUTONOMY: new_omega_fixed6 tracks Nav2 /cmd_vel again (press twice)
  m m         back to MANUAL (press twice). Then restart new_omega_fixed6.py: it exits on HOLD.
  q           quit the console (does NOT stop the rover)

E-STOP order: HOLD first (the physical stop -- ArduRover holds, and new_omega_fixed6.py
centres its RC override and exits), then everything on the ROS side so nothing tries to
drive again: casualty_manager abort, cancel Nav2 / Spin / Orbit goals, zero /cmd_vel.
"""

import select
import sys
import termios
import time
import tty

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from vortex_interfaces.msg import RoverState
from vortex_interfaces.srv import StartOrbit

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)
STATE_NAMES = {RoverState.TRAVEL: 'TRAVEL', RoverState.STANDBY: 'STANDBY',
               RoverState.ORBIT: 'ORBIT', RoverState.IDLE: 'IDLE'}
CANCEL_ACTIONS = ['/navigate_to_pose', '/navigate_through_poses', '/follow_path',
                  '/compute_path_to_pose', '/spin', '/backup', '/wait', '/orbit_casualty']


class RoverConsole(Node):

    def __init__(self):
        super().__init__('rover_console')
        self.rover_state = None
        self.mode = '?'
        self.armed = None
        self.drive_mode = None
        self.create_subscription(RoverState, '/rover_state', self._on_state, LATCHED)
        self.create_subscription(String, '/drive_mode', self._on_drive_mode, LATCHED)
        self.drive_mode_req = self.create_publisher(String, '/drive_mode/request', 10)
        self.create_subscription(State, '/mavros/state', self._on_mavros, 10)
        self.set_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.start_orbit = self.create_client(StartOrbit, '/casualty_manager/start_orbit')
        self.trig = {n: self.create_client(Trigger, f'/casualty_manager/{n}')
                     for n in ('abort', 'skip_current', 'pause', 'start')}
        self.cancel = {a: self.create_client(CancelGoal, f'{a}/_action/cancel_goal')
                       for a in CANCEL_ACTIONS}
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _on_state(self, msg):
        self.rover_state = msg

    def _on_mavros(self, msg):
        self.mode, self.armed = msg.mode, msg.armed

    def _on_drive_mode(self, msg):
        self.drive_mode = msg.data

    def say(self, text):
        sys.stdout.write('\r\033[K' + text + '\r\n')
        sys.stdout.flush()

    def _log_result(self, label, future, ok_fn):
        def done(f):
            try:
                self.say(f'  {label}: {ok_fn(f.result())}')
            except Exception as e:      # noqa: BLE001
                self.say(f'  {label}: failed ({e})')
        future.add_done_callback(done)

    # ------------------------------------------------------------------ actions
    def estop(self):
        self.say('*** E-STOP ***')
        # 1. physical stop first
        if self.set_mode.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'HOLD'
            self._log_result('HOLD', self.set_mode.call_async(req),
                             lambda r: 'sent' if r.mode_sent else 'REJECTED by MAVROS')
        else:
            self.say('  HOLD: /mavros/set_mode NOT AVAILABLE -- is MAVROS running? '
                     'Use the RC/GCS or power cut.')
        # 2. stop anything that would command motion again
        if self.trig['abort'].service_is_ready():
            self._log_result('mission abort', self.trig['abort'].call_async(Trigger.Request()),
                             lambda r: r.message)
        for name, cli in self.cancel.items():
            if cli.service_is_ready():
                cli.call_async(CancelGoal.Request())   # empty request = cancel ALL goals
        for _ in range(5):
            self.cmd_pub.publish(Twist())
        self.say('  goals cancelled, /cmd_vel zeroed. Rover should be in HOLD.')

    def orbit(self):
        st = self.rover_state
        if st is None or st.state != RoverState.STANDBY:
            self.say(f'  orbit: rover is not in STANDBY ({self.state_text()})')
            return
        if not self.start_orbit.service_is_ready():
            self.say('  orbit: /casualty_manager/start_orbit not available')
            return
        req = StartOrbit.Request()
        req.casualty_id = st.casualty_id
        self._log_result(f'start_orbit({st.casualty_id})', self.start_orbit.call_async(req),
                         lambda r: ('accepted' if r.accepted else 'REJECTED') + f' - {r.message}')

    def trigger(self, name):
        cli = self.trig[name]
        if not cli.service_is_ready():
            self.say(f'  {name}: casualty_manager not available')
            return
        self._log_result(name, cli.call_async(Trigger.Request()), lambda r: r.message)

    def request_drive_mode(self, mode):
        if self.drive_mode_req.get_subscription_count() == 0:
            self.say(f'  {mode.upper()}: nobody listens on /drive_mode/request -- '
                     'is new_omega_fixed6.py running?')
            return
        self.drive_mode_req.publish(String(data=mode))
        self.say(f'  {mode.upper()} requested')

    def manual(self):
        if not self.set_mode.service_is_ready():
            self.say('  MANUAL: /mavros/set_mode not available')
            return
        req = SetMode.Request()
        req.custom_mode = 'MANUAL'
        self._log_result('MANUAL', self.set_mode.call_async(req),
                         lambda r: ('sent' if r.mode_sent else 'REJECTED') +
                         ' -- now restart new_omega_fixed6.py to drive')

    def state_text(self):
        st = self.rover_state
        s = 'no /rover_state' if st is None else \
            f'{STATE_NAMES.get(st.state, st.state)}' + \
            (f' casualty {st.casualty_id}' if st.state != RoverState.IDLE else '')
        drive = 'no /drive_mode (fixed6 not running)' if self.drive_mode is None \
            else self.drive_mode.upper()
        return f'mode={self.mode} armed={self.armed} drive={drive} | mission: {s}'


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def main_estop(args=None):
    """One-shot: E-STOP and exit."""
    rclpy.init(args=args)
    node = RoverConsole()
    spin_for(node, 1.0)            # discover services
    node.estop()
    spin_for(node, 2.0)            # let the calls finish and print results
    node.destroy_node()
    rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RoverConsole()
    if not sys.stdin.isatty():
        print('rover_console needs an interactive terminal (use `estop` for scripts).')
        return
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    pending_manual = 0.0
    pending_autonomy = 0.0
    last_status = ''
    print(__doc__.split('Keys')[1].split('E-STOP order')[0])
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            status = node.state_text()
            if status != last_status:
                node.say(f'[{status}]')
                last_status = status
            if not select.select([sys.stdin], [], [], 0.0)[0]:
                continue
            key = sys.stdin.read(1)
            if key in (' ', 'h', 'H'):
                node.estop()
            elif key == 'o':
                node.orbit()
            elif key == 'k':
                node.trigger('skip_current')
            elif key == 'p':
                node.trigger('pause')
            elif key == 'g':
                node.trigger('start')
            elif key == 'm':
                if time.monotonic() - pending_manual < 2.0:
                    node.manual()
                    pending_manual = 0.0
                else:
                    pending_manual = time.monotonic()
                    node.say('  press m again within 2 s to switch to MANUAL')
            elif key == 't':
                node.request_drive_mode('teleop')
            elif key == 'a':
                if time.monotonic() - pending_autonomy < 2.0:
                    node.request_drive_mode('autonomy')
                    pending_autonomy = 0.0
                else:
                    pending_autonomy = time.monotonic()
                    node.say('  press a again within 2 s to hand driving to AUTONOMY (Nav2)')
            elif key == 'q':
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
