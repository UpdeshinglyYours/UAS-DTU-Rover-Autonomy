"""Focused tests for manual alignment and fixed-only RTK integration."""

import ast
import math
from pathlib import Path

from mavros_msgs.msg import GPSRAW
from sensor_msgs.msg import NavSatFix
import yaml

from lirovo.manual_global_alignment import compose_map_to_odom
from lirovo.manual_global_alignment import MapOdomAuthority
from lirovo.rtk_fix_gate import GateLogic
from lirovo.rtk_fix_gate import antenna_to_base
from lirovo.rtk_fix_gate import horizontal_fix_variance
from lirovo.rtk_fix_gate import reentry_distance
from lirovo.rtk_fix_gate import timestamp_is_fresh


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_config(name):
    """Load one lirovo YAML configuration."""
    with (PACKAGE_ROOT / 'config' / name).open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def test_float_and_single_fixed_sample_are_rejected():
    logic = GateLogic(required_samples=10, max_variance=0.25)
    assert not logic.update_status(GPSRAW.GPS_FIX_TYPE_RTK_FLOAT, 0.1)
    assert not logic.fix_is_eligible(0.01)
    assert logic.update_status(GPSRAW.GPS_FIX_TYPE_RTK_FIXED, 0.1)
    assert logic.consecutive_fixed == 1
    assert not logic.fix_is_eligible(0.01)


def test_gate_requires_ten_good_fixed_samples_and_covariance():
    logic = GateLogic(required_samples=10, max_variance=0.25)
    for _index in range(9):
        logic.update_status(GPSRAW.GPS_FIX_TYPE_RTK_FIXED, 0.2)
        assert not logic.fix_is_eligible(0.04)
    logic.update_status(GPSRAW.GPS_FIX_TYPE_RTK_FIXED, 0.2)
    assert logic.fix_is_eligible(0.25)
    assert not logic.fix_is_eligible(0.250001)


def test_poor_reported_accuracy_resets_fixed_count():
    logic = GateLogic(required_samples=2, max_variance=0.25)
    logic.update_status(GPSRAW.GPS_FIX_TYPE_RTK_FIXED, 0.1)
    assert not logic.update_status(GPSRAW.GPS_FIX_TYPE_RTK_FIXED, 0.6)
    assert logic.consecutive_fixed == 0
    assert not logic.fix_is_eligible(0.01)


def test_stale_status_and_fix_timestamps_are_rejected():
    assert timestamp_is_fresh(100.0, 99.0, 1.5)
    assert not timestamp_is_fresh(100.0, 98.0, 1.5)
    assert not timestamp_is_fresh(100.0, 0.0, 1.5)
    assert not timestamp_is_fresh(100.0, 101.0, 1.5)


def test_reentry_distance_limit_and_rtk_loss():
    assert reentry_distance(1.0, 2.0, 1.0, 2.0) == 0.0
    assert reentry_distance(4.0, 6.0, 1.0, 2.0) == 5.0
    assert reentry_distance(4.0, 6.0, 1.0, 2.0) > 2.0

    logic = GateLogic(required_samples=1, max_variance=0.25)
    logic.update_status(GPSRAW.GPS_FIX_TYPE_RTK_FIXED, 0.1)
    logic.gate_open = True
    logic.update_status(GPSRAW.GPS_FIX_TYPE_RTK_FLOAT, 0.1)
    assert not logic.gate_open
    assert logic.consecutive_fixed == 0


def test_reentry_removes_the_configured_gps_lever_arm():
    base = antenna_to_base(
        candidate_x=10.05,
        candidate_y=20.10,
        base_yaw=math.pi / 2.0,
        base_to_gps_x=0.10,
        base_to_gps_y=0.05,
    )
    assert math.isclose(base[0], 10.10)
    assert math.isclose(base[1], 20.00)


def test_navsat_covariance_is_conservative_and_required():
    fix = NavSatFix()
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    assert horizontal_fix_variance(fix) is None
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
    fix.position_covariance[0] = 0.04
    fix.position_covariance[4] = 0.09
    assert math.isclose(horizontal_fix_variance(fix), 0.09)


def test_manual_alignment_preserves_clicked_position_and_yaw():
    # T_map_base = T_map_odom * T_odom_base.
    alignment = compose_map_to_odom(
        clicked_x=10.0,
        clicked_y=20.0,
        clicked_yaw=1.2,
        odom_x=3.0,
        odom_y=-2.0,
        odom_yaw=0.4,
    )
    map_odom_x, map_odom_y, map_odom_yaw = alignment
    cosine = math.cos(map_odom_yaw)
    sine = math.sin(map_odom_yaw)
    reconstructed_x = map_odom_x + cosine * 3.0 - sine * -2.0
    reconstructed_y = map_odom_y + sine * 3.0 + cosine * -2.0
    assert math.isclose(reconstructed_x, 10.0)
    assert math.isclose(reconstructed_y, 20.0)
    assert math.isclose(map_odom_yaw + 0.4, 1.2)


def test_map_odom_authority_is_unchanged_by_republication_rate():
    authority = MapOdomAuthority()
    expected = authority.set_manual(10.0, 20.0, 0.8)
    for _index in range(10000):
        assert authority.alignment == expected

    corrected = authority.apply_rtk_position(
        map_base_x=15.0,
        map_base_y=25.0,
        odom_base_x=3.0,
        odom_base_y=-2.0,
    )
    assert corrected[2] == expected[2]
    assert corrected != expected


def test_map_odom_authority_needs_yaw_for_rtk_only_initialization():
    authority = MapOdomAuthority()
    assert authority.apply_rtk_position(1.0, 2.0, 3.0, 4.0) is None
    alignment = authority.apply_rtk_position(
        1.0, 2.0, 3.0, 4.0, initial_yaw=0.3
    )
    assert alignment is not None
    assert alignment[2] == 0.3


def test_manual_node_uses_set_pose_and_is_the_tf_authority():
    path = PACKAGE_ROOT / 'lirovo' / 'manual_global_alignment.py'
    source = path.read_text(encoding='utf-8')
    assert "'/initialpose'" in source
    assert 'SetPose.Request()' in source
    assert 'TransformBroadcaster' in source
    assert 'sendTransform' in source
    assert "'gps_odometry_topic', '/odometry/gps'" in source
    assert 'self.authority.alignment' in source


def test_gate_defaults_and_topics():
    config = load_config('dtu_rtk_gate.yaml')
    gate = config['rtk_fix_gate']['ros__parameters']
    assert gate['required_consecutive_fixed_samples'] == 10
    assert gate['max_horizontal_variance_m2'] == 0.25
    assert gate['max_reentry_position_error_m'] == 2.0
    assert gate['enable_reentry_sanity_check'] is True
    assert gate['stale_status_timeout_sec'] == 1.5
    assert gate['stale_fix_timeout_sec'] == 1.5
    assert gate['gps_status_topic'] == '/mavros/gpsstatus/gps1/raw'
    assert gate['output_fix_topic'] == (
        '/mavros/global_position/rtk_fixed_only'
    )
    assert gate['map_frame'] == 'map'
    assert gate['base_frame'] == 'base_link'
    assert gate['gps_frame'] == 'gps_antenna'


def test_launch_has_distinct_tf_owners_and_gated_navsat_input():
    launch_path = PACKAGE_ROOT / 'launch' / 'lirovo.launch.py'
    source = launch_path.read_text(encoding='utf-8')
    tree = ast.parse(source)
    assert "('gps/fix', '/mavros/global_position/rtk_fixed_only')" in source
    assert "('odometry/filtered', '/odometry/global')" in source
    assert "('set_pose', '/ekf_filter_node_map/set_pose')" in source
    assert "('fromLL', '/navsat_transform/fromLL')" in source
    assert 'manual_global_alignment = Node(' in source
    assert 'rtk_fix_gate = Node(' in source
    assert "'--yaw', '0.0'" in source
    assert "'--x', '0.10'" in source
    assert "'--y', '0.05'" in source
    assert "'--z', '0.40'" in source
    assert "'--frame-id', 'map'" not in source
    assert "'--child-frame-id', 'odom'" not in source

    autostart = next(
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == 'DeclareLaunchArgument'
        and isinstance(call.args[0], ast.Constant)
        and call.args[0].value == 'dtu_nav2_autostart'
    )
    default = next(
        keyword.value.value
        for keyword in autostart.keywords
        if keyword.arg == 'default_value'
    )
    assert default == 'false'


def test_filter_tf_ownership_and_dead_reckoning_inputs_unchanged():
    local = load_config('localization.yaml')['ekf_filter_node'][
        'ros__parameters'
    ]
    global_filter = load_config('dtu_rtk_localization.yaml')[
        'ekf_filter_node_map'
    ]['ros__parameters']
    assert local['world_frame'] == 'odom'
    assert local['publish_tf'] is True
    assert global_filter['world_frame'] == 'map'
    assert global_filter['publish_tf'] is False
    assert global_filter['frequency'] == 30.0
    assert global_filter['odom0'] == '/genz/odometry'
    assert global_filter['odom0_differential'] is True
    assert global_filter['imu0'] == '/mavros/imu/data'
    assert global_filter['odom1'] == '/odometry/gps'
