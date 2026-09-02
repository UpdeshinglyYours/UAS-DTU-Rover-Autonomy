import ast
import math
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_config(name):
    with (PACKAGE_ROOT / 'config' / name).open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def test_dual_filter_tf_ownership_and_inputs():
    local = load_config('localization.yaml')['ekf_filter_node'][
        'ros__parameters'
    ]
    rtk = load_config('dtu_rtk_localization.yaml')
    global_filter = rtk['ekf_filter_node_map']['ros__parameters']
    datum_path = PACKAGE_ROOT.parent / 'dtu_prior_map' / 'config' / (
        'navsat_datum.yaml'
    )
    with datum_path.open(encoding='utf-8') as stream:
        navsat = yaml.safe_load(stream)['navsat_transform']['ros__parameters']

    assert local['odom_frame'] == 'odom'
    assert local['base_link_frame'] == 'base_link'
    assert local['world_frame'] == 'odom'
    assert local['publish_tf'] is True
    assert local['odom0'] == '/genz/odometry'
    assert local['odom0_config'] == [
        True, True, True,
        False, False, False,
        False, False, False,
        False, False, False,
        False, False, False,
    ]
    assert local['odom0_differential'] is False
    assert local['odom0_relative'] is False
    assert local['imu0'] == '/mavros/imu/data'
    assert local['imu0_config'] == [
        False, False, False,
        True, True, True,
        False, False, False,
        False, False, False,
        True, True, True,
    ]
    assert local['imu0_relative'] is True
    assert global_filter['map_frame'] == 'map'
    assert global_filter['odom_frame'] == 'odom'
    assert global_filter['base_link_frame'] == 'base_link'
    assert global_filter['world_frame'] == 'map'
    assert global_filter['publish_tf'] is False
    assert global_filter['odom0'] == '/genz/odometry'
    assert global_filter['odom0_differential'] is True
    assert global_filter['odom1'] == '/odometry/gps'
    assert global_filter['odom1_config'] == [
        True, True, False,
        False, False, False,
        False, False, False,
        False, False, False,
        False, False, False,
    ]
    assert global_filter['imu0'] == '/mavros/imu/data'
    assert global_filter['imu0_config'] == [
        False, False, False,
        False, False, True,
        False, False, False,
        False, False, True,
        False, False, False,
    ]
    assert global_filter['imu0_relative'] is False
    assert navsat['wait_for_datum'] is True
    assert navsat['use_odometry_yaw'] is False


def test_mavros_raw_fix_uses_physical_gps_frame():
    mavros = load_config('mavros_dtu_gps_frame.yaml')
    global_position = mavros['/**/global_position']['ros__parameters']

    assert global_position == {'frame_id': 'gps_antenna'}


def test_datum_cancels_generated_grid_convergence():
    datum_path = PACKAGE_ROOT.parent / 'dtu_prior_map' / 'config' / (
        'navsat_datum.yaml'
    )
    origin_path = PACKAGE_ROOT.parent / 'dtu_prior_map' / 'maps' / (
        'dtu_map_origin.yaml'
    )
    with datum_path.open(encoding='utf-8') as stream:
        navsat = yaml.safe_load(stream)['navsat_transform']['ros__parameters']
    with origin_path.open(encoding='utf-8') as stream:
        origin = yaml.safe_load(stream)

    assert navsat['datum'][:2] == [28.7451419213, 77.1119513181]
    assert math.isclose(
        origin['datum_yaw_radians'],
        -origin['grid_convergence_radians'],
        abs_tol=1e-8,
    )
    assert math.isclose(
        navsat['datum'][2], origin['datum_yaw_radians'], abs_tol=1e-10
    )


def test_global_filter_initializes_xy_from_first_absolute_fix():
    global_filter = load_config('dtu_rtk_localization.yaml')[
        'ekf_filter_node_map'
    ]['ros__parameters']
    covariance = global_filter['initial_estimate_covariance']

    assert len(covariance) == 15 * 15
    diagonal = [covariance[index * 15 + index] for index in range(15)]
    assert diagonal == [
        1.0e6, 1.0e6, 1.0e-9, 1.0e-9, 1.0e-9,
        1.0, 1.0, 1.0, 1.0e-9, 1.0e-9,
        1.0e-9, 1.0, 1.0e-9, 1.0e-9, 1.0e-9,
    ]
    assert all(
        value == 0.0
        for index, value in enumerate(covariance)
        if index // 15 != index % 15
    )


def test_dtu_nav2_uses_hard_map_and_soft_global_preference():
    config = load_config('nav2_dtu_rtk_params.yaml')
    local = config['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = config['global_costmap']['global_costmap'][
        'ros__parameters'
    ]

    assert local['global_frame'] == 'odom'
    assert local['static_layer']['map_topic'] == '/dtu_static_map'
    assert 'road_preference_filter' not in local
    assert global_costmap['static_layer']['map_topic'] == '/dtu_static_map'
    assert global_costmap['road_preference_filter'][
        'filter_info_topic'
    ] == '/dtu_centerline_filter_info'
    assert global_costmap['road_preference_filter']['enabled'] is True
    expected_footprint = (
        '[[0.50, 0.45], [-0.50, 0.45], '
        '[-0.50, -0.45], [0.50, -0.45]]'
    )
    assert local['footprint'] == expected_footprint
    assert global_costmap['footprint'] == expected_footprint


def test_known_working_scan_pipeline_and_costmap_ranges():
    launch_path = PACKAGE_ROOT / 'launch' / 'lirovo.launch.py'
    tree = ast.parse(launch_path.read_text(encoding='utf-8'))
    input_argument = next(
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == 'DeclareLaunchArgument'
        and isinstance(call.args[0], ast.Constant)
        and call.args[0].value == 'input_cloud_topic'
    )
    input_default = next(
        keyword.value.value
        for keyword in input_argument.keywords
        if keyword.arg == 'default_value'
    )
    output_argument = next(
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == 'DeclareLaunchArgument'
        and isinstance(call.args[0], ast.Constant)
        and call.args[0].value == 'output_scan_topic'
    )
    output_default = next(
        keyword.value.value
        for keyword in output_argument.keywords
        if keyword.arg == 'default_value'
    )
    pointcloud_node = next(
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and isinstance(node.targets[0], ast.Name)
        and node.targets[0].id == 'pointcloud_to_laserscan'
    )
    parameters = next(
        keyword.value.elts[0]
        for keyword in pointcloud_node.keywords
        if keyword.arg == 'parameters'
    )
    scan_parameters = {}
    for key, value in zip(parameters.keys, parameters.values):
        try:
            scan_parameters[key.value] = ast.literal_eval(value)
        except ValueError:
            pass

    assert input_default == '/bf_lidar/point_cloud_out'
    assert output_default == '/scan'
    assert scan_parameters == {
        'target_frame': 'base_link',
        'transform_tolerance': 0.05,
        'min_height': 0.2,
        'max_height': 1.0,
        'angle_min': -0.6293,
        'angle_max': 0.6293,
        'angle_increment': 0.00872665,
        'scan_time': 0.1,
        'range_min': 1.5,
        'range_max': 41.0,
        'use_inf': True,
        'inf_epsilon': 1.0,
        'queue_size': 50,
    }

    config = load_config('nav2_dtu_rtk_params.yaml')
    local = config['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = config['global_costmap']['global_costmap'][
        'ros__parameters'
    ]
    local_scan = local['obstacle_layer']['scan']
    global_scan = global_costmap['obstacle_layer']['scan']
    assert local_scan == {
        'topic': '/scan',
        'data_type': 'LaserScan',
        'marking': True,
        'clearing': True,
        'obstacle_min_range': 1.5,
        'obstacle_max_range': 20.0,
        'raytrace_min_range': 1.5,
        'raytrace_max_range': 15.0,
        'min_obstacle_height': 0.0,
        'max_obstacle_height': 1.0,
        'inf_is_valid': True,
    }
    assert global_scan == {
        'topic': '/scan',
        'data_type': 'LaserScan',
        'marking': True,
        'clearing': True,
        'obstacle_min_range': 1.5,
        'obstacle_max_range': 30.0,
        'raytrace_min_range': 1.5,
        'raytrace_max_range': 30.0,
        'min_obstacle_height': 0.0,
        'max_obstacle_height': 1.0,
        'inf_is_valid': True,
    }
    assert local['update_frequency'] == 25.0
    assert local['publish_frequency'] == 10.0
    assert local['width'] == 8
    assert local['height'] == 8
    assert local['resolution'] == 0.04
    assert local['inflation_layer'] == {
        'plugin': 'nav2_costmap_2d::InflationLayer',
        'cost_scaling_factor': 3.0,
        'inflation_radius': 0.8,
    }


def test_controller_mppi_and_smac_baselines_are_preserved():
    config = load_config('nav2_dtu_rtk_params.yaml')
    controller = config['controller_server']['ros__parameters']
    follow_path = controller['FollowPath']
    planner = config['planner_server']['ros__parameters']

    assert controller['odom_topic'] == '/odometry/filtered'
    assert follow_path == {
        'plugin': 'nav2_mppi_controller::MPPIController',
        'time_steps': 56,
        'model_dt': 0.05,
        'batch_size': 2000,
        'vx_std': 0.3,
        'vy_std': 0.0,
        'wz_std': 0.3,
        'vx_max': 0.9,
        'vx_min': -0.2,
        'vy_max': 0.0,
        'wz_max': 0.9,
        'iteration_count': 1,
        'prune_distance': 2.5,
        'transform_tolerance': 0.1,
        'temperature': 0.2,
        'gamma': 0.018,
        'motion_model': 'DiffDrive',
        'visualize': False,
        'critics': [
            'ConstraintCritic', 'ObstaclesCritic', 'GoalCritic',
            'GoalAngleCritic', 'PathAlignCritic', 'PathFollowCritic',
            'PathAngleCritic', 'PreferForwardCritic',
            'VelocityDeadbandCritic',
        ],
        'ConstraintCritic': {'cost_weight': 7.0},
        'ObstaclesCritic': {
            'repulsion_weight': 4.0,
            'critical_weight': 25.0,
            'consider_footprint': True,
            'collision_cost': 100000.0,
            'collision_margin_distance': 0.25,
            'near_goal_distance': 0.5,
        },
        'GoalCritic': {
            'cost_weight': 5.0,
            'threshold_to_consider': 1.4,
        },
        'GoalAngleCritic': {
            'cost_weight': 3.0,
            'threshold_to_consider': 0.5,
        },
        'PathAlignCritic': {'cost_weight': 12.0},
        'PathFollowCritic': {'cost_weight': 8.0},
        'PathAngleCritic': {'cost_weight': 2.0},
        'PreferForwardCritic': {
            'enabled': True,
            'cost_weight': 25.0,
        },
        'VelocityDeadbandCritic': {
            'cost_weight': 20.0,
            'deadband_velocities': [0.2, 0.0, 0.0],
        },
    }
    assert planner['expected_planner_frequency'] == 5.0
    assert planner['GridBased']['plugin'] == (
        'nav2_smac_planner/SmacPlanner2D'
    )
    assert planner['GridBased']['tolerance'] == 0.5
    assert planner['GridBased']['allow_unknown'] is False
    assert planner['GridBased']['downsample_costmap'] is False
    assert planner['GridBased']['downsampling_factor'] == 1
    assert planner['GridBased']['cost_travel_multiplier'] == 3.0
    assert planner['GridBased']['use_final_approach_orientation'] is True


def test_dtu_launch_preserves_centerline_rewrites():
    text = (PACKAGE_ROOT / 'launch' / 'lirovo.launch.py').read_text(
        encoding='utf-8'
    )
    assert "'enable_centerline_tether',\n            default_value='true'" in text
    assert 'road_preference_filter.enabled' in text
    assert 'GridBased.' in text
    assert 'cost_travel_multiplier' in text


def test_launch_routes_each_parameter_file_to_the_correct_node():
    launch_path = PACKAGE_ROOT / 'launch' / 'lirovo.launch.py'
    tree = ast.parse(launch_path.read_text(encoding='utf-8'))
    assignments = {
        node.targets[0].id: node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
        and node.targets[0].id in {
            'base_footprint_transform', 'global_ekf',
            'gps_antenna_transform', 'navsat_transform'
        }
    }

    def first_parameter_name(assignment):
        parameters = next(
            keyword.value
            for keyword in assignment.keywords
            if keyword.arg == 'parameters'
        )
        return parameters.elts[0].id

    assert first_parameter_name(assignments['global_ekf']) == (
        'dtu_rtk_localization_path'
    )
    assert first_parameter_name(assignments['navsat_transform']) == (
        'dtu_navsat_datum_path'
    )

    base_footprint_arguments = next(
        keyword.value
        for keyword in assignments['base_footprint_transform'].keywords
        if keyword.arg == 'arguments'
    )
    assert [element.value for element in base_footprint_arguments.elts] == [
        '--x', '0', '--y', '0', '--z', '0',
        '--roll', '0', '--pitch', '0',
        '--yaw', '0.0',
        '--frame-id', 'base_link',
        '--child-frame-id', 'base_footprint',
    ]

    gps_antenna_arguments = next(
        keyword.value
        for keyword in assignments['gps_antenna_transform'].keywords
        if keyword.arg == 'arguments'
    )
    assert [element.value for element in gps_antenna_arguments.elts] == [
        '--x', '0.10', '--y', '0.05', '--z', '0.40',
        '--roll', '0', '--pitch', '0', '--yaw', '0',
        '--frame-id', 'base_footprint',
        '--child-frame-id', 'gps_antenna',
    ]

    dtu_autostart = next(
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == 'DeclareLaunchArgument'
        and isinstance(call.args[0], ast.Constant)
        and call.args[0].value == 'dtu_nav2_autostart'
    )
    default_value = next(
        keyword.value
        for keyword in dtu_autostart.keywords
        if keyword.arg == 'default_value'
    )
    assert isinstance(default_value, ast.Constant)
    assert default_value.value == 'false'

    launch_text = launch_path.read_text(encoding='utf-8')
    assert "'--frame-id', 'map'" not in launch_text
    assert "'--child-frame-id', 'odom'" not in launch_text
