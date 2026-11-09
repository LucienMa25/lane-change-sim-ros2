"""
simulation.launch.py — ROS2 launch file for lane_change_sim.

Usage:
  ros2 launch lane_change_sim simulation.launch.py
  ros2 launch lane_change_sim simulation.launch.py scenario_profile:=builtin_highway safety_mode:=conservative
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# RSS safety modes documented in the README (maps to the `rss_cost` weight).
RSS_COST_BY_MODE = {
    'conservative': 50.0,
    'standard': 30.0,
    'aggressive': 10.0,
}


def generate_launch_description():
    ld = LaunchDescription()

    # ── Declare arguments ──
    ld.add_action(DeclareLaunchArgument(
        'scenario_profile', default_value='builtin_highway',
        description='Scenario profile YAML name (without path)'))
    ld.add_action(DeclareLaunchArgument(
        'safety_mode', default_value='standard',
        description='RSS safety model: conservative (rss_cost=50), standard (30), aggressive (10)'))
    ld.add_action(DeclareLaunchArgument(
        'enable_robot_model', default_value='true',
        description='Enable URDF ego RobotModel in RViz'))
    ld.add_action(DeclareLaunchArgument(
        'start_delay', default_value='5.0',
        description='Delay in seconds before launching simulation nodes'))

    # Nodes are built only once the launch context exists, so the arguments above
    # can be resolved into concrete, correctly typed Python values.
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld


def launch_setup(context):
    pkg_dir = get_package_share_directory('lane_change_sim')

    # ── Resolve launch arguments ──
    scenario_profile = LaunchConfiguration('scenario_profile').perform(context)
    safety_mode = LaunchConfiguration('safety_mode').perform(context)
    enable_robot_model = LaunchConfiguration('enable_robot_model').perform(context)
    start_delay = float(LaunchConfiguration('start_delay').perform(context))

    if safety_mode not in RSS_COST_BY_MODE:
        raise RuntimeError(
            f"Unknown safety_mode '{safety_mode}'. "
            f"Choose one of: {', '.join(sorted(RSS_COST_BY_MODE))}")

    # ── Parameter files ──
    params_yaml = os.path.join(pkg_dir, 'config', 'params.yaml')
    scenario_yaml = os.path.join(pkg_dir, 'config', 'scenarios', scenario_profile + '.yaml')
    if not os.path.isfile(scenario_yaml):
        raise RuntimeError(f'Scenario profile not found: {scenario_yaml}')

    # ── RViz2 ──
    # Window size/position come from the 'Window Geometry' block in rviz/simulation.rviz.
    rviz_config = os.path.join(pkg_dir, 'rviz', 'simulation.rviz')
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        output='screen')


    # ── Scene node ──
    scene_node = Node(
        package='lane_change_sim', executable='scene_node', name='scene_node',
        output='screen',
        parameters=[
            params_yaml,
            scenario_yaml,
            {'publish_rate': 10.0},
        ])

    # ── Simulation node ──
    sim_node = Node(
        package='lane_change_sim', executable='simulation_node', name='simulation_node',
        output='screen',
        parameters=[
            params_yaml,
            {'ctrl_ts': 0.1},
            {'target_x': 200.0},
        ])

    # ── Planner node ──
    planner_node = Node(
        package='lane_change_sim', executable='planner_node', name='planner_node',
        output='screen',
        parameters=[
            params_yaml,
            {'target_lane_id': 2},
            {'target_x': 200.0},
            {'rss_cost': RSS_COST_BY_MODE[safety_mode]},
        ])

    # ── Robot state publisher (optional URDF model) ──
    xacro_file = os.path.join(pkg_dir, 'urdf', 'main_car', 'car.xacro')
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
            'publish_frequency': 30.0,
        }],
        remappings=[('joint_states', '/sim/joint_states')],
        condition=IfCondition(enable_robot_model))

    return [
        rviz_node,
        TimerAction(
            period=start_delay,
            actions=[scene_node, sim_node, planner_node, robot_state_publisher]),
    ]
