# FR3 hardware bringup + MoveIt move_group + RViz, adapted for hand-eye calibration.
#
# This is an adapted copy of franka_fr3_moveit_config/launch/moveit.launch.py
# (Apache-2.0, Copyright Franka Robotics GmbH). Changes vs. upstream:
#   * load_gripper defaults to false and ee_id to 'none' (this setup has no hand).
#   * the bundled franka_gripper launch is included only when load_gripper:=true.
#   * RViz is toggleable (use_rviz) and its config is selectable (rviz_config),
#     so we can preload the HandEyeCalibration display instead of the stock config.
#
# All MoveIt parameters/SRDF/controllers are still read from the installed
# franka_fr3_moveit_config / franka_bringup / franka_description packages.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import yaml


def load_yaml(package_name, file_path):
    absolute_file_path = os.path.join(get_package_share_directory(package_name), file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    namespace = LaunchConfiguration('namespace')
    load_gripper = LaunchConfiguration('load_gripper')
    ee_id = LaunchConfiguration('ee_id')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    # --- robot_description (xacro -> URDF) ---
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_bringup'), 'urdf', 'franka_arm.urdf.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file,
         ' hand:=', load_gripper, ' robot_type:=fr3',
         ' robot_ip:=', robot_ip, ' ee_id:=', ee_id,
         ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands])
    robot_description = {'robot_description': ParameterValue(robot_description_config, value_type=str)}

    # --- robot_description_semantic (SRDF) ---
    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_description'), 'robots', 'fr3', 'fr3.srdf.xacro')
    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file,
         ' hand:=', load_gripper, ' ee_id:=', ee_id])
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(robot_description_semantic_config, value_type=str)}

    kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')
    kinematics_config = {'robot_description_kinematics': kinematics_yaml}

    joint_limits_yaml = load_yaml('franka_fr3_moveit_config', 'config/fr3_joint_limits.yaml')
    joint_limits_config = {'robot_description_planning': joint_limits_yaml}

    # --- Planning pipeline (OMPL) ---
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
            ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath',
            ],
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/fr3_controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # --- RViz (toggleable + selectable config; the HandEyeCalibration display
    #     needs the MoveIt parameters below to be passed in). ---
    default_rviz = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'), 'rviz', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_config,
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'config', 'fr3_ros_controllers.yaml')
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'franka/joint_states')],
        output={'stdout': 'screen', 'stderr': 'screen'},
        on_exit=Shutdown(),
    )

    load_controllers = []
    for controller in ['fr3_arm_controller', 'joint_state_broadcaster']:
        load_controllers.append(
            ExecuteProcess(
                cmd=['ros2', 'run', 'controller_manager', 'spawner', controller,
                     '--controller-manager-timeout', '60',
                     '--controller-manager',
                     PathJoinSubstitution([namespace, 'controller_manager'])],
                output='screen',
            ))

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[{'source_list': ['franka/joint_states', 'fr3_gripper/joint_states'],
                     'rate': 30}],
    )

    franka_robot_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['franka_robot_state_broadcaster'],
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
    )

    # Gripper is only relevant if a Franka Hand is attached.
    gripper_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
        launch_arguments={'robot_ip': robot_ip,
                          'use_fake_hardware': use_fake_hardware,
                          'namespace': namespace}.items(),
        condition=IfCondition(load_gripper),
    )

    args = [
        DeclareLaunchArgument('robot_ip', description='FCI IP of the FR3 (e.g. 169.254.67.230).'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='Optional namespace for the robot.'),
        DeclareLaunchArgument('load_gripper', default_value='false',
                              description='Whether a Franka Hand is attached.'),
        DeclareLaunchArgument('ee_id', default_value='none',
                              description='End-effector id: none | franka_hand | cobot_pump.'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false',
                              description='Use mock hardware instead of the real robot.'),
        DeclareLaunchArgument('fake_sensor_commands', default_value='false',
                              description='Fake sensor commands (only with use_fake_hardware).'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch RViz.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz,
                              description='Absolute path to the RViz config to load.'),
    ]

    return LaunchDescription(
        args + [
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            joint_state_publisher,
            franka_robot_state_broadcaster,
            rviz_node,
            gripper_launch_file,
        ] + load_controllers
    )
