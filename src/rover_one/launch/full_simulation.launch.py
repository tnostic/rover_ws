import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_one')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

    world_path = os.path.join(pkg_share, 'worlds', 'training_arena.world')
    xacro_file = os.path.join(pkg_share, 'urdf', 'rover.urdf.xacro')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')

    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # 1. Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # 2. Gazebo World
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # 3. Spawn robot on the green start pad (-4.25, 0)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rover_one',
            '-x', '-4.25', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    # 4. Live SLAM (delayed until odom TF exists)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'slam_params_file': slam_params,
        }.items()
    )
    delayed_slam = TimerAction(period=8.0, actions=[slam])

    # 5. Nav2 nodes started explicitly so params file is applied
    common = [nav2_params, {'use_sim_time': True}]

    nav2_nodes = [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', parameters=common, output='screen'),
        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', parameters=common, output='screen'),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', parameters=common, output='screen'),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', parameters=common, output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', parameters=common, output='screen'),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', parameters=common, output='screen'),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', parameters=common, output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation',
             parameters=[{
                 'use_sim_time': True,
                 'autostart': True,
                 'node_names': [
                     'controller_server',
                     'smoother_server',
                     'planner_server',
                     'behavior_server',
                     'bt_navigator',
                     'waypoint_follower',
                     'velocity_smoother',
                 ],
             }],
             output='screen'),
    ]
    delayed_nav2 = TimerAction(period=12.0, actions=nav2_nodes)

    # 6. RViz2
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    # 7. GPS waypoint follower (after Nav2 is active)
    gps_follower_node = TimerAction(
        period=22.0,
        actions=[
            Node(
                package='rover_one',
                executable='gps_waypoint_follower',
                name='gps_waypoint_follower',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        rsp_node,
        gazebo,
        spawn_entity,
        delayed_slam,
        delayed_nav2,
        rviz,
        gps_follower_node,
    ])
