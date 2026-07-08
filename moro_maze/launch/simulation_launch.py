import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    moro_maze_dir = get_package_share_directory('moro_maze')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    
    ## FIND CONFIG FILES
    world_path = os.path.join(moro_maze_dir, 'worlds', 'default_gzsim.world')
    map_yaml_path = os.path.join(moro_maze_dir, 'maps', 'map.yaml')
    rviz_config_file = os.path.join(moro_maze_dir, 'rviz', 'config.rviz')

    ## GAZEBO
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -s -v2 {world_path}', 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'models'))

    ## SPAWN ROBOT
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='2.0')
    y_pose = LaunchConfiguration('y_pose', default='1.0')

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    map_to_odom_tf_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'rviz_launch.py')),
        launch_arguments={'namespace': "",
                        'use_namespace': "False",
                        'use_sim_time': use_sim_time,
                        'rviz_config': rviz_config_file}.items())

    global_planner_cmd = Node(
        package='moro_maze',
        executable='global_planner',
        name='global_planner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_yaml': map_yaml_path,
            'start_x': x_pose,
            'start_y': y_pose,
            'goal_x': 4.0,
            'goal_y': 4.0,
        }]
    )

    local_control_cmd = Node(
        package='moro_maze',
        executable='local_control',
        name='local_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cmd_vel_stamped': True,
        }]
    )

    return LaunchDescription([
        set_env_vars_resources, gzserver_cmd, #gzclient_cmd, # comment out gzclient_cmd to omit the graphical simulation and save performance
        spawn_turtlebot_cmd, robot_state_publisher_cmd, map_to_odom_tf_cmd,
        global_planner_cmd, local_control_cmd, rviz_cmd
    ])
