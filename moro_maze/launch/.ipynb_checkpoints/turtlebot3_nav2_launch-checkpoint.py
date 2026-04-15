import os

os.environ["TURTLEBOT3_MODEL"] = "burger"
ros_distro = os.environ["ROS_DISTRO"]

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    moro_maze_dir = get_package_share_directory('moro_maze')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')

    ## FIND CONFIG FILES
    params_path = os.path.join(moro_maze_dir, 'params', 'nav2_params.yaml')
    rviz_config_file = os.path.join(moro_maze_dir, 'rviz', 'config.rviz')
    map_yaml_path = os.path.join(moro_maze_dir, 'maps', 'map.yaml')

    ## ROBOT DESCRIPTION
    namespace = ''; use_namespace = "False"
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    robot_name = TURTLEBOT3_MODEL
    urdf = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                             'urdf', f"turtlebot3_{TURTLEBOT3_MODEL}.urdf")

    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    robot_sdf = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models', model_folder, 'model.sdf')
    
    with open(urdf, 'r') as infp: robot_description = infp.read()
    
    ## RVIZ
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'rviz_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file}.items())

    ## RUN NAV2
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': "True",
                          'map': map_yaml_path,
                          'use_sim_time': "False",
                          'params_file': params_path,
                          'autostart': "True",
                          'use_composition': "True",
                          'use_respawn': "False"}.items())

    return LaunchDescription([
        rviz_cmd,
        bringup_cmd,
    ])