import os

os.environ["TURTLEBOT3_MODEL"] = "burger"
model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
ros_distro = os.environ["ROS_DISTRO"]
os.environ["GAZEBO_MODEL_PATH"] = f"{model_path}:/opt/ros/{ros_distro}/share/turtlebot3_gazebo/models"

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

    ## TOGGLE GAZEBO DISPLAY FOR CONVENIENCE
    headless = LaunchConfiguration('headless')
    declare_headless_cmd = DeclareLaunchArgument('headless', default_value='True', description='Whether to execute gzclient)')

    ## FIND CONFIG FILES
    world_path = os.path.join(moro_maze_dir, 'worlds', 'default.world')
    map_yaml_path = os.path.join(moro_maze_dir, 'maps', 'map.yaml')
    params_path = os.path.join(moro_maze_dir, 'params', 'nav2_params.yaml')
    rviz_config_file = os.path.join(moro_maze_dir, 'rviz', 'config.rviz')

    ## GAZEBO
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', '--verbose', world_path],
        cwd=[nav2_launch_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(['not ', headless])),
        cmd=['gzclient'],
        cwd=[nav2_launch_dir], output='screen')

    ## ROBOT DESCRIPTION
    namespace = ''; use_namespace = "False"
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    robot_name = TURTLEBOT3_MODEL
    urdf = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                             'urdf', f"turtlebot3_{TURTLEBOT3_MODEL}.urdf")

    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    robot_sdf = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models', model_folder, 'model.sdf')
    
    with open(urdf, 'r') as infp: robot_description = infp.read()

    ## ROBOT STATE PUBLISHER
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        remappings=remappings)

    ## SPAWN MODEL
    # Always make sure to use strings here -> floats will cause weird errors
    namespace_spawner=namespace if use_namespace else '""' # TODO: find a better way
    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-file', robot_sdf,
            '-robot_namespace', namespace_spawner,
            '-x', '2.', '-y', '1.', '-z', '0.01',
            '-R', '0.', '-P', '0.', '-Y', '0.'])
    
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
                          'slam': "False",
                          'map': map_yaml_path,
                          'use_sim_time': "True",
                          'params_file': params_path,
                          'autostart': "True",
                          'use_composition': "True",
                          'use_respawn': "False"}.items())

    return LaunchDescription([
        declare_headless_cmd,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        start_robot_state_publisher_cmd,
        start_gazebo_spawner_cmd,
        rviz_cmd,
        bringup_cmd,
    ])