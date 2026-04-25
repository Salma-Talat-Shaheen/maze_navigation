import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro 

def generate_launch_description():
    pkg_maze_navigation = get_package_share_directory('maze_navigation')
    
    # 1. Define the path to the world file (Direct path for WSL compatibility)
    world_file = '/home/shahd/ros2_project_ws/src/maze_navigation/worlds/complex_maze.world'
    
    # 2. Include the Gazebo simulation launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 3. Process the Xacro file and initialize robot_state_publisher
    xacro_file = os.path.join(pkg_maze_navigation, 'urdf', 'turtlebot4.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_content = robot_description_config.toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}]
    )

    # 4. Spawn the robot at (1.0, 1.0) to start inside the complex maze
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot4',
            '-string', robot_description_content, 
            '-x', '1.0', '-y', '1.0', '-z', '0.2'
        ],
        output='screen',
    )

    # 5. Configure the parameter_bridge using the project-specific configuration file
    bridge_config = os.path.join(pkg_maze_navigation, 'parameters', 'bridge_parameters.yaml')
    
    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    # Return the full launch description
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        parameter_bridge
    ])
