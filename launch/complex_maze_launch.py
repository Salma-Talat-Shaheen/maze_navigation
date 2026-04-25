import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro 

def generate_launch_description():
    pkg_maze_navigation = get_package_share_directory('maze_navigation')
    
    # 1. تحديد ملف العالم (المسار المباشر لضمان العمل في WSL)
    world_file = '/home/shahd/ros2_project_ws/src/maze_navigation/worlds/complex_maze.world'
    
    # 2. تشغيل Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 3. معالجة ملف Xacro (نفس ملف الروبوت)
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

    # 4. إحضار الروبوت (Spawn) عند إحداثيات (1.0, 1.0) لتكون داخل المتاهة المعقدة
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

    # 5. الجسر (Bridge) باستخدام ملف الإعدادات الخاص بك
    bridge_config = os.path.join(pkg_maze_navigation, 'parameters', 'bridge_parameters.yaml')
    
    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        parameter_bridge
    ])