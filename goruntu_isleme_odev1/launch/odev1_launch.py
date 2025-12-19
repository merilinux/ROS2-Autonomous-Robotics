import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    set_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi')
    
    # SENİN DÜNYAN
    world_config = '/home/meri/Desktop/odev1.world'

    # Gazebo'yu senin dünyanla açan ana launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch'),
            '/gazebo.launch.py'
        ]),
        launch_arguments={'world': world_config}.items()
    )

    # Robot Tanımı
    xml = open('/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf', 'r').read()
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'turtlebot3_waffle_pi', '-file', '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf', '-x', '0', '-y', '0', '-z', '0.01'],
        output='screen'
    )

    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch'),
            '/robot_state_publisher.launch.py'
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # HOCANIN İSTEDİĞİ 2 AYRI NODE
    vision_node = Node(
        package='goruntu_isleme',
        executable='vision_node',
        name='vision_node'
    )

    controller_node = Node(
        package='goruntu_isleme',
        executable='controller_node',
        name='controller_node'
    )

    return LaunchDescription([
        set_model,
        gazebo,
        robot_state_pub,
        spawn_robot,
        vision_node,
        controller_node
    ])
