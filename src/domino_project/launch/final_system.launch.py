import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    panda_pkg = get_package_share_directory('panda_ros2_gazebo')
    my_pkg = get_package_share_directory('domino_project')
    
    # Percorsi Modelli
    domino_rg = os.path.join(my_pkg, 'models', 'domino_rg', 'model.sdf')
    domino_gb = os.path.join(my_pkg, 'models', 'domino_gb', 'model.sdf')
    domino_br = os.path.join(my_pkg, 'models', 'domino_br', 'model.sdf')
    table_sdf = os.path.join(my_pkg, 'models', 'work_table', 'model.sdf')
    camera_sdf = os.path.join(my_pkg, 'models', 'camera_sensor', 'model.sdf') # <-- NUOVO

    # Robot Panda
    panda_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(panda_pkg, 'launch', 'panda_simulation.launch.py'))
    )

    # 1. SPAWN TAVOLO (Lo avviciniamo a 0.7 invece di 0.9)
    spawn_table = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'work_table', '-file', table_sdf, '-x', '0.7', '-y', '0.0', '-z', '0.5'],
        output='screen'
    )
    
    # 2. SPAWN CAMERA (Deve seguire il tavolo, quindi 0.7)
    spawn_camera = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'camera_sensor', '-file', camera_sdf, '-x', '0.7', '-y', '0.0', '-z', '2.0', '-R', '0.0', '-P', '1.57', '-Y', '0.0'],
        output='screen'
    )

    # 3. SPAWN DOMINO (Erano a 0.7, li avviciniamo a 0.5 per tenerli al centro del tavolo)
    z_height = '1.02'
    spawn_domino1 = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'domino_rg', '-file', domino_rg, '-x', '0.5', '-y', '0.0', '-z', z_height], output='screen')
    spawn_domino2 = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'domino_gb', '-file', domino_gb, '-x', '0.5', '-y', '0.2', '-z', z_height], output='screen')
    spawn_domino3 = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'domino_br', '-file', domino_br, '-x', '0.5', '-y', '-0.2', '-z', z_height], output='screen')
    return LaunchDescription([
        panda_simulation,
        TimerAction(period=5.0, actions=[spawn_table]),
        TimerAction(period=6.0, actions=[spawn_camera]), # Spawna la camera
        TimerAction(period=8.0, actions=[spawn_domino1, spawn_domino2, spawn_domino3])
    ])
