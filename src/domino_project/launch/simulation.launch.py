import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('domino_project')
    
    # 1. Percorso del file World
    world_file = os.path.join(pkg_dir, 'worlds', 'domino_table.world')

    # 2. Percorsi dei modelli SDF
    domino_rg_path = os.path.join(pkg_dir, 'models', 'domino_rg', 'model.sdf')
    domino_gb_path = os.path.join(pkg_dir, 'models', 'domino_gb', 'model.sdf')
    domino_br_path = os.path.join(pkg_dir, 'models', 'domino_br', 'model.sdf')

    # Percorso del file URDF del robot Panda
    robot_urdf_path = os.path.join(pkg_dir, 'urdf', 'simple_arm.urdf')

    return LaunchDescription([
        # Avvia Gazebo caricando il nostro mondo con il tavolo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawna Domino Rosso-Verde (Sopra il tavolo, che è alto 1m)
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', 'domino1', '-file', domino_rg_path, '-x', '0.2', '-y', '0.0', '-z', '1.05'],
            output='screen'
        ),

        # Spawna Domino Verde-Blu (Spostato un po')
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', 'domino2', '-file', domino_gb_path, '-x', '0.2', '-y', '0.2', '-z', '1.05'],
            output='screen'
        ),

        # Spawna Domino Blu-Rosso (Spostato ancora)
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', 'domino3', '-file', domino_br_path, '-x', '0.2', '-y', '-0.2', '-z', '1.05'],
            output='screen'
        ),

	Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=[
                '-entity', 'my_robot', 
                '-file', robot_urdf_path, 
                '-x', '0.0', 
                '-y', '0.0', 
                '-z', '1.0'
            ],
            output='screen'
        ),
    ])
