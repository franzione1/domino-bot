import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    panda_pkg = get_package_share_directory('panda_ros2_gazebo')
    my_pkg = get_package_share_directory('domino_project')
    
    # Percorsi File
    domino_rg = os.path.join(my_pkg, 'models', 'domino_rg', 'model.sdf')
    domino_gb = os.path.join(my_pkg, 'models', 'domino_gb', 'model.sdf')
    domino_br = os.path.join(my_pkg, 'models', 'domino_br', 'model.sdf')
    table_sdf = os.path.join(my_pkg, 'models', 'work_table', 'model.sdf')
    camera_sdf = os.path.join(my_pkg, 'models', 'camera_sensor', 'model.sdf')

    # Robot Panda Simulation
    panda_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(panda_pkg, 'launch', 'panda_simulation.launch.py'))
    )

    # Avvio Controller Pinza (CORREZIONE QUI: 'start' invece di 'active')
    spawn_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'panda_hand_controller', '--set-state', 'start'],
        output='screen'
    )

    # Spawners Oggetti
    spawn_table = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'work_table', '-file', table_sdf, '-x', '0.7', '-y', '0.0', '-z', '0.8'], output='screen')
    
    spawn_camera = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'camera_sensor', '-file', camera_sdf, '-x', '0.7', '-y', '0.0', '-z', '2.3', '-R', '0.0', '-P', '1.57', '-Y', '0.0'], output='screen')

    z_height = '1.32'
    spawn_domino1 = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'domino_rg', '-file', domino_rg, '-x', '0.5', '-y', '0.0', '-z', z_height], output='screen')
    spawn_domino2 = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'domino_gb', '-file', domino_gb, '-x', '0.5', '-y', '0.2', '-z', z_height], output='screen')
    spawn_domino3 = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'domino_br', '-file', domino_br, '-x', '0.5', '-y', '-0.2', '-z', z_height], output='screen')

    return LaunchDescription([
        panda_simulation,
        TimerAction(period=5.0, actions=[spawn_table]),
        TimerAction(period=6.0, actions=[spawn_camera]),
        TimerAction(period=8.0, actions=[spawn_domino1, spawn_domino2, spawn_domino3]),
        
        # SEQUENZA PINZA RITARDATA
        # Aumentiamo un po' il ritardo per dare tempo al controller manager di svegliarsi
        TimerAction(period=22.0, actions=[spawn_gripper_controller]),
    ])