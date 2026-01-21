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

    # Spawners Oggetti
    spawn_table = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'work_table', '-file', table_sdf, '-x', '0.7', '-y', '0.0', '-z', '0.8'], output='screen')
    
    spawn_camera = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'camera_sensor', '-file', camera_sdf, '-x', '0.7', '-y', '0.0', '-z', '2.3', '-R', '0.0', '-P', '1.57', '-Y', '0.0'], output='screen')

    z_height = '1.32'
    spawn_domino1 = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'domino_rg', '-file', domino_rg, '-x', '0.5', '-y', '0.0', '-z', z_height], output='screen')
    spawn_domino2 = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'domino_gb', '-file', domino_gb, '-x', '0.5', '-y', '0.2', '-z', z_height], output='screen')
    spawn_domino3 = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'domino_br', '-file', domino_br, '-x', '0.5', '-y', '-0.2', '-z', z_height], output='screen')
    
# --- SPAWNER PER IL BRACCIO E STATI ---
    
    # 1. Joint State Broadcaster (Pubblica gli stati dei giunti su /joint_states)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py", # Ricorda il .py per Foxy
        arguments=["joint_state_broadcaster"],
    )

    # 2. Arm Controller (Il controller principale del braccio)
    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["panda_arm_controller"],
    )

    # 3. Hand Controller (Quello che avevi già)
    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["panda_hand_controller"],
    )

    return LaunchDescription([
        panda_simulation,
        TimerAction(period=5.0, actions=[spawn_table]),
        TimerAction(period=6.0, actions=[spawn_camera]),
        TimerAction(period=8.0, actions=[spawn_domino1, spawn_domino2, spawn_domino3]),
        
# --- SEQUENZA DI AVVIO AUTOMATICA ---
        
        # STEP 1: Aspetta 5 secondi che Gazebo sia pronto, poi lancia gli stati e il braccio
        TimerAction(
            period=5.0,
            actions=[
                joint_state_broadcaster_spawner,
                panda_arm_controller_spawner
            ]
        ),

        # STEP 2: Aspetta altri secondi (totale 10 o più) e lancia la mano
        # (Oppure puoi metterli tutti insieme nel timer sopra, ma separarli a volte è più stabile)
        TimerAction(
            period=10.0,
            actions=[panda_hand_controller_spawner]
        ),
    ])