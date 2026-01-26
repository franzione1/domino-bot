import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    panda_pkg = get_package_share_directory('panda_ros2_gazebo')
    my_pkg = get_package_share_directory('domino_project')
    
    # --- 1. CONFIGURAZIONE MOVEIT (URDF + SRDF) ---
    # Cerchiamo il pacchetto di configurazione MoveIt installato
    try:
        moveit_config_pkg = get_package_share_directory('moveit_resources_panda_moveit_config')
    except:
        moveit_config_pkg = get_package_share_directory('panda_moveit_config')

    # Percorsi ai file di descrizione del robot
    # Nota: In alcuni pacchetti demo, l'URDF è direttamente in 'urdf/panda.urdf'
    urdf_path = os.path.join(moveit_config_pkg, 'urdf', 'panda.urdf')
    srdf_path = os.path.join(moveit_config_pkg, 'config', 'panda.srdf')
    
    # Leggiamo i file per passarli come parametri (stringhe)
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    
    with open(srdf_path, 'r') as f:
        robot_desc_semantic = f.read()

    # --- 2. PERCORSI MODELLI SCENA ---
    domino_rg = os.path.join(my_pkg, 'models', 'domino_rg', 'model.sdf')
    domino_gb = os.path.join(my_pkg, 'models', 'domino_gb', 'model.sdf')
    domino_br = os.path.join(my_pkg, 'models', 'domino_br', 'model.sdf')
    table_sdf = os.path.join(my_pkg, 'models', 'work_table', 'model.sdf')
    camera_sdf = os.path.join(my_pkg, 'models', 'camera_sensor', 'model.sdf')

    # --- 3. NODI ---
    
    # Gazebo
    panda_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(panda_pkg, 'launch', 'panda_simulation.launch.py'))
    )

    # MoveIt MoveGroup (Il Server)
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_config_pkg, 'launch', 'move_group.launch.py')),
        launch_arguments={'load_robot_description': 'false'}.items()
    )
    
    # IL TUO NODO C++ (Il Client)
    # Qui passiamo i parametri robot_description necessari!
    robot_mover_node = Node(
        package='domino_project',
        executable='robot_mover_cpp', 
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'robot_description_semantic': robot_desc_semantic
        }]
    )
    
    # Visione Python
    vision_node = Node(
        package='domino_project',
        executable='vision_processor.py', 
        output='screen'
    )

    # Spawners (Oggetti)
    spawn_table = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'work_table', '-file', table_sdf, '-x', '0.7', '-y', '0.0', '-z', '0.8'], output='screen')
    
    spawn_camera = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'camera_sensor', '-file', camera_sdf, '-x', '0.7', '-y', '0.0', '-z', '2.3', '-R', '0.0', '-P', '1.57', '-Y', '0.0'], output='screen')

    z_height = '1.32'
    spawn_domino1 = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'domino_rg', '-file', domino_rg, '-x', '0.5', '-y', '0.0', '-z', z_height], output='screen')
    spawn_domino2 = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'domino_gb', '-file', domino_gb, '-x', '0.5', '-y', '0.2', '-z', z_height], output='screen')
    spawn_domino3 = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'domino_br', '-file', domino_br, '-x', '0.5', '-y', '-0.2', '-z', z_height], output='screen')
    
    # Controllers
    joint_state_broadcaster = Node(package="controller_manager", executable="spawner.py", arguments=["joint_state_broadcaster"])
    panda_arm_controller = Node(package="controller_manager", executable="spawner.py", arguments=["panda_arm_controller"])
    panda_hand_controller = Node(package="controller_manager", executable="spawner.py", arguments=["panda_hand_controller"])

    return LaunchDescription([
        panda_simulation,
        
        TimerAction(period=3.0, actions=[move_group]),
        
        TimerAction(period=5.0, actions=[spawn_table]),
        TimerAction(period=6.0, actions=[spawn_camera]),
        TimerAction(period=8.0, actions=[spawn_domino1, spawn_domino2, spawn_domino3]),
        
        TimerAction(period=4.0, actions=[joint_state_broadcaster, panda_arm_controller]),
        TimerAction(period=8.0, actions=[panda_hand_controller]),

        # Avviamo il cervello C++ solo dopo che tutto è pronto
        TimerAction(period=12.0, actions=[robot_mover_node, vision_node]),
    ])