import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    
    # Percorso del modello SDF
    # Nota: Stiamo usando un percorso diretto per semplicità in questa fase di test
    # In produzione useremo la variabile d'ambiente GAZEBO_MODEL_PATH
    package_dir = get_package_share_directory('domino_project')
    sdf_path = os.path.join(package_dir, 'models', 'domino_rg', 'model.sdf')

    return LaunchDescription([
        # 1. Avvia Gazebo server + client
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # 2. Nodo per spawnare il domino
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'my_domino',  # Nome dell'oggetto in simulazione
                '-file', sdf_path,       # Percorso del file SDF
                '-x', '0.0',             # Posizione X
                '-y', '0.0',             # Posizione Y
                '-z', '0.5'              # Posizione Z (cade dall'alto)
            ],
            output='screen'
        ),
    ])