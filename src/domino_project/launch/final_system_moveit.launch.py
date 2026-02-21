import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
import yaml

def generate_launch_description():
    panda_pkg = get_package_share_directory('panda_ros2_gazebo')
    my_pkg = get_package_share_directory('domino_project')
    
    # --- 1. CONFIGURAZIONE MOVEIT (URDF + SRDF) ---
    # Robustly locate URDF and SRDF across likely packages and fall back to local files.
    urdf_path = None
    srdf_path = None
    candidate_pkgs = [
        'moveit_resources_panda_moveit_config',
        'panda_moveit_config',
        'panda_ros2_gazebo',
        'panda_ros2_moveit2',
        'domino_project'
    ]

    for pkg_name in candidate_pkgs:
        try:
            pkg_share = get_package_share_directory(pkg_name)
        except Exception:
            continue
        urdf_candidate = os.path.join(pkg_share, 'urdf', 'panda.urdf')
        srdf_candidate = os.path.join(pkg_share, 'config', 'panda.srdf')
        if urdf_path is None and os.path.exists(urdf_candidate):
            urdf_path = urdf_candidate
        if srdf_path is None and os.path.exists(srdf_candidate):
            srdf_path = srdf_candidate
        if urdf_path and srdf_path:
            break


    # Search for XACRO in any candidate package (prefer xacro over raw URDF)
    xacro_path = None
    for pkg_name in candidate_pkgs:
        try:
            pkg_share = get_package_share_directory(pkg_name)
        except Exception:
            continue
        for candidate_name in ('panda.urdf.xacro', 'panda.xacro'):
            x_candidate = os.path.join(pkg_share, 'urdf', candidate_name)
            if os.path.exists(x_candidate):
                xacro_path = x_candidate
                break
        if xacro_path:
            break

    if urdf_path is None and xacro_path is None:
        raise FileNotFoundError('Could not find panda.urdf or panda.urdf.xacro in any candidate package. Searched: ' + ','.join(candidate_pkgs))

    # Read SRDF if available (optional)
    if srdf_path is None:
        robot_desc_semantic = ''
    else:
        with open(srdf_path, 'r') as f:
            robot_desc_semantic = f.read()

    # --- T02: Ensure SRDF virtual_joint child matches URDF root ---
    # Determine URDF root link by parsing joints (child links) and finding a link
    # that is not a child of any joint (the root). If SRDF's virtual_joint child
    # does not match, update the SRDF string in-memory so MoveIt sees a matching
    # virtual_joint.
    try:
        import xml.etree.ElementTree as ET
        # find URDF root link
        urdf_root = None
        try:
            urdf_xml = ET.fromstring(robot_desc)
            links = {l.attrib['name'] for l in urdf_xml.findall('link')}
            child_links = {j.find('child').attrib['link'] for j in urdf_xml.findall('joint') if j.find('child') is not None}
            candidates = links - child_links
            if candidates:
                # pick one candidate (there should be one root)
                urdf_root = sorted(list(candidates))[0]
        except Exception:
            urdf_root = None

        if robot_desc_semantic and urdf_root:
            try:
                srdf_xml = ET.fromstring(robot_desc_semantic)
                vj = srdf_xml.find("virtual_joint")
                if vj is not None:
                    child = vj.attrib.get('child_link') or vj.attrib.get('child')
                    if child and child != urdf_root:
                        # replace attribute name whichever exists
                        if 'child_link' in vj.attrib:
                            vj.attrib['child_link'] = urdf_root
                        else:
                            vj.attrib['child'] = urdf_root
                        # write back to string
                        robot_desc_semantic = ET.tostring(srdf_xml, encoding='unicode')
            except Exception:
                # if parsing fails, leave SRDF untouched
                pass
    except Exception:
        pass

    # Prefer generating URDF from XACRO (so we can set non-interactive args);
    # otherwise read URDF file directly.
    if xacro_path is not None:
        doc = xacro.parse(open(xacro_path))
        # Use pedestal layout by default: cell_layout_2 = true (select option 2)
        xacro.process_doc(doc, mappings={
            'cell_layout_1': 'false',
            'cell_layout_2': 'true',
            'EE_no': 'true',
        })
        robot_desc = doc.toxml()
    else:
        with open(urdf_path, 'r') as f:
            robot_desc = f.read()

    # --- 2. PERCORSI MODELLI SCENA ---
    domino_rg = os.path.join(my_pkg, 'models', 'domino_rg', 'model.sdf')
    domino_gb = os.path.join(my_pkg, 'models', 'domino_gb', 'model.sdf')
    domino_br = os.path.join(my_pkg, 'models', 'domino_br', 'model.sdf')
    table_sdf = os.path.join(my_pkg, 'models', 'work_table', 'model.sdf')
    camera_sdf = os.path.join(my_pkg, 'models', 'camera_sensor', 'model.sdf')

    # --- 3. NODI ---
    
    # Gazebo (non-interactive): include generic gazebo launch with specific world
    panda_world = os.path.join(panda_pkg, 'worlds', 'panda.world')
    gazebo_launch_pkg = get_package_share_directory('gazebo_ros')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(gazebo_launch_pkg, 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': panda_world}.items(),
    )

    # Static TF and robot_state_publisher so `robot_description` is available
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': robot_desc}],
    )

    # Spawn the robot from the `robot_description` topic (non-interactive)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'panda'],
        output='screen'
    )

    # Helper to load files from packages
    def load_file(package_name, file_path):
        try:
            pkg_path = get_package_share_directory(package_name)
        except Exception:
            return None
        abs_path = os.path.join(pkg_path, file_path)
        if not os.path.exists(abs_path):
            return None
        with open(abs_path, 'r') as f:
            return f.read()

    def load_yaml(package_name, file_path):
        try:
            pkg_path = get_package_share_directory(package_name)
        except Exception:
            return None
        abs_path = os.path.join(pkg_path, file_path)
        if not os.path.exists(abs_path):
            return None
        with open(abs_path, 'r') as f:
            return yaml.safe_load(f)

    # Load MoveIt configs from panda_ros2_moveit2 when available
    moveit_pkg = None
    try:
        moveit_pkg = get_package_share_directory('panda_ros2_moveit2')
    except Exception:
        moveit_pkg = None

    # Prepare parameters similar to the panda launch
    robot_description = {'robot_description': robot_desc}
    robot_description_semantic = {'robot_description_semantic': robot_desc_semantic} if robot_desc_semantic else {}

    # Kinematics
    kinematics_yaml = load_yaml('panda_ros2_moveit2', 'config/kinematics.yaml') or {}

    # OMPL planning pipeline
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_from_pkg = load_yaml('panda_ros2_moveit2', 'config/ompl_planning.yaml')
    if ompl_from_pkg:
        ompl_planning_pipeline_config['move_group'].update(ompl_from_pkg)

    # Also allow local tuning overrides in this package
    ompl_local = load_yaml('domino_project', 'config/ompl_tuning.yaml')
    if ompl_local:
        ompl_planning_pipeline_config['move_group'].update(ompl_local)

    # Controllers
    moveit_simple_controllers_yaml = load_yaml('panda_ros2_moveit2', 'config/panda_controllers.yaml') or {}
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # MoveIt MoveGroup (Il Server) - launch move_group directly with robot descriptions and configs
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': True},
        ],
    )
    # RViz (MoveIt configuration) - use panda_moveit2 rviz config when available
    try:
        rviz_config = os.path.join(get_package_share_directory('panda_ros2_moveit2'), 'config', 'panda_moveit2.rviz')
    except Exception:
        rviz_config = None
    # Fallback to package RViz config if the panda config isn't available
    if not rviz_config or not os.path.exists(rviz_config):
        candidate = os.path.join(my_pkg, 'config', 'domino_moveit.rviz')
        if os.path.exists(candidate):
            rviz_config = candidate

    rviz_node = None
    if rviz_config and os.path.exists(rviz_config):
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[robot_description, robot_description_semantic, ompl_planning_pipeline_config, kinematics_yaml],
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
        }, os.path.join(my_pkg, 'config', 'params.yaml')]
    )
    
    # Visione Python
    vision_node = Node(
        package='domino_project',
        executable='vision_processor.py', 
        output='screen'
    )

    # Test publisher for vision detections (used for automatic testing)
    vision_test_publisher = Node(
        package='domino_project',
        executable='vision_test_publisher.py',
        name='vision_test_publisher',
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
        gazebo,

        # bring up robot_state_publisher and static TF early, then spawn robot
        TimerAction(period=1.0, actions=[static_tf, robot_state_publisher, spawn_robot]),

        TimerAction(period=3.0, actions=[move_group]),
        # Start RViz a bit after move_group if available
        TimerAction(period=6.0, actions=[rviz_node] if rviz_node is not None else []),

        TimerAction(period=5.0, actions=[spawn_table]),
        TimerAction(period=6.0, actions=[spawn_camera]),
        TimerAction(period=8.0, actions=[spawn_domino1, spawn_domino2, spawn_domino3]),

        TimerAction(period=4.0, actions=[joint_state_broadcaster, panda_arm_controller]),
        TimerAction(period=8.0, actions=[panda_hand_controller]),

        # Avviamo il cervello C++ solo dopo che tutto è pronto
        TimerAction(period=12.0, actions=[robot_mover_node, vision_node, vision_test_publisher]),
    ])