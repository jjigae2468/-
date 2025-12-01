from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
import os

def generate_launch_description():

    # 경로 설정
    pkg_name = "simple_perception_cpp"      # perception + tracking 패키지명
    nav2_dir = "/opt/ros/humble/share/nav2_bringup/launch"   # nav2 설치경로
    stack_master_dir = "/home/forza/forza_ws/src/stack_master"

    costmap_yaml = os.path.join(
        "/home/forza/forza_ws/src/config",
        "costmap_params.yaml"
    )

    controller_yaml = os.path.join(
        "/home/forza/forza_ws/src/config",
        "controller_params.yaml"
    )

    planner_yaml = os.path.join(
        "/home/forza/forza_ws/src/config",
        "planner_params.yaml"
    )

    # ===========================
    # 1. BASE SYSTEM (RACE STACK)
    # ===========================
    base_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stack_master_dir, "base_system_launch.py")
        ),
        launch_arguments={
            "racecar_version": "OrinNano",
            "sim": "true",
            "map_dir": "small_hall",
            "map_name": "small_hall_orig"
        }.items()
    )

    # =================================
    # 2. PERCEPTION NODE (detect_cpp)
    # =================================
    detection_node = Node(
        package=pkg_name,
        executable="simple_detector_node",
        name="detect_cpp",
        output="screen",
        parameters=[]
    )

    # =====================================
    # 3. TRACKING NODE (tracking_cpp)
    # =====================================
    tracking_node = Node(
        package=pkg_name,
        executable="tracking_cpp",
        name="tracking_cpp",
        output="screen",
        parameters=[]
    )

    # =====================================
    # 4. NAV2 (planner + controller + costmap)
    # =====================================
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'bringup_launch.py')
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": controller_yaml,   # controller + planner + costmap 통합 yaml
            "autostart": "true"
        }.items()
    )

    # =====================================
    # 5. TWIST TO DRIVE (Ackermann interface)
    # =====================================
    twist_to_drive = Node(
        package="f1tenth_raceline_planner",
        executable="twist_to_drive.py",
        name="twist_to_drive",
        output="screen"
    )

    return LaunchDescription([
        base_system,
        detection_node,
        tracking_node,
        nav2_bringup,
        twist_to_drive
    ])
