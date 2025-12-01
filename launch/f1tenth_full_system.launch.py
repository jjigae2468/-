from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_name = "simple_perception_cpp"
    pkg_share = get_package_share_directory(pkg_name)

    costmap_yaml = os.path.join(pkg_share, "config", "costmap_params.yaml")
    controller_yaml = os.path.join(pkg_share, "config", "controller_params.yaml")

    nav2_dir = "/opt/ros/humble/share/nav2_bringup/launch"

    detection_node = Node(
        package=pkg_name,
        executable="simple_detector_node",
        output="screen"
    )

    tracking_node = Node(
        package=pkg_name,
        executable="tracking_cpp",
        output="screen"
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": controller_yaml,
            "autostart": "true"
        }.items()
    )

    twist_to_drive = Node(
        package="f1tenth_raceline_planner",
        executable="twist_to_drive.py",
        output="screen"
    )
    
    base_system = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("stack_master"),
            "base_system_launch.py"
        )
    ),
    launch_arguments={
        "racecar_version": "OrinNano",
        "map_dir": "small_hall",
        "map_name": "small_hall_orig",
        "sim": "true"
    }.items()
)
    

    return LaunchDescription([
        base_system,
        detection_node,
        tracking_node,
        nav2_bringup,
        twist_to_drive
    ])
