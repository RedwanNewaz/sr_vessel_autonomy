from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
import os
import yaml
from ament_index_python.packages import get_package_share_directory


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    current_pkg_dir = get_package_share_directory('sr_vessel_autonomy')

    param_file = os.path.join(current_pkg_dir, 'config', 'param.yaml')
    with open(param_file) as file:
        parameters = yaml.safe_load(file)
    # server_info = {
    #     "IP" : '192.168.0.50',
    #     "PORT": 8001
    # }

    # server_info = {
    #     "IP" : '127.0.0.1',
    #     "PORT": 8080
    # }

    sr_vessel_autonomy = Node(
        package='sr_vessel_autonomy',
        executable='sr_vessel_autonomy',
        namespace=namespace,
        name='sr_vessel_autonomy',
        parameters=[parameters],
        output='screen',
    )

    joy_node = Node(
        name='joy_node',
        package="joy",
        namespace=namespace,
        executable="joy_node"
    )

    teleop_node = Node(
        name = 'teleop_node',
        namespace=namespace,
        package = "teleop_twist_joy",
        executable="teleop_node"
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(sr_vessel_autonomy)
    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    return ld
