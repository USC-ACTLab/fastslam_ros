from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
def generate_launch_description():
    ld = LaunchDescription([
        Node(
            package='slam_publisher',
            executable='create3',
            name='create3'
        ), 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('laser_filters'),
                    'examples',
                    'range_filter_example.launch.py'
                ])
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rplidar_ros'),
                    'launch',
                    'rplidar_a1_launch.py'
                ])
            ])
        )
    ])
    ld.add_action(
        ExecuteProcess(
            cmd=[
                [
                    FindExecutable(name="ros2"),
                    " service call ",
                    "/reset_pose ", 
                    "irobot_create_msgs/srv/ResetPose ",
                    '"{}"',
                ]
            ],
            shell=True,
        )
    )
    return ld