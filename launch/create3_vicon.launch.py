from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
import launch.event_handlers
def generate_launch_description():
    number_of_landmarks = 0 #number of landmarks being streamed by vicon. They must be named landmark0, landmark1, landmark2, etc. 
                            #If you do not start with landmark zero, subscribers will not initialize properly
    vicon_visualization = Node(
        package = 'slam_publisher',
        executable='vicon_visualization',
        name = 'vicon_visualization',
        parameters = [
            {"number_of_landmarks": number_of_landmarks}
        ]
    )
    ld = LaunchDescription([
        vicon_visualization,
                IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('vicon_bridge'),
                    'launch',
                    'all_segments.launch.py'
                ])
            ])
        ),
        #The vicon_bridge will shut down if the vicon_visualization throws an error or shuts down
        RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=vicon_visualization,
                    on_exit=[
                        EmitEvent(event=launch.events.Shutdown()),
                    ],
                )
            )
    ])
    return ld