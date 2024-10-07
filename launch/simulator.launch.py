import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import IncludeLaunchDescription, EmitEvent, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.events import matches_action


def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('expressive_eyes'), 'launch'), '/expressive_eyes_with_eyes_tf.launch.py'])
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('communication_hub'), 'launch'), '/communication_hub.launch.py']),
        launch_arguments={'transition_to_activate': 'True'}.items()
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('volume_control_pulseaudio'), 'launch'), '/volume.launch.py'])
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('hri_face_detect'), 'launch'), '/face_detect.launch.py'])
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('usb_cam'), 'launch'), '/camera.launch.py'])
    ))

    return ld
