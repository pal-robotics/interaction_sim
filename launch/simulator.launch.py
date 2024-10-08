import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    rqt_cmd = ['rqt',             
            '--perspective-file', 
            os.path.join(
                get_package_share_directory('interaction_sim'),
                'config/simulator.perspective')]

    # Create the ExecuteProcess action
    rqt = ExecuteProcess(
        cmd=rqt_cmd,
        output='log',
        shell=False,
        on_exit=Shutdown()
    )

    ld.add_action(rqt)

    return ld
