from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():

    ld = LaunchDescription()

    # Set env var to print messages to stdout immediately
    arg = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    ld.add_action(arg)

    xrc_node = Node(
        package='bluerov2common',
        executable='xrcClientStart.sh',
        name='xrcClientStart',
        output='screen',
        parameters=[],
        arguments=[]
    )
    ld.add_action(xrc_node)

    # camera_node = Node(
    #     package='v4l2_camera',
    #     executable='v4l2_camera_node',
    #     name='v4l2_camera_node',
    #     output='screen',
    #     parameters=[
    #         {"video_device": "/dev/video0"},
    #         {"pixel_format": "YUYV"},
    #         {"image_size": [640, 480]},
    #         {"time_per_frame": [1, 15]}
    #     ],
    #     arguments=[]
    # )
    # ld.add_action(camera_node)

    # pwm_node = Node(
    #     package='bluerov2common',
    #     executable='pwmServicesTopTube.py',
    #     name='pwmServices',
    #     output='screen',
    #     parameters=[],
    #     arguments=[]
    # )
    # ld.add_action(pwm_node)

    # leakage_node = Node(
    #     package='bluerov2common',
    #     executable='leakageSensorTopTube.py',
    #     name='leakageSensor',
    #     output='screen',
    #     parameters=[],
    #     arguments=[]
    # )
    # ld.add_action(leakage_node)

    # video_node = Node(
    #     package='bluerov2common',
    #     executable='mjpeg_cam_python.py',
    #     name='mjpeg_cam_python',
    #     output='screen',
    #     parameters=[],
    #     arguments=[]
    # )
    # ld.add_action(video_node)

    # mavroute_node = Node(
    #     package='bluerov2common',
    #     executable='xrcClientStart.sh',
    #     name='mavrouteStart',
    #     output='screen',
    #     parameters=[],
    #     arguments=[]
    # )
    # ld.add_action(mavroute_node)



    return ld
