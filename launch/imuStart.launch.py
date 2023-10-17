from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():

    ld = LaunchDescription()

    # Set env var to print messages to stdout immediately
    # arg = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    # ld.add_action(arg)



    # parameters_file_path = Path(get_package_share_directory('bluespace_ai_xsens_mti_driver'), 'param', 'xsens_mti_node.yaml')

    imu_node = Node(
        package='bluespace_ai_xsens_mti_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        output='screen',
        parameters=[
            {"scan_for_devices": True},
            {"port": "/dev/ttyUSB0"},
            {"baudrate": 115200},
            {"pub_imu": True},
            {"pub_quaternion": False},
            {"pub_mag": True},
            {"pub_angular_velocity": False},
            {"pub_acceleration": False},
            {"pub_free_acceleration": False},
            {"pub_dq": False},
            {"pub_dv": False},
            {"pub_sampletime": True},
            {"pub_temperature": False},
            {"pub_pressure": False},
            {"pub_gnss": False},
            {"pub_twist": False},
            {"pub_transform": False},
            {"pub_positionLLA": False},
            {"pub_velocity": False}
        ],
        arguments=[]
    )
    ld.add_action(imu_node)

    return ld
