import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package="ldlidar",
            executable="ldlidar",
            name="ldlidar",
            parameters=[
                {"serial_port_candidates": ["/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"]}
            ]
        ),
    ])
