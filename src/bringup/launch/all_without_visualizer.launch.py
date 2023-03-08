import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package="ldlidar",
            executable="ldlidar",
            name="lidar0",
            parameters=[
                {"serial_port_candidates": [
                    # Dev PC
                    # "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",

                    # Prod PC
                    "/dev/serial/by-path/pci-0000:00:15.0-usb-0:2.2:1.0-port0",
                ]}
            ],
            remappings=[
                ("scan", "scan0"),
            ],
        ),
        Node(
            package="ldlidar",
            executable="ldlidar",
            name="lidar1",
            parameters=[
                {"serial_port_candidates": [
                    # Dev PC
                    # "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",

                    # Prod PC
                    "/dev/serial/by-path/pci-0000:00:15.0-usb-0:2.1:1.0-port0",
                ]}
            ],
            remappings=[
                ("scan", "scan1"),
            ],
        ),
        Node(
            package="robot_interface_proxy",
            executable="can_proxy",
            name="can_proxy",
            parameters=[
            ]
        )
    ])
