from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='msger_mosaic_bridge',
            executable='mosaic_adapter_node',
            name='mosaic_adapter',
            output='screen',
            parameters=[
                {'role_id': 'msg_veh_1'},
                {'ip_address': '172.2.0.2'},
                {'host_ip': '172.2.0.3'},
                {'enable_registration': True},
                {'enable_vehicle_status': True},
                {'registration_port_remote': 6001},
                {'registration_port_local': 4003},
                {'vehicle_status_port_remote': 7001},
                {'vehicle_status_port_local': 4004}
            ],
            arguments=['--ros-args', '--log-level', 'DEBUG']  # Setting log level to DEBUG
        )
    ])
