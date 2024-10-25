#   Copyright (C) 2024 LEIDOS.
#   Licensed under the Apache License, Version 2.0 (the "License"); you may not
#   use this file except in compliance with the License. You may obtain a copy of
#   the License at
#   http://www.apache.org/licenses/LICENSE-2.0
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#   WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#   License for the specific language governing permissions and limitations under
#   the License.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='msger_mosaic_bridge',  # Ensure this is the correct package name
            executable='mosaic_adapter_node',  # Ensure this is the correct executable name
            name='mosaic_adapter',
            output='screen',
            parameters=[
                {'role_id': 'msg_veh_1'},
                {'messenger_ip_address': '127.0.0.1'},
                {'cdasim_ip_address': '127.0.0.1'},
                {'host_ip': '127.0.0.1'},
                {'enable_registration': True},
                {'enable_vehicle_status': True},
                {'registration_port_remote': 6001},
                {'registration_port_local': 4001},
                {'vehicle_status_port_remote': 7001},
                {'vehicle_status_port_local': 4002},
                {'siren_and_light_status_port_remote': 8001},
                {'siren_and_light_status_port_local': 4003}
                
            ],
            remappings=[
                ('vehicle_pose', 'hardware_interface/gps_common_fix'),  # Remap the GPS topic
                ('velocity', 'hardware_interface/velocity')  # Remap the velocity topic
            ],
            arguments=['--ros-args', '--log-level', 'INFO']  # Setting log level to DEBUG
        )
    ])