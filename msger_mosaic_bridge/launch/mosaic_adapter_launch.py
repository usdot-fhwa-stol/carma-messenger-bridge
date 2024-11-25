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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments for all parameters
    role_id_arg = DeclareLaunchArgument(
        'role_id',
        default_value='msger_1',
        description='Role ID for the messenger'
    )
    messenger_ip_address_arg = DeclareLaunchArgument(
        'messenger_ip_address',
        default_value='127.0.0.1',
        description='IP address of the CARMA Messenger'
    )
    cdasim_ip_address_arg = DeclareLaunchArgument(
        'cdasim_ip_address',
        default_value='127.0.0.1',
        description='IP address of the CDASim'
    )
    host_ip_arg = DeclareLaunchArgument(
        'host_ip',
        default_value='127.0.0.1',
        description='Host IP address'
    )
    enable_registration_arg = DeclareLaunchArgument(
        'enable_registration',
        default_value='true',
        description='Enable registration feature'
    )
    enable_vehicle_status_arg = DeclareLaunchArgument(
        'enable_vehicle_status',
        default_value='true',
        description='Enable vehicle status feature'
    )
    registration_port_remote_arg = DeclareLaunchArgument(
        'registration_port_remote',
        default_value='3100',
        description='Remote port for registration'
    )
    siren_and_light_status_port_remote_arg = DeclareLaunchArgument(
        'siren_and_light_status_port_remote',
        default_value='3101',
        description='Remote port for siren and light status'
    )
    registration_port_local_arg = DeclareLaunchArgument(
        'registration_port_local',
        default_value='3050',
        description='Local port for registration'
    )
    vehicle_status_port_local_arg = DeclareLaunchArgument(
        'vehicle_status_port_local',
        default_value='3051',
        description='Local port for vehicle status'
    )
    traffic_event_port_local_arg = DeclareLaunchArgument(
        'traffic_event_port_local',
        default_value='3052',
        description='Local port for traffic events'
    )

    # Node definition
    mosaic_adapter_node = Node(
        package='msger_mosaic_bridge',
        executable='mosaic_adapter_node',
        name='mosaic_adapter',
        output='screen',
        parameters=[
            {'role_id': LaunchConfiguration('role_id')},
            {'messenger_ip_address': LaunchConfiguration('messenger_ip_address')},
            {'cdasim_ip_address': LaunchConfiguration('cdasim_ip_address')},
            {'host_ip': LaunchConfiguration('host_ip')},
            {'enable_registration': LaunchConfiguration('enable_registration')},
            {'enable_vehicle_status': LaunchConfiguration('enable_vehicle_status')},
            {'registration_port_remote': LaunchConfiguration('registration_port_remote')},
            {'siren_and_light_status_port_remote': LaunchConfiguration('siren_and_light_status_port_remote')},
            {'registration_port_local': LaunchConfiguration('registration_port_local')},
            {'vehicle_status_port_local': LaunchConfiguration('vehicle_status_port_local')},
            {'traffic_event_port_local': LaunchConfiguration('traffic_event_port_local')}
        ],
        remappings=[
            ('vehicle_pose', '/hardware_interface/gps_common_fix'),
            ('velocity', '/hardware_interface/velocity')
        ],
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    return LaunchDescription([
        # Add all the declared arguments
        role_id_arg,
        messenger_ip_address_arg,
        cdasim_ip_address_arg,
        host_ip_arg,
        enable_registration_arg,
        enable_vehicle_status_arg,
        registration_port_remote_arg,
        siren_and_light_status_port_remote_arg,
        registration_port_local_arg,
        vehicle_status_port_local_arg,
        traffic_event_port_local_arg,
        # Add the node
        mosaic_adapter_node
    ])
