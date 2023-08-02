#################################################################################
#   Copyright Lars Ludvigsen. All Rights Reserved.                              #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_pkg',
            namespace='imu',
            executable='imu_node',
            name='imu_node',
            parameters=[{'com_id': '/dev/ttyUSB2'}]
        ),
        Node(
            package='imu_integrator',
            namespace='imu',
			executable='imu_integrator_node',
			name='imu_integrator_node',
        )
    ])