# Copyright (c) 2021 PAL Robotics S.L.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Modified by José Miguel Guerrero Hernández

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_pal.include_utils import include_launch_py_description
from launch_ros.actions import SetRemap
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    sim_dir = get_package_share_directory('tiago_simulator')
    config = os.path.join(sim_dir, 'config', 'params.yaml')
    sim_time_arg = DeclareLaunchArgument(
      'use_sim_time', default_value='True',
      description='Yaml file with the info of the motions. ')

    with open(config, "r") as stream:
        try:
            conf = (yaml.safe_load(stream))

        except yaml.YAMLError as exc:
            print(exc)

    world_name = conf['tiago_simulator']['world']
    nav2 = include_launch_py_description(
        'tiago_2dnav', ['launch', 'tiago_nav_bringup.launch.py'],
        launch_arguments={
          'map': os.path.join(sim_dir, 'maps', world_name + '.yaml'),
        }.items()
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Remappings
    scan_remap = SetRemap(src='scan', dst='scan_raw')
    ld.add_action(scan_remap)

    ld.add_action(sim_time_arg)
    ld.add_action(nav2)

    return ld
