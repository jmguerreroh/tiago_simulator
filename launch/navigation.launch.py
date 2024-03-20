# Copyright (c) 2024 José Miguel Guerrero Hernández
#
# Licensed under the Attribution-ShareAlike 4.0 International (CC BY-SA 4.0) License;
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://creativecommons.org/licenses/by-sa/4.0/
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    sim_dir = get_package_share_directory('tiago_simulator')
    tiago_2dnav_dir = get_package_share_directory('tiago_2dnav')
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    config = os.path.join(sim_dir, 'config', 'params.yaml')

    with open(config, "r") as stream:
        try:
            conf = (yaml.safe_load(stream))

        except yaml.YAMLError as exc:
            print(exc)

    world_name = conf['tiago_simulator']['world']

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "params_file": os.path.join(
                tiago_2dnav_dir, "params", "tiago_nav_public_sim.yaml"
            ),
            "map": os.path.join(sim_dir, 'maps', world_name + '.yaml'),
            "use_sim_time": "True",
        }.items(),
    )

    rviz_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "rviz_launch.py")
        ),
        launch_arguments={
            "rviz_config": os.path.join(
                tiago_2dnav_dir, "config", "rviz", "navigation.rviz"
            ),
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(nav2_bringup_launch)
    ld.add_action(rviz_bringup_launch)

    return ld
