#
# Copyright 2026 shiryu nakano
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
#

"""Launch OAK-D on-device YOLO inference."""

from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create the on-device inference launch description."""
    package_share = Path(get_package_share_directory("arcanain_depthai_ros2"))
    default_parameters = package_share / "config" / "inference.yaml"
    default_yolo_config = package_share / "config" / "yolov4-tiny.json"
    default_model = os.path.expanduser(
        "~/.cache/blobconverter/"
        "yolov4_tiny_coco_416x416_openvino_2022.1_6shave.blob"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=default_model,
                description="Path to a DepthAI .blob model",
            ),
            DeclareLaunchArgument(
                "config_path",
                default_value=str(default_yolo_config),
                description="YOLO metadata JSON path",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_parameters),
                description="ROS parameter YAML path",
            ),
            DeclareLaunchArgument(
                "confidence_threshold",
                default_value="-1.0",
                description="-1 uses JSON value; otherwise a value from 0 to 1",
            ),
            Node(
                package="arcanain_depthai_ros2",
                executable="inference_node",
                name="yolo_detection_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "model": LaunchConfiguration("model"),
                        "config_path": LaunchConfiguration("config_path"),
                        "confidence_threshold": ParameterValue(
                            LaunchConfiguration("confidence_threshold"),
                            value_type=float,
                        ),
                    },
                ],
            ),
        ]
    )
