# Copyright 2026 BandieraRossa
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('astra_pro_bringup')

    args = [
        DeclareLaunchArgument('pairs_dir', default_value='/tmp/d2c_pairs'),
        DeclareLaunchArgument('auto_interval', default_value='2.0'),
        DeclareLaunchArgument('ir_raw_topic', default_value='/camera/ir/image_raw'),
        DeclareLaunchArgument('ir_topic', default_value='/camera/ir/image_mono8'),
        # Capture the raw mirrored stream; d2c_solve --flip-color applies the
        # matching image-coordinate and intrinsic-camera correction.
        DeclareLaunchArgument('color_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('mode', default_value='normalize'),
        DeclareLaunchArgument('cols', default_value='10'),
        DeclareLaunchArgument('rows', default_value='7'),
        DeclareLaunchArgument('square', default_value='0.02'),
        # false when cameras are already running elsewhere
        DeclareLaunchArgument('with_cameras', default_value='true'),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument(
            'ir_info_url',
            default_value=[
                'file://',
                PathJoinSubstitution([bringup_share, 'config', 'astra_pro_ir.yaml']),
            ],
        ),
        DeclareLaunchArgument(
            'color_info_url',
            default_value=[
                'file://',
                PathJoinSubstitution([bringup_share, 'config', 'astra_pro_color.yaml']),
            ],
        ),
    ]

    camera_params = [
        PathJoinSubstitution([bringup_share, 'config', 'astra_pro_params.yaml']),
        {
            'camera_name': LaunchConfiguration('camera_name'),
            'ir_info_url': LaunchConfiguration('ir_info_url'),
        },
    ]
    orbbec = ComposableNode(
        package='orbbec_camera', plugin='orbbec_camera::OBCameraNodeDriver',
        name='orbbec', namespace='', parameters=camera_params,
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    v4l2 = ComposableNode(
        package='v4l2_camera', plugin='v4l2_camera::V4L2Camera',
        name='v4l2_camera_node', namespace='color',
        parameters=[
            PathJoinSubstitution([bringup_share, 'config', 'astra_pro_params.yaml']),
            {'camera_info_url': LaunchConfiguration('color_info_url')},
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    flip = ComposableNode(
        package='image_flip', plugin='ImageFlip', name='flip_node', namespace='color',
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # mono16 -> mono8 for the calibrator (normalize: brighter, better contrast)
    # intra-process: same-container edge ir_converter -> d2c_capture avoids
    # DDS serialization (both sides already SensorDataQoS-compatible).
    converter = ComposableNode(
        package='ir_converter',
        plugin='ir_converter::IrY10Converter',
        name='ir_converter',
        parameters=[{
            'input_topic': LaunchConfiguration('ir_raw_topic'),
            'output_topic': LaunchConfiguration('ir_topic'),
            'mode': LaunchConfiguration('mode'),
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    capture = ComposableNode(
        package='stereo_d2c_calib',
        plugin='stereo_d2c_calib::D2CCapture',
        name='d2c_capture',
        parameters=[{
            'out_dir': LaunchConfiguration('pairs_dir'),
            'ir_topic': LaunchConfiguration('ir_topic'),
            'color_topic': LaunchConfiguration('color_topic'),
            'auto_interval': LaunchConfiguration('auto_interval'),
            'cols': LaunchConfiguration('cols'),
            'rows': LaunchConfiguration('rows'),
            'square': LaunchConfiguration('square'),
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name='calib_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[converter, capture],
        output='screen',
    )
    camera_nodes = LoadComposableNodes(
        target_container=container,
        composable_node_descriptions=[orbbec, v4l2, flip],
        condition=IfCondition(LaunchConfiguration('with_cameras')),
    )

    color_optical_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='color_optical_static_tf',
        arguments=['0', '0', '0', '0', '0', '0',
                   'camera_depth_optical_frame', 'camera_color_optical_frame'],
        output='screen',
    )
    return LaunchDescription([
        *args,
        GroupAction([
            PushRosNamespace(LaunchConfiguration('camera_name')),
            container,
            camera_nodes,
        ]),
        color_optical_tf,
    ])
