import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: F401
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('astra_pro_bringup')

    args = [
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

    # All node parameters live in config/astra_pro_params.yaml
    # (orbbec_camera + v4l2_camera share one file, each reads its own keys).
    # Launch args below win over the yaml on conflict.
    orbbec_parameters = [
        PathJoinSubstitution([bringup_share, 'config', 'astra_pro_params.yaml']),
        {
            'camera_name': LaunchConfiguration('camera_name'),
            'ir_info_url': LaunchConfiguration('ir_info_url'),
        },
    ]

    # Node structure mirrors orbbec_camera/launch/astra.launch.py.
    ros_distro = os.environ.get('ROS_DISTRO', '')
    if ros_distro == 'foxy':
        orbbec_node = Node(
            package='orbbec_camera',
            executable='orbbec_camera_node',
            name='orbbec',
            namespace=LaunchConfiguration('camera_name'),
            parameters=orbbec_parameters,
            output='screen',
        )
        return LaunchDescription(
            args
            + [
                orbbec_node,
                Node(
                    package='v4l2_camera',
                    executable='v4l2_camera_node',
                    name='v4l2_camera_node',
                    namespace=[LaunchConfiguration('camera_name'), '/color'],
                    parameters=[
                        PathJoinSubstitution(
                            [bringup_share, 'config', 'astra_pro_params.yaml']
                        ),
                        {'camera_info_url': LaunchConfiguration('color_info_url')},
                    ],
                    output='screen',
                ),
            ]
        )

    compose_node = ComposableNode(
        package='orbbec_camera',
        plugin='orbbec_camera::OBCameraNodeDriver',
        name='orbbec',
        namespace='',
        parameters=orbbec_parameters,
    )
    # Same process as orbbec: v4l2 as a composable node in the same container.
    # namespace 'color' joins the container's pushed namespace -> /camera/color.
    v4l2_compose_node = ComposableNode(
        package='v4l2_camera',
        plugin='v4l2_camera::V4L2Camera',
        name='v4l2_camera_node',
        namespace='color',
        parameters=[
            PathJoinSubstitution([bringup_share, 'config', 'astra_pro_params.yaml']),
            {'camera_info_url': LaunchConfiguration('color_info_url')},
        ],
    )
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[compose_node, v4l2_compose_node],
        output='screen',
    )

    # camera_color_optical_frame 挂进 TF 树 (父系 camera_depth_optical_frame, 初值重合;
    # 有外参标定后再填 xyz/rpy). 与 yaml 中 camera_frame_id 对应.
    color_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='color_optical_static_tf',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'camera_depth_optical_frame', 'camera_color_optical_frame',
        ],
        output='screen',
    )

    return LaunchDescription(
        args
        + [
            GroupAction(
                [PushRosNamespace(LaunchConfiguration('camera_name')), container]
            ),
            color_optical_tf,
        ]
    )
