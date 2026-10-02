from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('astra_pro_bringup')

    args = [
        DeclareLaunchArgument('pairs_dir', default_value='/tmp/d2c_pairs'),
        DeclareLaunchArgument('auto_interval', default_value='2.0'),
        DeclareLaunchArgument('ir_raw_topic', default_value='/camera/ir/image_raw'),
        DeclareLaunchArgument('ir_topic', default_value='/camera/ir/image_mono8'),
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

    # Camera stack (orbbec + v4l2 + flip relay + static TF).
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, 'launch', 'astra_pro.launch.py'])
        ),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'ir_info_url': LaunchConfiguration('ir_info_url'),
            'color_info_url': LaunchConfiguration('color_info_url'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('with_cameras')),
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

    return LaunchDescription(args + [cameras, container])
