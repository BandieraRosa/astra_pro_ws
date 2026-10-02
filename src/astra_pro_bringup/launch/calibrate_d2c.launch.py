from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    args = [
        DeclareLaunchArgument('pairs_dir', default_value='/tmp/d2c_pairs'),
        DeclareLaunchArgument('auto_interval', default_value='0.0'),
        DeclareLaunchArgument('ir_raw_topic', default_value='/camera/ir/image_raw'),
        DeclareLaunchArgument('ir_topic', default_value='/camera/ir/image_mono8'),
        DeclareLaunchArgument('color_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('mode', default_value='normalize'),
        DeclareLaunchArgument('cols', default_value='10'),
        DeclareLaunchArgument('rows', default_value='7'),
        DeclareLaunchArgument('square', default_value='0.02'),
    ]

    # mono16 -> mono8 for the calibrator (normalize: brighter, better contrast)
    converter = ComposableNode(
        package='ir_converter',
        plugin='ir_converter::IrY10Converter',
        name='ir_converter',
        parameters=[{
            'input_topic': LaunchConfiguration('ir_raw_topic'),
            'output_topic': LaunchConfiguration('ir_topic'),
            'mode': LaunchConfiguration('mode'),
        }],
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
    )

    container = ComposableNodeContainer(
        name='calib_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[converter, capture],
        output='screen',
    )

    return LaunchDescription(args + [container])
