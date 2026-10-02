from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    converter = Node(
        package='ir_converter',
        executable='ir_converter',
        name='ir_converter',
        parameters=[{
            'input_topic': LaunchConfiguration('ir_raw_topic'),
            'output_topic': LaunchConfiguration('ir_topic'),
            'mode': LaunchConfiguration('mode'),
        }],
        output='screen',
    )

    capture = Node(
        package='stereo_d2c_calib',
        executable='d2c_capture',
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
        output='screen',
    )

    return LaunchDescription(args + [converter, capture])
