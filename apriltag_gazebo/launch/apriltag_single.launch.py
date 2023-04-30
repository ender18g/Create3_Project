from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess

tag_name_arg = DeclareLaunchArgument("tag_name", default_value="Apriltag36_11_00000", description="Name of april tag to spawn")
tag_name = LaunchConfiguration("tag_name")

z_offset = '0.05'

package_dir = get_package_share_directory('apriltag_gazebo')

sdf_file= PathJoinSubstitution([package_dir, 'models', tag_name, 'model.sdf'])

def generate_launch_description():

    ignition_launch  = ExecuteProcess(
        cmd = ['ign', 'gazebo', '-v', '4', 'empty.sdf'],
        output='screen',
    )

    spawn_node = Node(
        package='ros_ign_gazebo',
        executable='create',
        name=tag_name,
        output = 'screen',
        arguments=['-file', sdf_file, '-z', z_offset],

    )

    return LaunchDescription([tag_name_arg, 
                            ignition_launch,
                            spawn_node,
                            ])

