from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import  AnyLaunchDescriptionSource

tag_name_arg = DeclareLaunchArgument("tag_name", default_value="Apriltag36_11_00000", description="Name of april tag to spawn")
tag_name = LaunchConfiguration("tag_name")

x = '-1.0'
y = '0.0'
z = '0.5'
r = '0'
p = '1.57'
t = '0'


april_gazebo_package_dir = get_package_share_directory('apriltag_gazebo')
sdf_file= PathJoinSubstitution([april_gazebo_package_dir, 'models', tag_name, 'model.sdf'])

april_ros_package_dir = get_package_share_directory('apriltag_ros')
param_file = PathJoinSubstitution([april_ros_package_dir, 'cfg', 'tags_3611.yaml'])

def generate_launch_description():

    camera_bringup_L = IncludeLaunchDescription(AnyLaunchDescriptionSource([get_package_share_directory("oakd_lite_description"),
                                                                            '/launch', '/oakd_bringup.launch.xml']))

    spawn_april_N = Node(
        package='ros_ign_gazebo',
        executable='create',
        name=tag_name,
        output = 'screen',
        arguments=['-file', sdf_file,
                    '-x', x,
                    '-y', y,
                    '-z', z,
                     '-R', r,
                     '-P', p,
                     '-Y', y],
    )

    apriltag_transform_N = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name = "apriltag_transform_node",
        output = 'screen',
        remappings=[
            ('image_rect', '/oakd_lite/image'),
            ('camera_info', 'oakd_lite/camera_info'),
        ],
        arguments=['--params-file', param_file],

    )

    Args = [tag_name_arg]
    Nodes = [spawn_april_N, apriltag_transform_N]
    Launch = [camera_bringup_L]
    return LaunchDescription(Args + Nodes + Launch)
