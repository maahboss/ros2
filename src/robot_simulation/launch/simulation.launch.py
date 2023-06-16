import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable


def generate_launch_description():
    world_file_name = 'house.world'
    world = os.path.join(get_package_share_directory(
        'robot_simulation'), 'worlds', world_file_name)
    sdf_dir = os.path.join(get_package_share_directory('robot_description'), 'urdf')
    sdf_file = os.path.join(sdf_dir, 'robot.urdf')
    default_rviz_config_path = os.path.join(get_package_share_directory('robot_simulation'), 'rviz/urdf_config.rviz')
    declare_world_fname = DeclareLaunchArgument(
        'world_fname', default_value=world, description='absolute path of gazebo world file')
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true', description='Set to "false" to run headless.')

    world_fname = LaunchConfiguration('world_fname')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')

    pkg_megarover_samples_ros2 = get_package_share_directory(
        'robot_simulation')
    launch_file_dir = os.path.join(pkg_megarover_samples_ros2, 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_fname,
            'gui': gui
        }.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', 'robot',
            '-x', '0',
            '-y', '0',
            '-z', '1',
            '-file', sdf_file,
        ]
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_file_dir, '/robot_state_publisher.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )



    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('robot_simulation'), 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )


    return LaunchDescription([
        declare_world_fname,
        declare_gui,
        gazebo,
        spawn_entity,
        robot_state_publisher,
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        rviz_node,
        robot_localization_node
    ])
