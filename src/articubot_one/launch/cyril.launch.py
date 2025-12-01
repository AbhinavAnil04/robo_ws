from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    articubot_pkg = FindPackageShare('articubot_one')

    return LaunchDescription([
        # Launch robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([articubot_pkg, 'launch', 'launch_robot.launch.py'])
            ),
            launch_arguments={'use_sim_time': 'false'}.items()
        ),


        # Launch online async navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([articubot_pkg, 'launch', 'online_async_launch.py'])
            ),
            launch_arguments={'use_sim_time': 'false'}.items()
        ),

        # Run twist_mux with parameters
        ExecuteProcess(
            cmd=['ros2', 'run', 'twist_mux', 'twist_mux', '--ros-args', '--params-file',
                 './src/articubot_one/config/twist_mux.yaml', '-r', 'cmd_vel_out:=diff_cont/cmd_vel_unstamped'],
            output='screen'
        ),

        # Launch navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([articubot_pkg, 'launch', 'navigation_launch.py'])
            ),
            launch_arguments={'use_sim_time': 'false'}.items()
        ),
    ])
