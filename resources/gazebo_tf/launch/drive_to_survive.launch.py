import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    LogInfo,
    TimerAction,
    GroupAction,
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ------------------- 1) Declare Launch Arguments -------------------
    world_path_arg = DeclareLaunchArgument(
        'world_path',
        default_value=PathJoinSubstitution([
            FindPackageShare("gazebo_tf"), "worlds", "race_track.world"
        ]),
        description='Path to the world SDF, default is race_track.world'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false', # set to true to launch gzclient GUI Gazebo
        description='Whether to launch gzclient GUI'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true', # set to true to launch RViz
        description='Whether to launch RViz'
    )

    set_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(get_package_share_directory('gazebo_tf'), 'models')
    )

    # Load arguments
    world_path = LaunchConfiguration('world_path')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')

    # ------------------- 2) Start Gazebo (server + client) -------------------
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen',
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gui),  # only if gui:=true
    )

    # ------------------- 3) Conditionally Launch RViz -------------------
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='a3_audi_rviz',
        output='screen',
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('gazebo_tf'),
                'rviz',
                'audi.rviz'
            )
        ]
    )

    # ------------------- 4) Xacro -> robot_description -------------------
    audibot_xacro = os.path.join(
        get_package_share_directory('audibot_description'),
        'urdf',
        'audibot.urdf.xacro'
    )
    robot_description_cmd = Command([
        FindExecutable(name='xacro'),
        ' ',
        audibot_xacro,
        ' ',
        'pub_tf:=true',
        ' ',
        'blue:=false',   # orange car
    ])

    declare_robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        default_value=robot_description_cmd,
        description='Combined URDF for the orange Audibot'
    )

    # ------------------- 5) Spawn the orange Audibot -------------------
    orange_audibot_options = {
        'robot_name': 'orange',
        'start_x': '24.2',
        'start_y': '13.2',
        'start_z': '0',
        'start_yaw': '0',
        'pub_tf': 'true',
        'tf_freq': '100.0',
        'blue': 'false',
        'robot_description': LaunchConfiguration('robot_description')
    }

    spawn_orange_audibot = GroupAction([
        PushRosNamespace('orange'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('audibot_gazebo'),
                    'launch',
                    'audibot_robot.launch.py'
                )
            ]),
            launch_arguments=orange_audibot_options.items()
        )
    ])

    # Wait 5s after gzserver starts, then spawn
    spawn_orange_robot_callback = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gzserver,
            on_start=[
                LogInfo(msg='gzserver has started!'),
                TimerAction(
                    period=5.0,
                    actions=[spawn_orange_audibot],
                )
            ]
        )
    )

    # ------------------- 6) "gazebo_connect" bridging node -------------------
    gazebo_connect_node = Node(
        package='gazebo_tf',
        executable='gazebo_connect',
        name='gazebo_connect',
        parameters=[{'use_sim_time': False}]
    )

    # ------------------- 7) Build LaunchDescription -------------------
    ld = LaunchDescription()

    # Add declared arguments
    ld.add_action(world_path_arg)
    ld.add_action(gui_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(declare_robot_description_arg)

    # Env for models
    ld.add_action(set_model_path)

    # Start Gazebo stuff
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(gazebo_connect_node)
    ld.add_action(spawn_orange_robot_callback)

    # Conditionally run RViz
    ld.add_action(rviz_node)

    return ld
