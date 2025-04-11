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
        default_value='false',
        description='Whether to launch gzclient GUI'
    )

    # For environment models
    # We append the 'gazebo_tf/models' directory into GAZEBO_MODEL_PATH
    set_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(get_package_share_directory('gazebo_tf'), 'models')
    )

    # ------------------- 2) Load world, spawn Gazebo server/client -------------------
    world_path = LaunchConfiguration('world_path')
    gui = LaunchConfiguration('gui')

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

    # ------------------- 3) RViZ Node (optional) -------------------
    # Using a sample config from 'gazebo_tf/rviz/audi.rviz'
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='a3_audi_rviz',
        output={'both': 'log'},
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('gazebo_tf'),
                'rviz',
                'audi.rviz'
            )
        ]
    )

    # ------------------- 4) Generate robot_description from Xacro -------------------
    # We'll pass this to the child audibot_robot.launch.py
    audibot_xacro = os.path.join(
        get_package_share_directory('audibot_description'),
        'urdf',
        'audibot.urdf.xacro'
    )
    # This Command(...) runs xacro on the .urdf.xacro
    # We'll store it in a LaunchConfiguration called 'robot_description'
    robot_description_cmd = Command([
        FindExecutable(name='xacro'),
        ' ',
        audibot_xacro,
        ' ',
        'pub_tf:=true',  # optional arguments
        ' ',
        'blue:=false',   # pass "false" => "orange" car
    ])

    declare_robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        default_value=robot_description_cmd,
        description='Combined URDF for Audibot (orange version)'
    )

    # ------------------- 5) Spawn the "orange" Audibot with above URDF -------------------
    orange_audibot_options = {
        'robot_name': 'orange',
        'start_x': '24.2',
        'start_y': '13.2',
        'start_z': '0',
        'start_yaw': '0',
        'pub_tf': 'true',
        'tf_freq': '100.0',
        'blue': 'false',
        # crucially: pass the same LaunchConfiguration('robot_description')
        'robot_description': LaunchConfiguration('robot_description')
    }

    # We'll nest the existing audibot_gazebo => audibot_robot.launch.py
    spawn_orange_audibot = GroupAction([
        PushRosNamespace('orange'),  # put in "orange/" namespace
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

    # ------------------- 6) A small "callback" so we only spawn after gzserver is up -------------------
    spawn_orange_robot_callback = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gzserver,
            on_start=[
                LogInfo(msg='gzserver has started!'),
                TimerAction(
                    period=5.0,  # wait 5s
                    actions=[spawn_orange_audibot],
                )
            ]
        )
    )

    # ------------------- 7) Extra "gazebo_connect" Node (some custom bridging) -------------------
    gazebo_connect_node = Node(
        package='gazebo_tf',
        executable='gazebo_connect',
        name='gazebo_connect',
        parameters=[{'use_sim_time': False}]
    )

    # ------------------- 8) Build final LaunchDescription -------------------
    ld = LaunchDescription()

    # Add all declared arguments
    ld.add_action(world_path_arg)
    ld.add_action(gui_arg)
    ld.add_action(declare_robot_description_arg)

    # Set environment for models
    ld.add_action(set_model_path)

    # Start Gazebo
    ld.add_action(gzserver)
    ld.add_action(gzclient)

    # Start bridging + spawn robot
    ld.add_action(gazebo_connect_node)
    ld.add_action(spawn_orange_robot_callback)

    # Launch RViz
    ld.add_action(rviz_node)

    return ld
