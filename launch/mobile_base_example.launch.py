import os
from ament_index_python.packages import (
    get_package_share_directory,
    get_package_prefix,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # External args
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    x_arg = DeclareLaunchArgument("x", default_value="0")    
    y_arg = DeclareLaunchArgument("y", default_value="0")
    z_arg = DeclareLaunchArgument("z", default_value="0")
    R_arg = DeclareLaunchArgument("R", default_value="0")
    P_arg = DeclareLaunchArgument("P", default_value="0")
    Y_arg = DeclareLaunchArgument("Y", default_value="0")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")

    # Setup project paths
    pkg_project = get_package_share_directory('gz_virtual_base_system')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Opaque function to use the launch argument inside here
    def create_robot_description(context):
        robot_xacro = os.path.join(pkg_project, "urdf/", "base_example.urdf.xacro")
        assert os.path.exists(robot_xacro), "The base_example.urdf.xacro doesnt exist in " + str(robot_xacro)
        robot_description_config = xacro.process_file(
            robot_xacro
        )
        robot_desc = robot_description_config.toxml()

        return [SetLaunchConfiguration("robot_desc", robot_desc)]

    create_robot_description_arg = OpaqueFunction(
        function=create_robot_description
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_project, "/config/rviz", "mobile_base.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[
            {"use_sim_time": True},
        ],
    )

    ############ Simulation stuff
    # Simulator launch file
    world_file = pkg_project + '/world/mobile_base.world'
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={
            'gz_args': " -r " + world_file
        }.items()
    )


    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='mobile_base_spawner',
        arguments=[
            "-name", "mobile_base",
            "-entity", "model",
            "-x", LaunchConfiguration('x'),
            "-y", LaunchConfiguration('y'),
            "-z", LaunchConfiguration('z'),
            "-R", LaunchConfiguration('R'),
            "-P", LaunchConfiguration('P'),
            "-Y", LaunchConfiguration('Y'),
            "-topic", "robot_description",
        ],
        output="screen",
    )

    ### Gazebo to ROS stuff

    ros_gz_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"robot_description": LaunchConfiguration("robot_desc")},
        ],
    )

    spawn_active_controllers = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_controller_manager",
        output="screen",
        arguments=[
            "--controller-manager", "controller_manager", #just the name of the controller_manager
            "joint_state_broadcaster", "virtual_base_cmd_controller"
        ],
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    # spawn_deactive_controllers = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     name="spawn_controller_manager",
    #     output="screen",
    #     arguments=[
    #         "--controller-manager", "controller_manager", #just the name of the controller_manager
    #         "--inactive", "base_velocity_controller",
    #     ],
    #     parameters=[
    #         {"use_sim_time": LaunchConfiguration("use_sim_time")},
    #     ],
    # )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        x_arg,
        y_arg,
        z_arg,
        R_arg,
        P_arg,
        Y_arg,
        create_robot_description_arg,
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        spawn_active_controllers,
        #spawn_deactive_controllers,
        ros_gz_clock,
        rviz,
    ])