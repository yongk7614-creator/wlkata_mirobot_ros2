from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    Real hardware launch for WLKATA Mirobot.

    This launch intentionally avoids the default demo/spawn_controllers flow and
    instead runs:
      - robot_state_publisher
      - optional joint_state_publisher (fallback current-state source)
      - move_group
      - optional RViz
      - mirobot_gcode_driver (FollowJointTrajectory action server)

    Use this launch for real hardware with the custom G-code driver.
    """

    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    serial_timeout_sec = LaunchConfiguration("serial_timeout_sec")
    auto_home = LaunchConfiguration("auto_home")
    startup_delay_sec = LaunchConfiguration("startup_delay_sec")
    settle_delay_sec = LaunchConfiguration("settle_delay_sec")
    wait_for_reply = LaunchConfiguration("wait_for_reply")
    use_rviz = LaunchConfiguration("use_rviz")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    driver_start_delay_sec = LaunchConfiguration("driver_start_delay_sec")

    moveit_config = (
        MoveItConfigsBuilder(
            "wlkata_mirobot_description",
            package_name="wlkata_mirobot_moveit_config",
        )
        .to_moveit_configs()
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": False},
        ],
    )

    # Fallback joint_states source. This is useful when no real joint-state
    # feedback publisher exists yet. Disable it once hardware feedback is added.
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": False},
        ],
        condition=IfCondition(use_joint_state_publisher),
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            str(moveit_config.package_path / "config" / "moveit.rviz"),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": False},
        ],
        condition=IfCondition(use_rviz),
    )

    mirobot_driver_node = Node(
        package="wlkata_arm_move",
        executable="mirobot_moveit_move",
        name="mirobot_gcode_driver",
        output="screen",
        parameters=[
            {
                "action_name": "mirobot_group_controller/follow_joint_trajectory",
                "serial_port": serial_port,
                "baud_rate": ParameterValue(baud_rate, value_type=int),
                "serial_timeout_sec": ParameterValue(
                    serial_timeout_sec, value_type=float
                ),
                "auto_home": ParameterValue(auto_home, value_type=bool),
                "startup_delay_sec": ParameterValue(
                    startup_delay_sec, value_type=float
                ),
                "settle_delay_sec": ParameterValue(
                    settle_delay_sec, value_type=float
                ),
                "wait_for_reply": ParameterValue(wait_for_reply, value_type=bool),
                "joint_names": [
                    "joint1",
                    "joint2",
                    "joint3",
                    "joint4",
                    "joint5",
                    "joint6",
                ],
                "command_prefix": "M21 G90 G00",
                "line_ending": "\r\n",
            }
        ],
    )

    delayed_driver = TimerAction(
        period=driver_start_delay_sec,
        actions=[mirobot_driver_node],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
                description="Serial device for WLKATA Mirobot",
            ),
            DeclareLaunchArgument(
                "baud_rate",
                default_value="115200",
                description="Serial baud rate",
            ),
            DeclareLaunchArgument(
                "serial_timeout_sec",
                default_value="1.0",
                description="Serial read timeout in seconds",
            ),
            DeclareLaunchArgument(
                "auto_home",
                default_value="false",
                description="Whether to send homing command on startup",
            ),
            DeclareLaunchArgument(
                "startup_delay_sec",
                default_value="2.0",
                description="Driver boot stabilization delay",
            ),
            DeclareLaunchArgument(
                "settle_delay_sec",
                default_value="0.0",
                description="Fallback per-waypoint delay when trajectory has no timing",
            ),
            DeclareLaunchArgument(
                "wait_for_reply",
                default_value="true",
                description="Wait for a serial reply after each G-code command",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz with MoveIt configuration",
            ),
            DeclareLaunchArgument(
                "use_joint_state_publisher",
                default_value="true",
                description=(
                    "Publish fallback /joint_states until a real hardware "
                    "joint-state publisher is available"
                ),
            ),
            DeclareLaunchArgument(
                "driver_start_delay_sec",
                default_value="3.0",
                description="Delay before starting the G-code driver node",
            ),
            rsp_node,
            joint_state_publisher_node,
            move_group_node,
            rviz_node,
            delayed_driver,
        ]
    )
