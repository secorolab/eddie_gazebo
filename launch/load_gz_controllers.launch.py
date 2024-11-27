from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node


# Function to select and load the appropriate controllers based on the given context and launch arguments
def select_controller(context, *args, **kwargs):
    """
    Selects the appropriate controllers based on the declared controller types.

    Args:
        context: The ROS2 launch configuration context.
        *args: Any additional arguments.
        **kwargs: Any additional keyword arguments.

    Returns:
        list: A list of controllers to be loaded.

    Raises:
        ValueError: If an unsupported controller type is specified.
    """
    # Retrieve the declared base controller and arm controller types from the ROS2 launch configuration
    declared_base_controller = LaunchConfiguration("base_controller").perform(context)
    declared_arm_controller = LaunchConfiguration("arm_controller").perform(context)

    use_kelo_tulip = LaunchConfiguration("use_kelo_tulip").perform(context)

    # Load and activate the arm effort controllers
    load_arm_left_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_left_effort_controller",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    load_arm_right_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_right_effort_controller",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    # Load and activate the arm joint trajectory controllers
    load_joint_trajectory_controller_left = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_left_joint_trajectory_controller",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )
    load_joint_trajectory_controller_right = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_right_joint_trajectory_controller",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    # Load and activate the base controllers
    load_joint_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "base_position_controller",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    load_joint_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "base_velocity_controller",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    load_joint_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "base_effort_controller",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    # Define a dictionary to map the declared base controller type to its corresponding controllers
    base_controller_map = {
        "position": load_joint_position_controller,
        "velocity": load_joint_velocity_controller,
        "effort": load_joint_effort_controller,
    }
    # Define a dictionary to map the declared arm controller type to its corresponding controllers
    arm_controller_map = {
        "effort": [load_arm_right_effort_controller, load_arm_left_effort_controller],
        "joint_trajectory": [
            load_joint_trajectory_controller_left,
            load_joint_trajectory_controller_right,
        ],
    }

    # Check if the declared base and arm controllers are supported
    if declared_base_controller not in base_controller_map:
        raise ValueError(f"Undefined base_controller: {declared_base_controller}")
    elif declared_arm_controller not in arm_controller_map:
        raise ValueError(f"Undefined arm_controller: {declared_arm_controller}")
    else:
        # Return the selected base and arm controllers for execution
        return (
            [*arm_controller_map[declared_arm_controller]]
            + [base_controller_map[declared_base_controller]]
            if use_kelo_tulip == "true"
            else []
        )


def generate_launch_description():
    """Generates the launch description for the Eddie robot controllers for the Gazebo simulation.

    This function sets up the environment variables, declares the base and arm controllers,
    loads the selected controllers or default controllers.

    Returns:
        LaunchDescription: The launch description for the Eddie robot simulation.
    """

    # Declare the base controller argument with a default value of "velocity"
    declare_base_controller_type = DeclareLaunchArgument(
        "base_controller",
        default_value="velocity",
        description="Specify controller for base (position, velocity, effort) (default: velocity)",
    )

    # Declare the arm controller argument with a default value of "joint_trajectory"
    declare_arm_controller_type = DeclareLaunchArgument(
        "arm_controller",
        default_value="joint_trajectory",
        description="Specify controller for arm (joint_trajectory, effort) (default: joint_trajectory)",
    )

    use_kelo_tulip_arg = DeclareLaunchArgument(
        "use_kelo_tulip",
        default_value="false",
        description="Use kelo_tulip to control platform",
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            use_kelo_tulip_arg,
            declare_base_controller_type,
            declare_arm_controller_type,
            load_joint_state_broadcaster,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[OpaqueFunction(function=select_controller)],
                )
            ),
        ]
    )
