import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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

    # Load and activate the arm effort controllers
    load_arm_left_effort_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "arm_left_effort_controller",
        ],
        output="screen",
    )

    load_arm_right_effort_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "arm_right_effort_controller",
        ],
        output="screen",
    )

    # Load and activate the arm joint trajectory controllers
    load_joint_trajectory_controller_left = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "arm_left_joint_trajectory_controller",
        ],
        output="both",
    )
    load_joint_trajectory_controller_right = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "arm_right_joint_trajectory_controller",
        ],
        output="both",
    )

    # Load and activate the base controllers
    load_joint_position_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "base_position_controller",
        ],
        output="screen",
    )

    load_joint_velocity_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "base_velocity_controller",
        ],
        output="screen",
    )

    load_joint_effort_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "base_effort_controller",
        ],
        output="screen",
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
        return [
            base_controller_map[declared_base_controller]
        ]  # , *arm_controller_map[declared_arm_controller]]


def generate_launch_description():
    """Generates the launch description for the Eddie robot simulation.

    This function sets up the environment variables, declares the base and arm controllers,
    launches the Gazebo simulation, loads the selected controllers or default controllers,
    and starts the robot state publisher.

    Returns:
        LaunchDescription: The launch description for the Eddie robot simulation.
    """
    # Configuration to use simulation time (use by default)
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

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

    # Get package directories
    pkg_eddie_gazebo = get_package_share_directory("eddie_gazebo")
    pkg_eddie_description = get_package_share_directory("eddie_description")

    # Resource paths to be added to the GZ_SIM_RESOURCE_PATH environment variable
    resource_paths = [
        pkg_eddie_description,
        os.path.join(pkg_eddie_description, "eddie_base_description", "meshes"),
        os.path.join(
            pkg_eddie_description, "eddie_base_description", "meshes", "sensors"
        ),
        os.path.join(pkg_eddie_description, "eddie_torso_description", "meshes"),
    ]
    resource_paths_str = os.pathsep.join(resource_paths)

    # Set the environment variable GZ_SIM_RESOURCE_PATH to include the resource paths
    set_env_vars_resources = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=resource_paths_str,
    )

    # World and robot files
    world_file = os.path.join(pkg_eddie_gazebo, "worlds", "my_world.sdf")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("eddie_description"), "robots", "eddie.urdf.xacro"]
            ),
        ]
    )

    # Load the joint state broadcaster controller on robot spawn -- see function return
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    # Parameters for the robot state publisher
    robot_description_params = {"robot_description": robot_description_content}

    # Robot state publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description_params, {"use_sim_time": True}],
    )

    # Spawn entity
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "robot",
            "-allow_renaming",
            "true",
        ],
    )

    return LaunchDescription(
        [
            set_env_vars_resources,
            # Declare and set base and arm controllers through launch arguments
            declare_base_controller_type,
            declare_arm_controller_type,
            # Launch gazebo environment
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ros_gz_sim"),
                                "launch",
                                "gz_sim.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments=[("gz_args", [f" -r -v 1 {world_file}"])],
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            # Load selected controllers or default controllers
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[OpaqueFunction(function=select_controller)],
                )
            ),
            node_robot_state_publisher,
            gz_spawn_entity,
            # Launch Arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="If true, use simulated clock",
            ),
        ]
    )
