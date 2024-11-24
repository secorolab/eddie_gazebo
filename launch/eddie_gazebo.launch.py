import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


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
        return [
            base_controller_map[declared_base_controller],
            *arm_controller_map[declared_arm_controller],
        ]


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

    # enable rviz
    use_rviz = LaunchConfiguration("use_rviz", default=False)

    # Declare the base controller argument with a default value of "velocity"
    declare_base_controller_type = DeclareLaunchArgument(
        "base_controller",
        default_value="velocity",
        description="Specify controller for base (position, velocity, effort) (default: velocity)",
    )

    # Declare path of rviz configuration file
    eddie_rviz_config_file = os.path.join(
        get_package_share_directory("eddie_description"), "config/rviz", "eddie.rviz"
    )

    # Declare the arm controller argument with a default value of "joint_trajectory"
    declare_arm_controller_type = DeclareLaunchArgument(
        "arm_controller",
        default_value="joint_trajectory",
        description="Specify controller for arm (joint_trajectory, effort) (default: joint_trajectory)",
    )

    # Get package directories
    pkg_eddie_gazebo = get_package_share_directory("eddie_gazebo")
    pkg_eddie_share_description = get_package_share_directory("eddie_description")
    pkg_robotiq_share_description = get_package_share_directory("robotiq_description")

    # Resource paths to be added to the GZ_SIM_RESOURCE_PATH environment variable
    resource_paths = [
        os.path.join(pkg_eddie_share_description, ".."),
        os.path.join(pkg_robotiq_share_description, ".."),
    ]
    resource_paths_str = ":".join(resource_paths)

    gz_reource_env_var = "GZ_SIM_RESOURCE_PATH"

    # check if the environment variable is already set
    if gz_reource_env_var in os.environ:
        resource_paths_str = os.environ[gz_reource_env_var] + ":" + resource_paths_str

        # append the new path to the existing path
        set_env_vars_resources = AppendEnvironmentVariable(
            name=gz_reource_env_var, value=resource_paths_str
        )

    else:
        # set the new path
        set_env_vars_resources = SetEnvironmentVariable(
            name=gz_reource_env_var, value=resource_paths_str
        )

    # World and robot files
    world_file = os.path.join(pkg_eddie_gazebo, "worlds", "eddie_basic_world.sdf")

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

    # load eddie
    load_eddie_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("eddie_description"),
                    "launch",
                    "load_eddie.launch.py",
                )
            ]
        ),
        launch_arguments={
            "use_ros2_control": "true",
            "use_gz_sim": "true",
        }.items(),
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
            "eddie",
            "-allow_renaming",
            "true",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.01",
        ],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", eddie_rviz_config_file],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    # gazebo launch description
    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])]
        ),
        launch_arguments={
            # "gz_args": f" -r -v 1 --physics-engine gz-physics-bullet-featherstone-plugin {world_file}"
            "gz_args": f" -r -v 1 {world_file}"
        }.items(),
    )

    return LaunchDescription(
        [
            set_env_vars_resources,
            declare_base_controller_type,
            declare_arm_controller_type,
            rviz2_node,
            gz_launch_description,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[OpaqueFunction(function=select_controller)],
                )
            ),
            load_eddie_launch,
            gz_spawn_entity,
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="If true, use simulated clock",
            ),
        ]
    )
