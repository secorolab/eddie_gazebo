import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    """Generates the launch description for the Eddie robot gazebo simulation.

    This function sets up the environment variables, launches the Gazebo simulation.

    Returns:
        LaunchDescription: The launch description for the Eddie robot simulation.
    """
    # Configuration to use simulation time (use by default)
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    # enable rviz
    use_rviz = LaunchConfiguration("use_rviz", default=False)

    # Declare path of rviz configuration file
    eddie_rviz_config_file = os.path.join(
        get_package_share_directory("eddie_description"), "config/rviz", "eddie.rviz"
    )

    use_kelo_tulip_arg = DeclareLaunchArgument(
        "use_kelo_tulip",
        default_value="false",
        description="Use kelo_tulip to control platform"
    )
    
    use_kelo_tulip = LaunchConfiguration("use_kelo_tulip")

    gz_bridge_config_path = os.path.join(
        get_package_share_directory("eddie_gazebo"),
        'config',
        'gz_ros_bridge.yaml'
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
            "use_kelo_tulip": use_kelo_tulip,
            "use_sim_time": use_sim_time,
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
            [
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                )
            ]
        ),
        launch_arguments={
            # "gz_args": f" -r -v 1 --physics-engine gz-physics-bullet-featherstone-plugin {world_file}"
            "gz_args": f" -r -v 1 {world_file}"
        }.items(),
    )

    bridge_gz_ros_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_config_path}'
        ],
        output='screen'
    )
    
    return LaunchDescription(
        [
            use_kelo_tulip_arg,
            set_env_vars_resources,
            rviz2_node,
            gz_launch_description,
            load_eddie_launch,
            gz_spawn_entity,
            bridge_gz_ros_node,
        ]
    )
