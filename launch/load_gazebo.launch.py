import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    AppendEnvironmentVariable,
    SetEnvironmentVariable,
)
from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher(use_sim_time=True)

    sl.declare_arg("use_rviz", default_value="false", description="Use RViz")
    sl.declare_arg(
        "use_floorplan_model", default_value="True", description="Use floorplan model"
    )
    sl.declare_arg(
        "floorplan_model_name",
        default_value="brsu_building_c_level_2",
        description="Name of the floorplan model",
    )
    sl.declare_arg(
        "use_kelo_tulip",
        default_value="false",
        description="Use kelo_tulip to control platform",
    )

    gz_bridge_config_path = sl.find("eddie_gazebo", "gz_ros_bridge.yaml", "config")

    # Get package directories
    pkg_eddie_gazebo = get_package_share_directory("eddie_gazebo")
    pkg_eddie_share_description = get_package_share_directory("eddie_description")
    pkg_robotiq_share_description = get_package_share_directory("robotiq_description")

    # Resource paths to be added to the GZ_SIM_RESOURCE_PATH environment variable
    resource_paths = [
        os.path.join(pkg_eddie_gazebo, ".."),
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

    sl.add_action(set_env_vars_resources)

    # World and robot files
    world_file = sl.find("eddie_gazebo", "eddie_basic_world.sdf", "worlds")

    # Load eddie_description
    eddie_description_args = {
        "use_kelo_tulip": sl.arg("use_kelo_tulip"),
        "use_sim_time": "true",
        "use_ros2_control": "true",
        "use_gz_sim": "true",
    }
    sl.include(
        "eddie_description",
        "load_eddie.launch.py",
        launch_arguments=eddie_description_args,
    )

    # launch gz sim
    sl.gz_launch(world_file, gz_args=" -r -v 1")

    # Spawn eddie model
    sl.spawn_gz_model(
        "eddie",
        "robot_description",
        spawn_args=["x", "0", "y", "0", "z", "0.01", "allow_renaming", "true"],
    )

    # Launch gz bridge
    sl.node(
        "ros_gz_bridge",
        "parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={gz_bridge_config_path}"],
    )

    # spawn floorplan model
    with sl.group(if_arg="use_floorplan_model"):
        model_sdf = sl.find(
            "eddie_gazebo",
            sl.arg("floorplan_model_name") + ".sdf",
            "meshes/floorplan-gen/" + sl.arg("floorplan_model_name"),
        )
        sl.spawn_gz_model(
            "floorplan_model",
            model_file=model_sdf,
            spawn_args=["x", "0", "y", "0", "z", "0", "allow_renaming", "true"],
        )

    with sl.group(if_arg="use_rviz"):
        sl.rviz(sl.find("eddie_description", "eddie.rvviz", "config/rviz"))

    return sl.launch_description()
