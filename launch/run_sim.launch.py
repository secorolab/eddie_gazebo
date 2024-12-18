from simple_launch import SimpleLauncher


def generate_launch_description():
    """Generates the launch description for the Eddie robot simulation.

    This function loads the Gazebo simulation and the Gazebo controllers.

    Returns:
        LaunchDescription: The launch description for the Eddie robot simulation.
    """

    sl = SimpleLauncher()

    sl.declare_arg(
        "use_kelo_tulip", "false", description="Use kelo_tulip for platform control"
    )

    sl.include(
        "eddie_gazebo",
        "load_gazebo.launch.py",
        launch_arguments={"use_kelo_tulip": sl.arg("use_kelo_tulip")},
    )

    sl.include(
        "eddie_gazebo",
        "load_gz_controllers.launch.py",
        launch_arguments={"use_kelo_tulip": sl.arg("use_kelo_tulip")},
    )

    return sl.launch_description()
