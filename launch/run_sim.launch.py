import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generates the launch description for the Eddie robot simulation.

    This function loads the Gazebo simulation and the Gazebo controllers.

    Returns:
        LaunchDescription: The launch description for the Eddie robot simulation.
    """

    # Get package directories
    pkg_eddie_share_description = get_package_share_directory("eddie_gazebo")

    load_eddie_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    pkg_eddie_share_description,
                    "launch",
                    "load_gazebo.launch.py",
                )
            ]
        )
    )

    load_gz_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    pkg_eddie_share_description,
                    "launch",
                    "load_gz_controllers.launch.py",
                )
            ]
        )
    )

    return LaunchDescription([load_eddie_gazebo_launch, load_gz_controllers_launch])
