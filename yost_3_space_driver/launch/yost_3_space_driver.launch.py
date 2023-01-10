import os

import ament_index_python.packages
import launch
import launch_ros.actions

def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(package='yost_3_space_driver').find('yost_3_space_driver')

    # yost_3_space_driver_node
    start_yost_3_space_driver_cmd = launch_ros.actions.Node(
        package='yost_3_space_driver',
        node_executable='yost_3_space_driver_node',
        node_name='yost_3_space_driver_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'yost_3_space.yaml')],
    )

    # Create the launch description and populate
    ld = launch.LaunchDescription()

    ld.add_action(start_yost_3_space_driver_cmd)

    return ld