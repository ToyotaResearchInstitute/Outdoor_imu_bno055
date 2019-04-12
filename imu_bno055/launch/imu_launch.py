import os

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('imu_bno055'),
        'config')

    imu_params = os.path.join(config_directory, 'default_imu_bno055_params.yaml')
    imu = launch_ros.actions.Node(package='imu_bno055',
                                  node_executable='bno055_i2c_node',
                                  output='screen',
                                  parameters=[imu_params])

    return launch.LaunchDescription([imu,
                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=imu,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])
