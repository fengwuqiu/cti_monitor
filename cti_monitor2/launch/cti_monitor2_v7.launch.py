from launch import LaunchDescription
import launch_ros.actions
import ament_index_python


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='cti_monitor2',
            executable='cti_monitor2_node',
            parameters=[
                {'config_file_dir': ament_index_python.get_package_share_directory("cti_monitor2") +'/config/'},
                {'ip_address_config_file': 'ip_address_config.yaml'},
                {'topic_state_config_file': 'topic_state.yaml'},
                {'node_list_config_file': 'node_list.yaml'},
            ],
            output='screen',
            # respawn=True
        ),
    ])
