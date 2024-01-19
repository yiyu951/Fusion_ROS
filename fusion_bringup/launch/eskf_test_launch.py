from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    dataloader_node = Node(
        package='dataloader',
        executable='analysis_data_test',
        output="both",
        parameters=[
            {'log_level': "DEBUG"}
        ]
    )

    eskf_node = Node(
        package='eskf',
        executable='eskf_node',
        output="both",
        parameters=[
            # {'log_level': "DEBUG"}
        ]
    )

    nodes = [dataloader_node, eskf_node]

    return LaunchDescription(nodes)
