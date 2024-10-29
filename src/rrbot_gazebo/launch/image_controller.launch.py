from launch import LaunchDescription
from launch_ros.actions import Node    

def generate_launch_description():

    image_controller = Node(
        package="controller",
        executable="visual_servoing",
        output="both"
    )

    nodes = [
        image_controller
    ]

    return LaunchDescription(nodes)