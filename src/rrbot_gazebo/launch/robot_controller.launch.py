from launch import LaunchDescription
from launch_ros.actions import Node    

def generate_launch_description():

    robot_controller = Node(
        package="robot_controller",
        executable="rrbot_controller",
        output="both"
    )

    nodes = [
        robot_controller
    ]

    return LaunchDescription(nodes)