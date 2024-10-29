from launch import LaunchDescription
from launch_ros.actions import Node    

def generate_launch_description():

    image_processor = Node(
        package="opencv_test_py",
        executable="feature_extractor",
        output="both"
    )

    image_view_node = Node(
        package="image_view",
        executable="image_view",
        arguments=['--ros-args', '--remap', 'image:=/output_image'],
        output="both"
    )

    nodes = [
        image_processor,
        image_view_node]

    return LaunchDescription(nodes)