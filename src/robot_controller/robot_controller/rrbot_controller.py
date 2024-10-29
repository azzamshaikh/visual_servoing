"""
Azzam Shaikh

Script to compute the convert the camera twist to joint velocities
"""

# Import necessary libraries
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np
import time

class RRBotController(Node):
    """
    Create a RRBotController class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initalize the Node classes constructor
        super().__init__('RRBotController')

        # Initialize necessary variables
        self.msg = Float64MultiArray()
        self.vr = np.zeros((6, 1))

        # Initalize the publisher for the joint commands
        self.publisher_ = self.create_publisher(Float64MultiArray, 'forward_velocity_controller/commands', 10)

        # Wait for topics to pop up
        self.wait_for_topic('/vr')
        self.wait_for_topic('/joint_states')

        # After topics are up, create the subscribers
        self.twist_subscriber_ = self.create_subscription(Twist,'vr',self.twist_callback,10)
        self.joint_pose_subscriber = self.create_subscription(JointState,'joint_states',self.joint_states_callback, 10)

    def wait_for_topic(self, topic_name):
        """
        Function to wait for the topic to be available
        """
        self.get_logger().info(f"Waiting for topic {topic_name}...")
        # Loop through available topics and check if the desired topic is present
        while topic_name not in [name for name, _ in self.get_topic_names_and_types()]:
            time.sleep(0.5)
            self.get_logger().info(f"Still waiting for {topic_name}...")
        self.get_logger().info(f"{topic_name} is now available.")
        
    def twist_callback(self, msg:Twist):
        """
        FUnction that extract the twist data and create it into a numpy array
        """
        vr = np.empty(6)
        vr[0] = msg.linear.x
        vr[1] = msg.linear.y
        vr[2] = msg.linear.z
        vr[3] = msg.angular.x
        vr[4] = msg.angular.y 
        vr[5] = msg.angular.z
        self.vr = vr.reshape((6,1))

    def joint_states_callback(self, msg:JointState):
        """
        Function that gets the current joint positions and jacobian
        """

        # Get current joint positions
        current = msg.position
        j1 = current[0]
        j2 = current[1]

        # Predefine variables for the matrix
        l1, l2 = 1,1
        J00 = l1*np.sin(j1)-l2*np.sin(j1+j2)
        J01 = -l2*np.sin(j1+j2)
        J10 = l1*np.cos(j1)+l2*np.cos(j1+j2)
        J11 = l2*np.cos(j1+j2)

        # Create the Jacobian
        self.jacobian = np.array([[J00, J01],
                                  [J10, J11],
                                  [0, 0],
                                  [0, 0],
                                  [0, 0],
                                  [1, 1]])
        
        # Publish joint commands from below
        self.send_joint_commands()

    def send_joint_commands(self):
        """
        Function to publish the joint commands
        """
        # Compute joint commands
        joint_cmds = (np.linalg.pinv(self.jacobian)@self.vr).flatten()
        # self.get_logger().info(f"Sending joint commands: {joint_cmds}")

        # Convert the joint commands to appropriate data format
        self.msg.data = [float(joint_cmds[0]), float(joint_cmds[1])]

        # Publish the message
        self.publisher_.publish(self.msg)


def main(args=None):
    """
    Main loop of the script
    """
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    controller = RRBotController()

    # Spin the node so that the callback functions are called
    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()