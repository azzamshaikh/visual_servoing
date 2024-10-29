"""
Azzam Shaikh

Script to compute the image jacobian and publish the computed twist
"""

# Import necessary libraries
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from cv_bridge import CvBridge 

class Controller(Node):
    """
    Create a Controller class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initalize the Node classes constructor
        super().__init__('image_controller')

        # Define the reference positions of the four circles to be used
        self.ref_red_circle     =(280,394)
        self.ref_green_circle   =(360,369) 
        self.ref_blue_circle    =(256, 314) 
        self.ref_purple_circle  =(336,290) 

        # Create a blank image to plot trajectories on 
        self.trajectory_img = np.zeros((800,800,3), dtype=np.uint8)

        # Define the conversion from pixel to meter (found via Google)
        self.s = 0.0002645833

        # Define the required topics for this script
        self.required_topics = [
            ('/pixel_position/red_circle', Int32MultiArray),
            ('/pixel_position/green_circle', Int32MultiArray),
            ('/pixel_position/blue_circle', Int32MultiArray),
            ('/pixel_position/purple_circle',Int32MultiArray),
            ('/output_image', Image),
        ]

        # Initalize varaibles needed for the script
        self.r, self.c = None, None
        self.br = CvBridge()
        self.img = None

        self.pixel_pose_red = None
        self.red_circle = None
        self.pixel_pose_green = None
        self.green_circle = None
        self.pixel_pose_blue = None
        self.blue_circle = None
        self.pixel_pose_purple = None
        self.purple_circle = None
        self.subscription = None

        # Initalize the twist publisher that the script will publish to
        self.controller_publisher_ = self.create_publisher(Twist, 'vr', 10)

        # Wait until the topics are available
        self.wait_for_topics()

    def wait_for_topics(self):
        """
        Function to wait until all topics are available
        """
        # Loop over all the required topics and wait for each one
        for topic, msg_type in self.required_topics:
            self.get_logger().info(f"Waiting for topic {topic}...")
            self.wait_for_topic(topic, msg_type)

        # Once all topics are available, enable subscriptions
        self.get_logger().info("All required topics are available. Starting subscriptions...")

        self.red_circle = self.create_subscription(Int32MultiArray,
                                                   'pixel_position/red_circle',
                                                   lambda msg: setattr(self, 'pixel_pose_red', msg.data),
                                                   10)

        self.green_circle = self.create_subscription(Int32MultiArray,
                                                     'pixel_position/green_circle',
                                                     lambda msg: setattr(self,'pixel_pose_green', msg.data),
                                                     10)

        self.blue_circle = self.create_subscription(Int32MultiArray,
                                                    'pixel_position/blue_circle',
                                                    lambda msg: setattr(self,'pixel_pose_blue', msg.data),
                                                    10)

        self.purple_circle = self.create_subscription(Int32MultiArray,
                                                      'pixel_position/purple_circle',
                                                      lambda msg: setattr(self,'pixel_pose_purple', msg.data),
                                                      10)


        self.subscription = self.create_subscription(Image,
                                                     '/output_image', 
                                                     self.image_callback, 10)


    def wait_for_topic(self, topic_name, msg_type):
        """
        Wait for a specific topic to be available.
        """
        # Loop through available topics and check if the desired topic is present
        while not self.topic_is_available(topic_name):
            self.get_logger().info(f"Waiting for {topic_name} to become available...")
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info(f"Topic {topic_name} is now available.")

    def topic_is_available(self, topic_name):
        """
        Check if the topic is currently available.
        """
        topic_names_and_types = self.get_topic_names_and_types()
        for (name, types) in topic_names_and_types:
            if name == topic_name:
                return True
        return False

    def get_image_plane_position(self, pixel_location):
        """
        Function to convert from the pixel frame to the image plane frame
        
        Notes:
            x is up
            y is to the left
        """

        if self.r is None and self.c is None:
            self.get_logger().warn("Image dimensions have not been received yet.")
        
        # Initialize the image center
        ox = int(self.r/2)
        oy = int(self.c/2)

        if pixel_location is None:
            self.get_logger().warn("Circle center not showing.")

        # If pixel locations are ready, create the pixel frame vector
        pixel_frame = np.array([pixel_location[0], pixel_location[1], 1]).reshape((3,1))

        # Create the Jacobian 
        J_img_plane_to_pixel = np.array([[-1/self.s, 0, ox],
                                        [0, -1/self.s, oy],
                                        [0, 0, 1]])
        
        # Compute the pose in meters
        pose_in_image = (np.linalg.inv(J_img_plane_to_pixel)@pixel_frame).flatten()

        # Return a 2x1
        return np.array(pose_in_image[0:2])
    
    
    def get_image_Jacobian(self, pose_in_image):
        """
        Function to compute the image jacobian given an x and y position
        Returns a 2x6 vector
        """
        x, y = pose_in_image[0], pose_in_image[1]
        f = 1
        Z = 1
        image_J = np.array([
            [-f*(1/Z), 0, x/Z, (x*y)/f, -(f+(x**2)/f),   y],
            [0, -f*(1/Z), y/Z, -(f+(y**2)/f), -(x*y)/f, -x]
        ])

        return image_J
    
    def get_full_image_Jacobian(self, curr_list):
        """
        Function to compute the image jacobian for multiple features.
        Converts it into an n*2 x 6 where n is the number of features
        """
        jacobian = []

        for index in range(0,4):
            jacobian.append(self.get_image_Jacobian(curr_list[index]))
        
        return np.row_stack(jacobian)
    
    def compute_error(self,ref,curr):
        """
        Function to compute the error between a single feature. 
        Returns a 2x1 vector
        """
        ref_x, ref_y = ref[0], ref[1]
        curr_x, curr_y = curr[0], curr[1]
        error_x = curr_x-ref_x
        error_y = curr_y-ref_y

        return np.array([error_x,error_y]).reshape((2,1))
    
    def compute_error_vector(self,ref_list, curr_list):
        """
        Function to compute the error vector for multiple features
        Converts it into an n*2 x 1 where n is the number of features
        """

        error_vector = []
        for index in range(0,4):
            error_vector.append(self.compute_error(ref_list[index],curr_list[index]))

        return np.row_stack(error_vector)

            

    def controller_callback(self):
        """
        Controller callback function that computes the camera twist for control
        """

        # Define the list of current positions in meters
        current_poses = [self.get_image_plane_position(self.pixel_pose_red),
                         self.get_image_plane_position(self.pixel_pose_blue),
                         self.get_image_plane_position(self.pixel_pose_green),
                         self.get_image_plane_position(self.pixel_pose_purple)]
        
        # Define the list of reference positions in meters
        reference_poses = [self.get_image_plane_position(self.ref_red_circle),
                           self.get_image_plane_position(self.ref_blue_circle),
                           self.get_image_plane_position(self.ref_green_circle),
                           self.get_image_plane_position(self.ref_purple_circle)]
        
        # Compute the error vector (8x1)
        error_vector = self.compute_error_vector(reference_poses, current_poses)
        
        # Compute the image jacobian (8x6)
        Le = self.get_full_image_Jacobian(current_poses)

        # Compute vr = lambda * pinv(Le) * e [scalar * (6x8) @ (8x1)]
        vr = -0.3*(np.linalg.pinv(Le)@error_vector)

        # If the total error between the different circles is < 0.05, the servoing 
        # is complete
        if np.abs(np.sum(error_vector)) < 0.05:
            self.get_logger().info(f"Goal Reached!")
            # Send 0 twist
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0

        else:
            self.get_logger().info(f"Current total error: {np.sum(error_vector)}.")
            # Send vr twist
            msg = Twist()
            msg.linear.x = float(vr[0])
            msg.linear.y = float(vr[1])
            msg.linear.z = float(vr[2])
            msg.angular.x = float(vr[3])
            msg.angular.y = float(vr[4])
            msg.angular.z = float(vr[5])

        # Call the plot function to plot the trajectory 
        self.plot_positions([self.pixel_pose_red,
                             self.pixel_pose_blue,
                             self.pixel_pose_green,
                             self.pixel_pose_purple])
        
        # Publish the twist
        self.controller_publisher_.publish(msg)


    def plot_positions(self, curr_list):
        """
        Function to plot the positions
        """

        # Loop through the list of current poses
        for curr in curr_list:
            # Place a dot at the point where the robot was
            curr_int = (int(curr[0]), int(curr[1]))
            cv2.circle(self.trajectory_img, curr_int, 2, (255, 255, 255), -1)

        # Overlay the trajectory to visualze in real time
        combined_img = cv2.addWeighted(self.img, 0.8, self.trajectory_img, 0.2, 0)

        # Show the plot
        cv2.imshow('Visual Servoing Trajectory', combined_img)
        cv2.waitKey(1)



    def image_callback(self, msg):
        """
        Image callback to obtain the live feed for plotting and img dimensions
        """

        try:
            self.img = self.br.imgmsg_to_cv2(msg)
            self.r, self.c, _ = self.img.shape
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

        # Call the controller callback function
        self.controller_callback()

def main(args=None):
    """
    Main loop of the script
    """
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    controller = Controller()

    # Spin the node so the callback functions are called
    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()