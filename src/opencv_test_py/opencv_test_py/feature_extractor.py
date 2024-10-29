"""
Azzam Shaikh

Script to extract image features and publish them as topics
"""
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from std_msgs.msg import Int32MultiArray
import cv2 # OpenCV library
import numpy as np
import time
 
class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor
    super().__init__('image_subscriber')

    # Wait until the /camera1/image_raw topic is available
    self.wait_for_topic('/camera1/image_raw')

    # Once available, subscribe to it
    self.subscription = self.create_subscription(
      Image, 
      '/camera1/image_raw', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning

    # Create the image publisher as well as the topic publisher for the 4 image centers
    self.publisher_ = self.create_publisher(Image, 'output_image', 10)
    self.red_publisher_ = self.create_publisher(Int32MultiArray, 'pixel_position/red_circle', 10)
    self.blue_publisher_ = self.create_publisher(Int32MultiArray, 'pixel_position/blue_circle', 10)
    self.green_publisher_ = self.create_publisher(Int32MultiArray, 'pixel_position/green_circle', 10)
    self.purple_publisher_ = self.create_publisher(Int32MultiArray, 'pixel_position/purple_circle', 10)

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    # Define upper and lower HSV bounds for the four different colors
    self.lower_red = np.array([0, 100, 100])  
    self.upper_red = np.array([10, 255, 255])

    self.lower_green = np.array([35, 100, 100])
    self.upper_green = np.array([85, 255, 255])

    self.lower_purple = np.array([125, 100, 100])
    self.upper_purple = np.array([160, 255, 255])

    self.lower_blue = np.array([90, 100, 100])
    self.upper_blue = np.array([130, 255, 255])

  def wait_for_topic(self, topic_name):
      """
      Function to wait until the topic is available. This prevents errors from occurring
      """
      self.get_logger().info(f"Waiting for topic {topic_name}...")
      # Loop through available topics and check if the desired topic is present
      while topic_name not in [name for name, _ in self.get_topic_names_and_types()]:
          time.sleep(0.5)
          self.get_logger().info(f"Still waiting for {topic_name}...")
      self.get_logger().info(f"{topic_name} is now available.")

  @staticmethod
  def find_center(image,lower_range,upper_range,name):
    """
    Function to obtain the center of an image given upper and lower HSV ranges
    """
    # Create a mask for the specified color ranges
    mask = cv2.inRange(image, lower_range, upper_range)

    # Locate the x and y coordinates of all the pixels found in the mask
    pixels = np.column_stack(np.where(mask > 0))

    # In case no pixels are detected, return an empty message 
    if len(pixels) == 0:
        msg = Int32MultiArray()      
        msg.data = [int(0), int(0)]
        return msg 
    
    # Calculate the x and y pixel averages to find the center 
    center_x = np.mean(pixels[:, 1])
    center_y = np.mean(pixels[:, 0])

    # Create the output message that needs to be sent
    msg = Int32MultiArray()      
    msg.data = [int(center_x), int(center_y)]

    return msg
  
  def find_center_of_colors(self, image):
    """
    Function to find the center of all four cirlces in the image
    """

    # Convert the image to HSV colorspace
    hsv_image=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Call the find_center method given an HSV converted image, and the predefined bounds
    red_center = self.find_center(hsv_image,self.lower_red,self.upper_red,'red')

    green_center = self.find_center(hsv_image,self.lower_green,self.upper_green,'green')

    purple_center = self.find_center(hsv_image,self.lower_purple,self.upper_purple, 'purple')

    blue_center = self.find_center(hsv_image, self.lower_blue, self.upper_blue, 'blue')

    # Create a list of the centers
    centers = [red_center, green_center, purple_center,blue_center]

    # Loop through centers and add an small circle to visualize its center
    for center in centers:
      cv2.circle(image, (center.data[0], center.data[1]), 5, (0, 255, 0), -1)

    # Return the updated image
    return centers, image
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    
    # Call the find_center_of_colors to obtain the center of the images
    # Pass a copy of the current frame so the current frame doesnt get modified
    centers, image = self.find_center_of_colors(current_frame.copy())

    output_image = image

    # Publish the image and center topics
    self.publisher_.publish(self.br.cv2_to_imgmsg(output_image, encoding="bgr8"))
    self.red_publisher_.publish(centers[0])
    self.green_publisher_.publish(centers[1])
    self.purple_publisher_.publish(centers[2])
    self.blue_publisher_.publish(centers[3])

  
def main(args=None):
  """
  Main loop of the script
  """
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
