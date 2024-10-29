"""
Azzam Shaikh

Script to extract print out image centers for validation
"""
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
 
class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.

    self.subscription = self.create_subscription(
      Image, 
      '/camera1/image_raw', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning

    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'output_image', 10)
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    # Define upper and lower HSV bounds for the four different circles
    self.lower_red = np.array([0, 100, 100])  
    self.upper_red = np.array([10, 255, 255])

    self.lower_green = np.array([35, 100, 100])
    self.upper_green = np.array([85, 255, 255])

    self.lower_purple = np.array([125, 100, 100])
    self.upper_purple = np.array([160, 255, 255])

    self.lower_blue = np.array([90, 100, 100])
    self.upper_blue = np.array([130, 255, 255])

  @staticmethod
  def find_center(image,lower_range,upper_range,name):
    """
    Function to obtain the center of an image given upper and lower HSV ranges
    """
    # Create a mask for the specified color ranges
    mask = cv2.inRange(image, lower_range, upper_range)

    # Locate the x and y coordinates of all the pixels found in the mask
    pixels = np.column_stack(np.where(mask > 0))

    # Calculate the x and y pixel averages to find the center 
    center_x = np.mean(pixels[:, 1])
    center_y = np.mean(pixels[:, 0])
    print(f"Median: The center of the {name} circle is at: ({center_x:.2f}, {center_y:.2f})")

    # Return the center
    return (center_x, center_y)
  
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
    centers = [red_center,green_center,purple_center,blue_center]
    
    # Loop through centers and add an small circle to visualize its center
    for center in centers:
      cv2.circle(image, (int(center[0]), int(center[1])), 5, (0, 255, 0), -1)

    # Return the updated image
    return image
  
  def canny(self,image):
    """
    Function to apply the canny edge detector to the image
    """
    # Convert the image to a gray scale image
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Call and return the output from the Canny function with given low and high threshold values
    return cv2.Canny(gray_image,100,200)

  def corner(self,image):
    """
    Function to obtain the corners in the image
    """
    # Convert the image to a gray scale image
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Convert the gray scale to a floating-point image as required for the cornerHarris input
    gray = np.float32(gray)

    # Call the cornerHarris method to obtain the harris corner detector responses
    dst = cv2.cornerHarris(gray,2,3,0.04)

    # Dilate the responses to have a larger areas; this will help show the response better in the image
    dst = cv2.dilate(dst,None,iterations=3)

    # Threshold the image using the responses and set them to be red to highlight the corners
    image[dst>0.01*dst.max()]=[0,0,255]

    # Return the updated image
    return image

  def hough_circles(self,image):
    """
    Function to obtain the Hough circles in the image
    """

    # Apply a median blue to smooth the image
    image = cv2.medianBlur(image,5)

    # Convert the iamge to a gray scale image
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Call the Hough Circles function to obtain a vector of circles
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30)

    # Convert the floating point outputs to integer values
    circles = np.uint16(np.round(circles))

    # Loop through the vector of cirlces
    for i in circles[0,:]:
        # For each circle:
        # draw the circle, the first two values are the x and y center positions, and the third is the radius
        cv2.circle(image,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle as a circle with a predefined radius
        cv2.circle(image,(i[0],i[1]),2,(0,0,255),3)
        print(f"Hough Circles: The center of the circle is at: ({i[0]:.2f}, {i[1]:.2f})")

    # Return the updated image
    return image


   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    # self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)

    # PLACE YOUR CODE HERE. PROCESS THE CURRENT FRAME AND PUBLISH IT. IF YOU ARE HAVING DIFFICULTY PUBLISHING IT YOU CAN USE THE FOLLOWING LINES TO DISPLAY IT VIA OPENCV FUNCTIONS
    #cv2.imshow("output_image", current_frame)
    #cv2.waitKey(1)
    
    # Call the find_center_of_colors to obtain the center of the images
    # Pass a copy of the current frame so the current frame doesnt get modified
    centers = self.find_center_of_colors(current_frame.copy())

    # Call the canny method to obtain the edges in the iamge
    # Pass a copy of the current frame so the current frame doesnt get modified
    # The output will be visualized via OpenCVs imshow method as the centers image is being published already
    # edges = self.canny(current_frame.copy())

    # Call the corner method to obtain the corners in the image
    # Pass a copy of the current frame so the current frame doesnt get modified
    # The output will be visualized via OpenCVs imshow method as the centers image is being published already
    # corners = self.corner(current_frame.copy())

    # Call the hough_cirlces method to obtain the hough circles in the image
    # Pass a copy of the current frame so the current frame doesnt get modified
    # The output will be visualized via OpenCVs imshow method as the centers image is being published already
    circles = self.hough_circles(current_frame.copy())
    
    # cv2.imshow('Centers', centers)
    # cv2.waitKey(1)

    # cv2.imshow('Edges', edges)
    # cv2.waitKey(1)

    # cv2.imshow('Corners',corners)
    # cv2.waitKey(1)

    cv2.imshow('Circles', circles)
    cv2.waitKey(1)

    output_image = centers

    # Publish the image.
    # The 'cv2_to_imgmsg' method converts an OpenCV
    # image to a ROS 2 image message
    self.publisher_.publish(self.br.cv2_to_imgmsg(output_image, encoding="bgr8"))

  
def main(args=None):
  
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
