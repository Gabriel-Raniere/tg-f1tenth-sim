import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class WallFollower(Node):
  
  target_distance: float = 1.0 # target distance from wall in m
  current_distance: float = 0.0 # current distance from wall in m
  
  #helpers
  last_laser: LaserScan
  last_odom: Odometry
  
  def __init__(self):
    super().__init__('wall_follower')
    self.scan_subscription = self.create_subscription(
      LaserScan,
      '/scan',
      self.scan_callback,
      10
    )

    self.odom_subscription = self.create_subscription(
      Odometry,
      '/ego_racecar/odom',
      self.odom_callback,
      10
    )
    
    self.ackerman_publisher = self.create_publisher(
      AckermannDriveStamped,
      '/drive',
      10
    )
  
  def scan_callback(self, msg):
    self.last_laser = msg
    self.calculate_current_distance()
    return 1
  
  def odom_callback(self, msg):
    self.last_odom = msg
    return 1
  
  def get_range_by_angle (self, scan: LaserScan, angle: float): 
    # desired_rad_angle = np.deg2rad(angle)
    prossible_desired_angle = angle - (angle % scan.angle_increment)
    
    desired_range_index = int((prossible_desired_angle - scan.angle_min) // scan.angle_increment)
    
    # self.get_logger().info(str(desired_range_index))
    
    return scan.ranges[desired_range_index]
    
  def get_distance_to_wall(self):
    a_angle = np.deg2rad(90)
    b_angle = np.deg2rad(45)
    
    a_range = self.get_range_by_angle(self.last_laser, a_angle)
    b_range = self.get_range_by_angle(self.last_laser, b_angle)
    
    theta = a_angle - b_angle
    
    alpha = np.arctan((a_range*np.cos(theta) - b_range)/(a_range*np.sin(theta)))
    
    return b_range*np.cos(alpha)
  
  def calculate_current_distance(self):
    if self.last_laser == None:
      return
    
    self.get_logger().info(str(self.get_distance_to_wall()))
      
    
    return 1
  

def main(args=None):
  rclpy.init(args=args)

  minimal_subscriber = WallFollower()

  rclpy.spin(minimal_subscriber)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  minimal_subscriber.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
    main()