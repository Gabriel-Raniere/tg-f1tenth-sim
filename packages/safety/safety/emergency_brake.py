import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np


class EmergencyBrake(Node):
  #car specs
  maxDeceleration: float = 1 #m/s^2
  safetyRadius: float = 0.3 #m

  #message variables
  last_odom: Odometry = None
  
  #helpers
  callCount: int = 0


  def __init__(self):
    super().__init__('emergency_brake')
    self.subscription = self.create_subscription(
      LaserScan,
      '/scan',
      self.scan_callback,
      10
    )

    self.subscription = self.create_subscription(
      Odometry,
      '/ego_racecar/odom',
      self.odom_callback,
      10
    )



  def scan_callback(self, msg: LaserScan):
    self.calculate_ttc(msg)
    self.last_scan = msg

  def odom_callback(self, msg):
    self.last_odom = msg

  def calculate_ttc(self, curr_scan):
    if self.last_odom == None:
      return
    
    speed_x = self.last_odom.twist.twist.linear.x or 1e-10

    info = {
      "curr_scan_ranges": np.array(curr_scan.ranges),
      "last_scan_ranges": np.array(self.last_scan.ranges),
      "ttc": [] 
    }
   
    i = 0 
    angle_min = curr_scan.angle_min
    angle_increment = curr_scan.angle_increment
    while i < len(curr_scan.ranges):
      speed_range_angle = i * angle_increment + angle_min
      info["ttc"].append(curr_scan.ranges[i] / (speed_x * np.cos(speed_range_angle)))
      i += 1

    middle_index = len(info['delta_ranges'])//2
    self.callCount += 1
    if self.callCount % 10 == 0:
      self.get_logger().info(str(info['ttc'][middle_index]))

def main(args=None):
  rclpy.init(args=args)

  minimal_subscriber = EmergencyBrake()

  rclpy.spin(minimal_subscriber)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  minimal_subscriber.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
    main()
