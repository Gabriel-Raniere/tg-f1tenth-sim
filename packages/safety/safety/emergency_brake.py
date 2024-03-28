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
  last_scan: LaserScan = None


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



  def scan_callback(self, msg):
    # self.get_logger().info("1")
    self.last_scan = msg
    self.calculate_ttc()

  def odom_callback(self, msg):
    # self.get_logger().info("2")
    self.last_odom = msg
    self.calculate_ttc()

  def calculate_ttc(self):
    if self.last_scan == None or self.last_odom == None:
      return

    ranges: list[float] = self.last_scan.ranges
    angleMin = self.last_scan.angle_min
    angleMax = self.last_scan.angle_max
    angleIncrement = self.last_scan.angle_increment

    ttcInfoList = []
    for index, range in enumerate(ranges):
      currAngle = index * angleIncrement + angleMin
      rangeX = np.cos(currAngle) * range
      rangeY = np.sin(currAngle) * range
      ttcInfo = {
        "angle": currAngle,
        "range": range,
        "rangeX": np.cos(currAngle) * range,
        "rangeY": np.sin(currAngle) * range,
        "ttcX": rangeX/self.last_odom.twist.twist.linear.x,
        "ttcY": rangeY/self.last_odom.twist.twist.linear.y
      }

      ttcInfoList.append(ttcInfo)

    self.get_logger().info(str(ttcInfoList[len(ttcInfoList)//2]))

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
