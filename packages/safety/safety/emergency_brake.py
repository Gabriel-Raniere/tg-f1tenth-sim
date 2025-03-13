import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import time


class EmergencyBrake(Node):
  #car specs
  max_deceleration: float = 0.7 #m/s^2
  car_width: float = 0.2 #m
  speed_cutoff: float = 0.1 #m/s
  max_speed: float = 0.0 #m/s
  safety_distance: float = 1.0 #m
  
  #class properties
  ttc = []
  
  #message variables
  last_odom: Odometry = None
  last_scan: LaserScan = None
  
  #helpers
  previous_time: float = None


  def __init__(self):
    super().__init__('emergency_brake')
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



  def scan_callback(self, msg: LaserScan):
    print(self.max_speed)
    self.calculate_ttc(msg)
    self.break_if_needed()
    self.last_scan = msg

  def odom_callback(self, msg):
    self.last_odom = msg

  def calculate_ttc_old(self, curr_scan):
    if self.last_odom == None:
      return
    
    speed_x = self.last_odom.twist.twist.linear.x or 1e-10
    ttc = []
   
    i = 0 
    angle_min = curr_scan.angle_min
    angle_increment = curr_scan.angle_increment
    while i < len(curr_scan.ranges):
      speed_range_angle = i * angle_increment + angle_min
      ttc.append(curr_scan.ranges[i] / max(speed_x * np.cos(speed_range_angle), 1e-10))
      i += 1

    self.ttc = ttc
    middle_index = len(ttc)//2
    self.callCount += 1
    if self.callCount % 10 == 0:
      self.get_logger().info(str(ttc[middle_index]))
  
  def calculate_ttc(self, curr_scan):
    if self.last_odom == None:
      return
    if self.last_scan == None:
      return
    
    curr_time = time.time()
    speeds = []
    ttcs = []
    max_speeds = []
    
    i = 0
    speed_x = self.last_odom.twist.twist.linear.x or 1e-10
    while i < len(self.last_scan.ranges):
      curr_range = curr_scan.ranges[i]
      angle = i * curr_scan.angle_increment + curr_scan.angle_min
      
      # calculate ttc
      speed = speed_x * np.cos(angle)
      speeds.append(speed)

      #calculate max speeds for car front
      i += 1
      if np.abs(np.sin(angle) * curr_range) < self.car_width/2:
        ttc =  (curr_range - self.safety_distance)/speed
        ttcs.append(ttc)
        max_speed = max(self.max_deceleration * ttc, 0)
        max_speeds.append(max_speed)
      else:
        continue
    
    min_max_speed = min(max_speeds)
    if min_max_speed < self.speed_cutoff:
      self.max_speed = 0.0
    else:
      self.max_speed = min_max_speed
  
  def break_if_needed(self):
    if self.last_odom == None:
      return
    ackerman_msg = AckermannDriveStamped()
    ackerman_msg.drive.speed = self.max_speed
    self.ackerman_publisher.publish(ackerman_msg)
    # MIN_TTC = 2 # ttc in seconds
    # for ttc in self.ttc:
    #   if ttc < MIN_TTC:
    #     ackerman_msg = AckermannDriveStamped()
    #     ackerman_msg.drive.speed = 0.0
    #     self.ackerman_publisher.publish(ackerman_msg)


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
