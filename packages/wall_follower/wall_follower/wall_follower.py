import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
import numpy as np
import time

class WallFollower(Node):
  
  target_distance: float = 1.0 # target distance from wall in m
  current_distance: float = 0.0 # current distance from wall in m
  current_angle: float = 0.0 # current angle between car and wall
  predicted_distance_left: float = 0.0 # predicted distance from left wall in m
  predicted_distance_right: float = 0.0 # predicted distance from right wall in m
  
  #helpers
  last_laser: LaserScan = None
  last_odom: Odometry = None
  last_steering_angle: float = 0.0
  dt: float = 0.0
  i: float = 0.0
  current_time = time.time()
  previous_time = time.time()
  
  last_error: float = 0
  
  #plot
  error_list = []
  time_list = []
  
  counter = 0
  
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
    
    self.error_publisher = self.create_publisher(Float64, "/debug/error", 10)
  
  def scan_callback(self, msg):
    self.last_laser = msg
    # self.set_current_distance_and_angle()
    # self.set_predicted_distace(self.dt)
    # # print('predicted distance', self.predicted_distance)
    
    # self.previous_time = self.current_time
    # self.current_time = time.time()
    # self.dt = self.previous_time - self.current_time
    
    self.pid()
    return 1
  
  def odom_callback(self, msg):
    self.last_odom = msg
    return 1
  
  def get_range_by_angle (self, angle: float): 
    prossible_desired_angle = angle - (angle % self.last_laser.angle_increment)
    desired_range_index = int((prossible_desired_angle - self.last_laser.angle_min) // self.last_laser.angle_increment)
    
    return self.last_laser.ranges[desired_range_index]
    
  def get_distance_from_wall(self, direction: str):
    a_angle = 70
    b_angle = 45
    
    if(direction == 'right'):
      a_angle *= -1
      b_angle *= -1
    
    a_range = self.get_range_by_angle(np.deg2rad(a_angle))
    b_range = self.get_range_by_angle(np.deg2rad(b_angle))
    
    theta = a_angle - b_angle
    alpha = np.arctan((b_range*np.cos(theta)-a_range)/(b_range*np.sin(theta)))
    
    
    return b_range*np.cos(alpha)
  
  def get_error(self):
    distance_left_wall = self.get_distance_from_wall('left')
    distance_right_wall = self.get_distance_from_wall('right')
    
    error = distance_left_wall - distance_right_wall
    msg = Float64()
    msg.data = error
    self.error_publisher.publish(msg)
    return distance_left_wall - distance_right_wall
  
  def pid(self):
    k_max = 1
    f_o = 1
    
    kp = 0.5
    ki = 0.1
    kd = 0
    error = self.get_error()
    
    p = kp * error
    i =+ ki * error
    d = kd * (error - self.last_error)
     
    desired_steering_error = p + i + d
    
    
    print('desired steering angle:', np.rad2deg(desired_steering_error))
    self.set_steering_angle(desired_steering_error)
    self.last_error = error
    
    
  def set_current_distance_and_angle(self):
    a_angle = np.deg2rad(60)
    b_angle = np.deg2rad(45)
    
    a_range = self.get_range_by_angle(self.last_laser, a_angle)
    b_range = self.get_range_by_angle(self.last_laser, b_angle)
    
    theta = a_angle - b_angle
    
    alpha = np.arctan((b_range*np.cos(theta)-a_range)/(b_range*np.sin(theta)))
    
    print("current_distance", str(self.current_distance))
    self.current_angle = alpha
    self.current_distance = b_range*np.cos(alpha)
    
  def set_predicted_distace(self, dt):
    if self.last_odom == None or dt == 0:
      return
    
    current_speed = self.last_odom.twist.twist.linear.x
    self.predicted_distance = self.current_distance + dt*current_speed*np.sin(self.current_angle)      
  
  def pid_old(self):
    if self.last_odom == None:
      return
    # current_speed = self.last_odom.twist.twist.linear.x
    predicted_error = self.target_distance - self.predicted_distance
    current_error = self.target_distance - self.current_distance
    
    msg = Float64()
    msg.data = current_error
    self.error_publisher.publish(msg)
    # self.get_logger().info("calculate error: " + str(error))
    
    k_max = -1
    f_o = 0.0001
    
    kp = k_max*0.6
    ki = 2*f_o
    kd = 0.125*f_o
    
    proportional = kp * current_error
    self.integral += ki * current_error * self.dt
    derivative = kd * (predicted_error - current_error)/self.dt
    
    desired_steering_angle = proportional + self.integral + derivative
    
    self.set_steering_angle(desired_steering_angle)
    self.counter = 0
    
  def set_speed(self, speed):
    ackerman_msg = AckermannDriveStamped()
    ackerman_msg.drive.speed = speed
    self.ackerman_publisher.publish(ackerman_msg)
  
  def set_steering_angle(self, angle):
    desired_steering_angle_deg = np.rad2deg(angle)
    
    if abs(desired_steering_angle_deg) < 5:
      speed = 1.5
    elif abs(desired_steering_angle_deg) < 10:
      speed = 1.0
    else:
      speed = 0.5
    
    ackerman_msg = AckermannDriveStamped()
    ackerman_msg.drive.speed = speed
    ackerman_msg.drive.speed = 0.0
    ackerman_msg.drive.steering_angle = angle
    self.ackerman_publisher.publish(ackerman_msg)
    
  def calculate_current_distance(self):
    if self.last_laser == None:
      return
    
    # self.get_logger().info(str(np.rad2deg(self.current_angle)))
      
    
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