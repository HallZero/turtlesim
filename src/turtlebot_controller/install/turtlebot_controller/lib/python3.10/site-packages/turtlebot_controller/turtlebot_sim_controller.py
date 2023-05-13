#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from tf_transformations import euler_from_quaternion

MAX_DIFF = 0.1

class MyNode():
    def __init__(self, x = 0.0, y = 0.0):
        self.x = x
        self.y = y

class Position(Pose):
    def __init__(self, x = 0.0, y = 0.0, theta = 0.0):
        super().__init__(x = x, y = y, theta = theta)

class TurtlebotSimController(Node):

    def __init__(self, node_list = []):
        super().__init__('turtlebot_sim_controller')

        self.position = Position(x=0.0)
        self.setpoint = Position(x=0.0)
        self.roadmap = node_list

        self.twist_msg_ = Twist()

        self.get_logger().info('turtlebot_sim_controller node created!')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_ = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.timer_ = self.create_timer(1, self.navigate)
    
    def navigate(self):

        if self.setpoint.x == -40.0:
            return
        
        deltaX = self.setpoint.x - self.position.x
        deltaY = self.setpoint.y - self.position.y

        print(deltaX, deltaY)
        print(self.setpoint.x, self.position.x)
        print(self.setpoint.y, self.position.y)

        if self.setpoint.x == self.position.x and self.setpoint.y == self.position.y:
            self.twist_msg_.linear.x = 0.0
            self.twist_msg_.linear.y = 0.0
            self.update_setpoint()
        
        
        self.twist_msg_.linear.x = (deltaX/abs(deltaX))*0.5 if abs(deltaX) > MAX_DIFF else 0.0
        self.twist_msg_.linear.y = (deltaY/abs(deltaY))*0.5 if abs(deltaY) > MAX_DIFF else 0.0

        self.publisher_.publish(self.twist_msg_)        

    def update_setpoint(self):
        try:
            self.setpoint.x = self.position.x + self.roadmap.pop().x
            self.setpoint.y = self.position.y + self.roadmap.pop().y
            print(self.setpoint.x, self.setpoint.y)
            self.get_logger().info("I've reached {self.position}. Now I'm reaching to {self.setpoint}")
        except IndexError:
            self.get_logger().info("Acabei minha jornada!")
            exit()
    
    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        ang = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([ang.x, ang.y, ang.z, ang.w])
        self.get_logger().info(f"x={x}, y={y}, theta={theta}")
        self.update_setpoint()
    

def generateNodeList():
    node_list = []

    continue_input = True

    print("Welcome to the Internet")

    while(continue_input):
        x = float(input('X: '))
        y = float(input('Y: '))
        node = MyNode(x=x, y=y)
        node_list.append(node)
        continue_input = int(input('Do you want do keep adding other nodes? (1/0):'))
    
    return node_list

def main(args=None):
    rclpy.init(args=args)
    
    node_list = generateNodeList()

    turtlebot_sim_controller = TurtlebotSimController(node_list)
    
    rclpy.spin(turtlebot_sim_controller)

    rclpy.shutdown()

if __name__ == '__main__':
    main()