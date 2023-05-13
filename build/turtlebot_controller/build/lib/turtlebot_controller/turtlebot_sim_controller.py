#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from tf_transformations import euler_from_quaternion
import math

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
        self.setpoint = Position(x=-40.0)
        self.roadmap = node_list

        self.twist_msg_ = Twist()

        self.get_logger().info('turtlebot_sim_controller node created!')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_ = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.timer_ = self.create_timer(0.02, self.navigate)
    
    def navigate(self):

        if self.setpoint.x == -40.0:
            #print("Waiting for new setpoint...")
            return
        
        deltaX = self.setpoint.x - self.position.x
        deltaY = self.setpoint.y - self.position.y
        angle_to_goal = math.atan2(deltaY, deltaX)

        print(angle_to_goal - self.position.theta)

        if self.setpoint.x < self.position.x + MAX_DIFF and self.setpoint.x > self.position.x - MAX_DIFF and self.setpoint.y > self.position.y - MAX_DIFF and self.setpoint.y < self.position.y + MAX_DIFF:
            self.twist_msg_.linear.x = 0.0
            self.twist_msg_.linear.y = 0.0
            self.twist_msg_.angular.z = 0.0
            self.update_setpoint()
        
        
        if abs(angle_to_goal - self.position.theta) >= 0.15 and abs(angle_to_goal - self.position.theta) <= 5.8:
            self.twist_msg_.linear.x = 0.0
            self.twist_msg_.angular.z = 0.3 if angle_to_goal - self.position.theta >= 0 else -0.3
        else:
            if abs(deltaX) >= MAX_DIFF or abs(deltaY) >= MAX_DIFF:
                self.twist_msg_.linear.x = 0.7

            self.twist_msg_.angular.z = 0.0
        

        self.publisher_.publish(self.twist_msg_)        

    def update_setpoint(self):
        try:
            node = self.roadmap.pop(0)
            self.setpoint.x = node.x
            self.setpoint.y = node.y
            print(self.setpoint.x, self.setpoint.y)
            #self.get_logger().info(f"I've reached {self.position}. Now I'm reaching to {self.setpoint}")
        except IndexError:
            self.get_logger().info("Acabei minha jornada!")
            self.twist_msg_.linear.x = 0.0
            self.twist_msg_.angular.z = 0.0
            self.publisher_.publish(self.twist_msg_)
            exit()
    
    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        ang = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([ang.x, ang.y, ang.z, ang.w])
        #self.get_logger().info(f"x={x}, y={y}, theta={theta}")
        self.position.x = x
        self.position.y = y
        self.position.theta = theta
        if self.setpoint.x == -40.0:
            self.update_setpoint()
    

def generateNodeList():
    node_list = []

    continue_input = True

    print("Welcome to the turtlebot3 simulation controller!")

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