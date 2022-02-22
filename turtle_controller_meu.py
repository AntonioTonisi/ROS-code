#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.srv import RemoveTurtle
from my_robot_interfaces.msg import TurtleArray

from functools import partial

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        self.declare_parameter("catch_closest_turtle_first", True)
        self.catch_closest_turtle_first_ = self.get_parameter("catch_closest_turtle_first").value

        self.pose_ = None
        self.target=None

        self.catch_the_turtles = self.create_subscription(TurtleArray,"alive_turtles",self.callback_turtle_alive,10)


        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, "turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(
            Pose, "turtle1/pose", self.callback_turtle_pose, 10)
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)

    def callback_turtle_pose(self, msg):
        self.pose_ = msg

    def callback_turtle_alive(self,msg):
        if len(msg.turtles) > 0:
            if self.catch_closest_turtle_first_:
                closest_turtle = None
                closest_turtle_distance = None

                for turtle in msg.turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt(dist_x*dist_x + dist_y*dist_y)
                    if closest_turtle == None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                self.target = closest_turtle
            else:
                self.target = msg.turtles[0]

    def control_loop(self):
        if self.pose_ == None or self.target==None:
            return
        dist_x = self.target.x - self.pose_.x
        dist_y = self.target.y - self.pose_.y
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)

        message = Twist()

        if distance > 0.5:
            # position
            message.linear.x = 2*distance

            # orientation
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi

            message.angular.z = 6*diff
        else:
            # target reached!
            message.linear.x = 0.0
            message.angular.z = 0.0
            self.call_the_call_kill_service_xD(self.target.name)
            self.target_name=None
        
        self.cmd_vel_publisher_.publish(message)

    def call_the_call_kill_service_xD(self, name_of_turtle):
        client = self.create_client(RemoveTurtle,"remove_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server.")
        request = RemoveTurtle.Request()
        request.name = name_of_turtle

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_set_led, name_of_the_turtle = name_of_turtle))

    def callback_call_set_led(self, future, name_of_the_turtle):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error("Turtle " + str(name_of_the_turtle) + " could not be caught")
        except Exception as e:
            self.get_logger.error("Service call failed %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()