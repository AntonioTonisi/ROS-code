#!/usr/bin/env python3
import turtle
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random
from functools import partial
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.srv import RemoveTurtle
from turtlesim.srv import Kill
import math

class TurtleSpawner(Node): 
    def __init__(self):
        super().__init__("turtle_spawner") 

        self.declare_parameter("spawn_frequency",1)
        self.spawn_frequency = self.get_parameter("spawn_frequency").value
        
        self.alive_turtles_=[]
        self.turtle_counter=1
        self.turtles_locations=self.create_publisher(TurtleArray,"alive_turtles",10)

        self.remove_the_turtle = self.create_service(RemoveTurtle,"remove_turtle",self.Remove_turtle_finally)

        self.spawn_timer_ = self.create_timer(self.spawn_frequency, self.call_the_Spawn_server)
    
    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.turtles_locations.publish(msg)

    def call_the_Spawn_server(self):
        self.turtle_counter+=1
        turtle_name = "turtle" + str(self.turtle_counter)
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, 2*math.pi)
        self.Spawn_server(turtle_name, x, y, theta)
    
    def Spawn_server(self, turtle_name, x, y, theta):
        client = self.create_client(Spawn,"spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server.")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta=theta
        request.name = turtle_name

        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn, turtle_name=turtle_name, x=x, y=y, theta=theta))
    
    def callback_spawn(self, future, turtle_name, x, y, theta):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info("Turtle " + response.name + " is now alive")
                new_turtle=Turtle()
                new_turtle.x = x
                new_turtle.y=y
                new_turtle.theta=theta
                new_turtle.name=response.name
                
                self.alive_turtles_.append(new_turtle)

                self.publish_alive_turtles()

        except Exception as e:
            self.get_logger.error("Service call failed %r" % (e,))

    def Remove_turtle_finally(self, request, response):
        self.Remove_turtle_server(request.name)
        response.success=True
        return response

    def Remove_turtle_server(self, turtle_name):
        client = self.create_client(Kill,"kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server.")
        
        request = Kill.Request()
        request.name = turtle_name


        future = client.call_async(request)
        future.add_done_callback(partial(self.callback, turtle_name=turtle_name))

    def callback(self, future, turtle_name):
        try:
            future.result()
            for (i, turtle) in enumerate(self.alive_turtles_):
                if turtle.name==turtle_name:
                    del self.alive_turtles_[i]
                    self.publish_alive_turtles()
                    break
        except Exception as e:
            self.get_logger.error("Service call failed %r" % (e,))
        
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner() 
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()