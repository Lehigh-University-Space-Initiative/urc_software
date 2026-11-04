#!/usr/bin/env python3
import math
#Import ROS 2 Python library and message types.
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist

#Create a class that inherits from Node, which is the base class for all ROS 2 nodes.
class BehaviorController(Node):
    #Creates a constructor for a BehaviorController object
    def __init__(self):
        #Calls node constructor with the name "behavior_controller"
        super().__init__("behavior_controller")
        #Sets default behavior to stay
        self.behavior = "stay"
        #Sets the default person position and last update time to None
        self.person_position = None
        self.last_person_update = None
        #Creates a subscription to the topic "behavior_command" with type string, 
        #calls self.on_command when a message is received, and sets the queue size to 10.
        self.create_subscription(String, "behavior_command", self.on_command, 10)
        #Creates a subscription to the topic "person_position" with type Point,
        #calls self.on_person_position when a message is received, and sets the queue size to 10.
        self.create_subscription(Point, "person_position", self.on_person_position, 10)
        #Creates a publisher to the topic cmd_vel that contains twists with a queue size of 10. This is used to send movement commands to the robot.
        self.drive_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Decide what to do 10 times per second.
        self.create_timer(0.1, self.update)

    def on_command(self, message):
        #Strip command down to essential form and make it lowercase for comparison.
        command = message.data.strip().lower()
        #Check if it is a registered command, if not, log a warning and return.
        if command not in {"stay", "follow", "come", "fetch"}:
            self.get_logger().warning(
                f"Unknown command: {command}"
            )
            return
        #sets current behavior to the command and logs the behavior change.
        self.behavior = command
        #displays behavior change in the console for debugging purposes.
        self.get_logger().info(f"Behavior: {command}")

        #sends a twist message without parameters
        #this sends a stop command to the robot to ensure it stops moving when the behavior changes.
        self.drive_publisher.publish(Twist())
    #creates a function called when a message is received on the person_position topic. 
    #It updates the person_position attribute with the new message and records the time of the update.
    def on_person_position(self, message):
        self.person_position = message
        self.last_person_update = self.get_clock().now()
    #defines a function to update the rovers movement every 0.1 seconds
    def update(self):
        # A fresh Twist starts with every speed set to zero.
        movement = Twist()
        #if behavior is stay then just do nothing
        if self.behavior == "stay":
            pass
            #if behavior is fetch then it will go to object
            #will be added later
        elif self.behavior == "fetch":
            # Add the object approach / pickup / return sequence here.
            # Until then, remain stopped.
            pass
            
        elif self.behavior in {"come", "follow"}:
            if self.person_position is not None:
                #creates a variable for age of the last person position update in seconds
                age = (
                    self.get_clock().now() - self.last_person_update
                ).nanoseconds / 1e9

                # Only move while receiving recent position updates.
                if age < 1.0:
                    #calculates the distance to the person using the Pythagorean theorem
                    distance = math.hypot(
                        self.person_position.x,
                        self.person_position.y,
                    )
                    #sets desired distance to astronaut to 2 meters away from the rover
                    desired_distance = 2.0
                    #if rover is within desired distance then it will stop moving, otherwise it will move toward the person
                    if distance <= desired_distance:
                        if self.behavior == "come":
                            self.behavior = "stay"
                    else:
                        #if not it sets the movement variable to the result of the move_toward_person function,
                        #which calculates the appropriate movement command based on the distance to the person and the desired distance.
                        movement = self.move_toward_person(
                            distance, desired_distance
                        )
        #publishes movement command to the cmd_vel topic, which will be received by the rover's motor controller 
        #to move the rover accordingly.
        self.drive_publisher.publish(movement)
        
    def move_toward_person(self, distance, desired_distance):
        #creates an empty Twist message to hold the movement command
        movement = Twist()
        #finds the angle to the person
        angle = math.atan2(
            self.person_position.y,
            self.person_position.x,
        )

        # Positive angle means the person is to the rover's left.
        #caps the angular velocity to a maximum of 0.6 rad/s in either direction to prevent the rover from
        #turning too quickly
        movement.angular.y =- math.degrees(max(-0.6, min(0.6, angle)))

        # Drive only while approximately facing the person.
        if abs(angle) < math.radians(30):
            gap = distance - desired_distance
            #moves the rover forward at a speed proportional to the gap
            #between the current distance and the desired distance,
            movement.linear.x = min(0.4, 0.3 * gap)

        return movement


def main():
    #initializes the ROS 2 Python client library, creates an instance of the BehaviorController class,
    #  and starts spinning the node to process incoming messages and timers. 
    # When the node is shut down, it sends a stop command to the rover and cleans up resources.
    rclpy.init()
    controller = BehaviorController()
    #initializes the node and starts spinning it to process incoming messages and timers. 
    #The try-finally block ensures that the rover stops moving and resources are cleaned up when the node is shut down.
    try:
        rclpy.spin(controller)
    finally:
        controller.drive_publisher.publish(Twist())
        controller.destroy_node()
        rclpy.shutdown()

#runs main if the file is executed as a script, allowing the BehaviorController node to be started and run independently.
if __name__ == "__main__":
    main()