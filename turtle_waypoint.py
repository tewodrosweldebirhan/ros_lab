#!/usr/bin/env python
import rospy

import sys  # command line arguments argv
import math  # atan2

# TODO: Import the messages we need
# import ...
# import ...

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

class TurtleWaipoint(object):
    """Class to guide the turtle to the specified waypoint."""

    def __init__(self, waypoint_x=None, waypoint_y=None):
        """Class constructor."""
        # Init all variables
        # Current turtle position
        self.x = None
        self.y = None
        self.theta = None
        # Tolerance to reach waypoint
        self.tolerance = 0.1
        # A position was received
        self.got_position = False
        # Reached position
        self.finished = False

        # ROS init
        rospy.init_node('turtle_waypoint')
        # TODO: Define pulisher: topic name, message type
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        # TODO: Define subscriber: topic name, message type, function callback
        self.sub = rospy.Subscriber('/turtle1/pose', Pose, self.callback)

        # Retrieve waypoint
        self.waypoint_x = waypoint_x
        self.waypoint_y = waypoint_y
        if waypoint_x is None or waypoint_y is None:
            # No waypoint specified => look at the param server
            if rospy.has_param('default_x') and rospy.has_param('default_y'):  # TODO: change for the correct expression
                print("Waypoint found in param server")
                # TODO: Save params from param server
                self.waypoint_x = rospy.get_param('default_x')  # TODO: change for the correct expression
                self.waypoint_y = rospy.get_param('default_y')  # TODO: change for the correct expression
            else:
                # No waypoint in param server => finish
                print("No waypoint found in param server")
                exit(1)
        # Show the waypoint
        print('Heading to: {:.2f}, {:.2f}'.format(self.waypoint_x,
                                                  self.waypoint_y))

    def callback(self, msg):
        """Saves the tutle position when a message is received."""
        # TODO: store the position in self.x, self.y and self.theta variables.
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.got_position = True

    def iterate(self):
        """Keeps sending control commands to turtle until waypoint reached."""
        if self.finished:
            print('Waypoint reached')
            exit(0)
        else:
            # We know where we are
            if self.got_position:
                #
                velocity = Twist()

                linear_const = 2                   
                angular_const = 3

                change_x = self.waypoint_x - self.x
                change_y = self.waypoint_y - self.y
                steering_angle = atan2(change_y, change_x) - self.theta

                error_pos = sqrt(pow((self.waypoint_x - self.x), 2) +
                                  pow((self.waypoint_y - self.y), 2))                
                
                if error_pos < self.tolerance:  # TODO: change for the correct expression
                    # Waypoint reached
                    self.finished = True                    

                else:
                    # Waypoint not reached yet
                    # TODO: Send a velocity command towards waypoint
                    #                    
                    
                    velocity.linear.x = linear_const * error_pos
                    velocity.linear.y = 0.0
                    velocity.linear.z = 0.0

                    velocity.angular.x = 0.0    
                    velocity.angular.y = 0.0    
                    velocity.angular.z = angular_const * (steering_angle)                                       

                    

                    self.pub.publish(velocity)



if __name__ == '__main__':
    # Check commandline inputs
    if not len(sys.argv) == 3:
        # No input waypoint specified
        print('No waypoint specified in commandline')
        node = TurtleWaipoint()
    else:
        node = TurtleWaipoint(float(sys.argv[1]), float(sys.argv[2]))
    # Run forever at 10HZ
    while not rospy.is_shutdown():
        node.iterate()
        # use rospy Rate
    print('\nROS shutdown')
