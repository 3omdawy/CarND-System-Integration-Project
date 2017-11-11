#!/usr/bin/env python
''''from __future__ import print_function
import traceback'''

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import numpy as np
'''import tf

#common utility functions
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
import utils'''


'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

# Similar to (distance) of Path planning project, just convert from C++ to Python
def distance(x1,y1,x2,y2):
    return np.sqrt((x2-x1)**2 + (y2-y1)**2)

# Similar to (ClosestWaypoint) of Path planning project, just convert from C++ to Python
def ClosestWaypoint(x,y,waypoints):
    closestLen = 100000 # large number
    closestWaypoint = 0
    
    for i in range(len(waypoints)):

        map_x = waypoints[i].pose.pose.position.x
        map_y = waypoints[i].pose.pose.position.y
        dist = distance(x,y,map_x,map_y)

        if dist < closestLen:
            closestLen = dist
            closestWaypoint = i
            #rospy.logerr('Waypoint found!')

    return closestWaypoint


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Make necessary subscriptions at the beginning
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_wp_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

	# Final waypoints will be published
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

	# Define local variables
        self.waypoints = None	# Array of needed waypoints (to be received by /base_waypoints)
        self.pos = None		# Current pose (to be received by /current_pose)
        self.light_i = None	# Current traffic light status (to be received by /traffic_waypoint)
	#self.willPrint = 1

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # As recommended by Udacity, refresh rate is 50 Hz

        while not rospy.is_shutdown():

            if self.pos is None or self.waypoints is None:
		# Initialization not yet done
                #rospy.logerr('Simulator is not started or communication could not be established')
                rate.sleep()
                continue


            # Get the nearest waypoint based on current position
            nearest_WP = ClosestWaypoint(self.pos.position.x,self.pos.position.y,self.waypoints)
            # Check weather there is a light traffic or not
            if self.light_i is None:
                self.light_i = -1
            #rospy.logerr('current position: %s , next waypoint: %i ,next red traffic: %i'%(self.pos,nearest_WP, self.light_i))
            
            # Identify velocity for next waypoints
            new_waypoints = Lane()
            # From current waypoint with a fixed size
            cutoff = nearest_WP+LOOKAHEAD_WPS
            new_waypoints.waypoints = self.waypoints[nearest_WP:cutoff] # no need to handle overflow, only 1 lap is needed


            speed = 10*.4457 # 10 mph, we want to driver as slowly as possible to avoid accidents and accelerations
            #if light is red and close ahead, make target speed 0
            #200: experimental distance of the white line before the stop
            if self.light_i - nearest_WP < 200 and self.light_i >= nearest_WP:
                rospy.logerr('Red light, target speed set to 0')
                speed = 0

            #set the speed for these waypoints
            for k,w in enumerate(new_waypoints.waypoints):
		w.twist.twist.linear.x = speed

            #Publish these waypoints
            '''if self.willPrint == 1:
            	rospy.logerr('%s'%(new_waypoints.waypoints))
            	self.willPrint = 0'''
            self.final_waypoints_pub.publish(new_waypoints)
            #rospy.logerr('Points published!')
            rate.sleep()

    def pose_cb(self, msg):
	# Update position with current position
	self.pos = msg.pose
        #self.pos.x = msg.pose.position.x
        #self.pos.y = msg.pose.position.y

    def waypoints_cb(self, waypoints):
	# Set the local waypoints array with the passed message
	rospy.loginfo('Waypoints received with length %i!' %(len(waypoints.waypoints)))
        self.waypoints = waypoints.waypoints

	# Unsubscribe as the message is needed only once at initialization
        self.base_wp_sub.unregister()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.light_i = msg.data

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
