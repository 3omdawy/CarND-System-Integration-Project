from __future__ import print_function
import traceback

import rospy
import numpy as np

from styx_msgs.msg import TrafficLightArray, TrafficLight
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import tf

class SimplePose():
    '''Just work with x,y,z, theta'''
    def __init__(self, pose=PoseStamped()):
        self.pose = pose
        self.x = pose.position.x
        self.y = pose.position.y
        self.z = pose.position.z

        euler = tf.transformations.euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w])
        self.theta = euler[2]

    def __repr__(self):
        return '<%.2f,%.2f,%.2f> (%.2f)'%(self.x,self.y,self.z,self.theta)        


def distance(a,b):
    '''return distance between two simple poses'''
    return np.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

def get_nearest_waypoint(pos,waypoints, ignore_heading=False):
    '''returns the nearest waypoint index
       pos a simple pos
    '''
    min_dist = 1e12
    min_index = 0
    
    for i, w in enumerate(waypoints):

        wpose = SimplePose(w.pose.pose)

        dist = distance(pos,wpose)

        heading = np.arctan2( pos.y-wpose.y, pos.x - wpose.x)

        dtheta = np.abs(heading - pos.theta)

        if dist < min_dist and ( dtheta <= np.pi/4 or ignore_heading == True):
            min_dist = dist
            min_index = i

    return min_index

