#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

import traceback
import numpy as np


#common utility functions
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
import utils



STATE_COUNT_THRESHOLD = 3



class TLDetector(object):
	def __init__(self):
		rospy.init_node('tl_detector')

		self.waypoints = None
		self.camera_image = None
		self.lights = []

		sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
		self.sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

		'''
		/vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
		helps you acquire an accurate ground truth data source for the traffic light
		classifier by sending the current color state of all traffic lights in the
		simulator. When testing on the vehicle, the color state will not be available. You'll need to
		rely on the position of the light and the camera image to predict it.
		'''
		sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
		sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

		config_string = rospy.get_param("/traffic_light_config")
		self.config = yaml.load(config_string)

		self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

		self.bridge = CvBridge()
		self.light_classifier = TLClassifier()
		self.listener = tf.TransformListener()

		self.state = TrafficLight.UNKNOWN
		self.last_state = TrafficLight.UNKNOWN
		self.last_wp = -1
		self.state_count = 0

		self.light_map =[]
		self.pos = None #the car's position

		rospy.spin()


	def pose_cb(self, msg):
		self.pos = utils.SimplePose(msg.pose)

	def waypoints_cb(self, waypoints):
		self.waypoints = waypoints
		#self.waypoints = waypoints.waypoints
		rospy.loginfo('got wp!')
		self.sub2.unregister()

	def traffic_cb(self, msg):
		
		if self.pos is None:
			return 

		#if len(self.lights) !=0:
		#    return

		if self.waypoints is None:
			return


		#map each light to its nearest waypoint
		if len(self.lights) ==0:
			#rospy.logerr("----------------------------------SETTING LIGHT WAYPOINTS--------------------------------------")
			#rospy.logerr('>>>light msg len: %i %i'%(len(msg.lights), len(self.waypoints.waypoints)))

			for k,light in enumerate(msg.lights):
				#rospy.logerr('light msg len: %i %i'%(len(msg.lights), len(self.waypoints.waypoints)))
				pos = utils.SimplePose(light.pose.pose)
				i = utils.get_nearest_waypoint(pos,self.waypoints.waypoints,ignore_heading=True) #lights do not have a direction

				#rospy.logerr('light %i gets waypoint %i'%(k,i))
				self.light_map.append(i)

			#rospy.logerr('-~~~~~~~~~~~~~~~~~~~~~~~~~~~~~---> got lights! %i'%len(self.light_map))


		self.lights = msg.lights



	def image_cb(self, msg):
		"""Identifies red lights in the incoming camera image and publishes the index
			of the waypoint closest to the red light's stop line to /traffic_waypoint
		Args:
			msg (Image): image from car-mounted camera
		"""

		self.has_image = True
		self.camera_image = msg
		light_wp, state = self.process_traffic_lights()


		'''
		Publish upcoming red lights at camera frequency.
		Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
		of times till we start using it. Otherwise the previous stable state is
		used.
		'''
		if self.state != state:
			self.state_count = 0
			self.state = state
		elif self.state_count >= STATE_COUNT_THRESHOLD:
			self.last_state = self.state
			light_wp = light_wp if state == TrafficLight.RED else -1
			self.last_wp = light_wp
			self.upcoming_red_light_pub.publish(Int32(light_wp))
		else:
			self.upcoming_red_light_pub.publish(Int32(self.last_wp))
		self.state_count += 1

	def get_light_state(self, light):
		"""Determines the current color of the traffic light
		Args:
			light (TrafficLight): light to classify
		Returns:
			int: ID of traffic light color (specified in styx_msgs/TrafficLight)
		"""
		if(not self.has_image):
			self.prev_light_loc = None
			return False

		cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

		#Get classification
		return self.light_classifier.get_classification(cv_image)

	def process_traffic_lights(self):
		"""Finds closest visible traffic light, if one exists, and determines its
			location and color
		Returns:
			int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
			int: ID of traffic light color (specified in styx_msgs/TrafficLight)
		"""
		light = None
		light_wp = -1

		# List of positions that correspond to the line to stop in front of for a given intersection
		#stop_line_positions = self.config['stop_line_positions']
		#if(self.pos):
			#car_position = utils.get_nearest_waypoint(self.pos,self.waypoints.waypoints)

		#TODO find the closest visible traffic light (if one exists)
		if len(self.lights) >0:
			i = utils.get_nearest_waypoint(self.pos,self.lights, ignore_heading=True) #lights do not have a direction
			l = utils.SimplePose(self.lights[i].pose.pose)
			rospy.logerr('%i nearest light: %i, %s, car; %s'%(len(self.lights),i, l, self.pos))

			state = self.get_light_state(self.lights[i])

			return self.light_map[i], state


		if light:
			state = self.get_light_state(light)
			return light_wp, state
		#self.waypoints = None # because we unsuscribe from it
		return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
	try:
		TLDetector()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start traffic node.')
		rospy.logerr(traceback.format_exc())
