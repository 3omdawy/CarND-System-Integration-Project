#!/usr/bin/env python

import rospy
# Messages and topics
from std_msgs.msg import Bool
from styx_msgs.msg import Lane, Waypoint
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped

# Math functions
import math

# Helpers for control
from twist_controller import Controller
from yaw_controller import YawController
from pid import PID

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

# Minimum speed passed to yaw controller
MINIMUM_SPEED = 0.4457 # 1mph

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

	# Constant parameters from Udacity repo
        self.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

	# We publish throttle, steering, and brake commands
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
	# Create controller objects
        self.controller = Controller()
        self.yaw_controller = YawController(wheel_base, steer_ratio, MINIMUM_SPEED, max_lat_accel,max_steer_angle)
	# Refer to PID project on my Github repo to know the rationale behind choice of parameters
	# Kp = 0.1, Ki = 0, Kd = 1
        self.pid = PID(0.1, 0.0, 1.0)

        # TODO: Subscribe to all the topics you need to
	# As in the architecture, subscribe to twist commands and to current velocity
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb,queue_size=1)
	# No need for dbw_enabled as we will run the code only on the simulator

	# Local variables to be set by received messaged
        self.current_velocity = None
        self.twist = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # Get basic initial commands
            throttle, brake, steering = self.controller.control()

            # If valid messages are received
            if self.twist is not None and self.current_velocity is not None:
                #rospy.logerr('target speed: %f'%(self.twist.linear.x))

                # Get the differnece between current speed and target speed
                difference = self.twist.linear.x - self.current_velocity.linear.x
		# Set taregt throttle by PID controller
                throttle = self.pid.step(difference, 0.05) # Sample time: 50Hz

		# If the difference is negative, we need to press the brake
                if throttle < 0:
                    brake = -self.vehicle_mass * throttle # - because throttle is -ve, speed is neglected because it is alredy very slow
                    throttle = 0
		# Get steering from yaw controller
                steering = self.yaw_controller.get_steering(self.twist.linear.x, self.twist.angular.z, self.current_velocity.linear.x)

                #rospy.logerr('%f,%f,%f'%(throttle, brake, steering))
		# Publish the commands
            	self.publish(throttle, brake, steering)
            rate.sleep()

    def current_velocity_cb(self,msg):
	#rospy.logerr('Received velocity message!')
        self.current_velocity = msg.twist

    def twist_cb(self,msg):
	#rospy.logerr('Received twist message!')
        self.twist = msg.twist

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
	#rospy.loginfo('Published throttle command: %f'%(throttle))
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
	#rospy.logwarn('Published steering command: %f'%(steer))
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
	#rospy.logerr('Published brake command: %f'%(brake))
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
        DBWNode()
