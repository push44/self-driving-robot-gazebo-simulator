#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

class str_to_twist():
	def __init__(self,roll, pitch, yaw, g_range_ahead, red_light_twist, green_light_twist, cmd_vel_pub,target,text):
		rospy.init_node('str_to_twist', anonymous=True) # Initiate the listner node.
		
		###### Making all the variables available inside the class listner ############
		
		self.text=text
		self.roll=roll
		self.pitch=pitch
		self.yaw=yaw
		self.g_range_ahead=g_range_ahead
		self.red_light_twist=red_light_twist
		self.green_light_twist=green_light_twist
		self.target = target
		
		######################## Subscribing to the necessary topics ############################################
		
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
		rospy.Subscriber('scan', LaserScan, self.scan_callback)
		rospy.Subscriber('img_to_str', String, self.string_callback)
		rospy.Subscriber ('/odom', Odometry, self.get_rotation)
		
		r = rospy.Rate(10)
		
		######### Go into the while-loop and keep running until Robot reaches to the finish block ##############
		
		while text!='Finish':
			
			self.red_light_twist = Twist()
			# Always assign 'green_light_twist' to Twist() so that code won't carry velocities from the last iteration. 
			self.green_light_twist = Twist()

			if self.g_range_ahead < 0.7:
				
				### Stop immediately #####
				
				self.cmd_vel_pub.publish(self.red_light_twist)
				
				####### Understand what img_str.py has to say and then based on that text add or subtract 90 Degrees. ###
				
				if self.text == "Right":
					self.target -= 90
				elif self.text == "Left":
					self.target += 90
				
				kp=0.5 # Proportional Controller
				key = True
				
				while key:
					#### Start rotating until key is false ####
					quat = quaternion_from_euler (self.roll, self.pitch, self.yaw) # Get current Roll, Pitch, Yaw values.
					target_rad = self.target*math.pi/180
					self.green_light_twist.angular.z = kp * (target_rad-self.yaw)
					
					self.cmd_vel_pub.publish(self.green_light_twist)
					
					stop = abs(target_rad-self.yaw)
					if stop <= 0.002:
						##### Now that rotation is accurate enough stop rotating ######
						key = False
					r.sleep()
			else :
				####### By default start moving forward #########
				self.green_light_twist.linear.x = 0.275
				self.green_light_twist.angular.z = 0.004
				self.cmd_vel_pub.publish(self.green_light_twist)
			r.sleep()
		print ("We are done!!!")

	def string_callback(self,msg):
		## Get instructions from img_str topic in String format ##
		self.text = msg.data

	def get_rotation (self,msg):
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)
		return self.roll, self.pitch, self.yaw
		
	def scan_callback(self,msg):
		## Get g_range_ahead from LaserScan topic ##
		self.g_range_ahead = msg.ranges[len(msg.ranges)/2]

if __name__ == '__main__':
	try:
		str_to_twist(roll=0, pitch=0, yaw=0, g_range_ahead=0, red_light_twist=0, green_light_twist=0, cmd_vel_pub=0,target=0,text='')
	except rospy.ROSInterruptException:
		pass
