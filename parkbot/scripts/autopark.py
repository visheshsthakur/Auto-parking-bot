#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion 
from tf.transformations import quaternion_from_euler
import math

REFRESHRATE = 10
max_dist = 6.5 #This is the maximum distance the laser sensor will work for. Beyond this limit it gives out nan reading, meaning not a number
mid_range = 1.5 #This gives the range of the robot straight ahead
range_left = 1
range_right = 1
g_range_ahead = 1
roll = pitch = yaw = 0.0
check_left = False #To check if the robot has already looked left
check_right = False #To check if the robot has already looked right
forward = True #To check if the robot is facing forward. We will start with forward as True because our robot is facing forward at starting
backward = False #To check if the robot is facing backward

class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window", 1)
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', 
										  Image, self.image_callback)
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',
										   Twist, queue_size=1)
		self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
		self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
		self.twist = Twist()
		time_now = rospy.Time.now()
		self.rate = 10
		self.r = rospy.Rate(self.rate)
		self.angular_speed = 1.0

	def get_rotation (self, msg):
		global roll, pitch, yaw
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)

	def scan_callback(self, msg):
		global g_range_ahead
		global mid_range
		global range_left
		global range_right	
		range_left = msg.ranges[639]
		range_right = msg.ranges[0]
		mid_range =  msg.ranges[len(msg.ranges)/2]		
		self.g_range_ahead = min(msg.ranges)
		driving_forward = True
		rate = rospy.Rate(10)

	def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_yellow = numpy.array([ 20, 100,  100])	#This is to mask the color yellow. The values represent the upper and lower limit of yellow spectrum
		upper_yellow = numpy.array([ 50, 255,  250])
		mask_left = cv2.inRange(hsv, lower_yellow, upper_yellow)
		mask_right = cv2.inRange(hsv, lower_yellow, upper_yellow)
		
		lower_blue = numpy.array([90, 240, 200])	#This is to mask the color blue. The values represent the upper and lower limit of yellow spectrum
		upper_blue = numpy.array([131, 255, 255])
		mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

		h, w, d = image.shape
		search_top = 3*h/4
		search_bot = 3*h/4 + 20
		mask_left[0:search_top, 0:w] = 0 #Here the mask_left represents the entire masking of yellow and not just the left half of the robot vision
		mask_left[search_bot:h, 0:w] = 0 #The user may edit the mask_left and mask_right to mask different portions of the robot vision
		
		mask_right[0:search_top, 0:w] = 0
		mask_right[search_bot:h, 0:w] = 0
		mask_right[0:h, 0:4*w/5] = 0
		
		search_top_blue = h/2 + 60
		search_bot_blue = 3*h/4 + 20
		mask_blue[0:search_top_blue, 0:w] = 0
		mask_blue[search_bot:h, 0:w] = 0
		
		M_left = cv2.moments(mask_left)
		M_right = cv2.moments(mask_right)
		M_blue = cv2.moments(mask_blue)
		park = True
		global check_left
		global check_right
		global forward
		global backward		

		if forward:
			if check_left == False: #Check left
				if (M_left['m00'] > 0):
					cx_left = int(M_left['m10']/M_left['m00'])
					cy_left = int(M_left['m01']/M_left['m00'])
					cv2.circle(image, (cx_left, cy_left), 20, (0,0,255), -1)
					cv2.circle
					err_1 = cx_left
					if (err_1 == cx_left):
						self.twist.linear.x = 0
						self.twist.angular.z = 0.3
						self.cmd_vel_pub.publish(self.twist)
					else:
						self.twist.linear.x = 0
				
				elif (M_blue['m00'] > 0): #When sees Blue line stop
					cx_blue = int(M_blue['m10']/M_blue['m00'])
					cy_blue = int(M_blue['m01']/M_blue['m00'])
					cv2.circle(image, (cx_blue, cy_blue), 20, (0,255,0), -1)
					cv2.circle
					err_blue = cx_blue
					if (err_blue == cx_blue):
						rospy.Duration(5)
						self.twist.linear.x = 0
						self.twist.angular.z = 0
						self.cmd_vel_pub.publish(self.twist) 	
					else:
						self.twist.linear.x = 0
			
				elif 1.5 < yaw < 1.63: #This yaw represents the left angle of the world frame
					check_left = True
					return True
				else:
					
					self.twist.linear.x = 0.4
					self.twist.angular.z = 0
					self.cmd_vel_pub.publish(self.twist)
					if (0.5 < mid_range < 3):
						self.twist.linear.x = 0
						self.twist.angular.z = 0.3
						self.cmd_vel_pub.publish(self.twist)
				print("Check left %s" % check_left)
				print("Check right %s" % check_right)
			
			if (check_right == False and check_left == True): #If left is checked then check for right
					
					if (M_left['m00'] > 0):
						cx_left = int(M_left['m10']/M_left['m00'])
						cy_left = int(M_left['m01']/M_left['m00'])
						cv2.circle(image, (cx_left, cy_left), 20, (0,0,255), -1)
						cv2.circle
						err_1 = cx_left
						if (err_1 == cx_left):
							self.twist.linear.x = 0
							self.twist.angular.z = 0.3
							self.cmd_vel_pub.publish(self.twist)
						else:
							self.twist.linear.x = 0

					elif (M_blue['m00'] > 0): #When sees Blue line stop
						cx_blue = int(M_blue['m10']/M_blue['m00'])
						cy_blue = int(M_blue['m01']/M_blue['m00'])
						cv2.circle(image, (cx_blue, cy_blue), 20, (0,255,0), -1)
						cv2.circle
						err_blue = cx_blue
						if (err_blue == cx_blue):
							self.twist.linear.x = 0
							self.twist.angular.z = 0
							self.cmd_vel_pub.publish(self.twist) 	
						else:
							self.twist.linear.x = 0

					elif -1.6 < yaw < -1.20: #This yaw represents the right side of the world frame
						check_right = True
						return True

					else:
						self.twist.linear.x = 0.4
						self.twist.angular.z = 0
						self.cmd_vel_pub.publish(self.twist)
						if (0.5 < mid_range < 3):
							self.twist.linear.x = 0
							self.twist.angular.z = 0.3
							self.cmd_vel_pub.publish(self.twist)
					print("Check left %s" % check_left)
					print("Check right %s" % check_right)
					print("forward %s" % forward)

			if (check_left == True and check_right == True and forward == True): #If left side and right side are checked
					if (M_left['m00'] > 0):
						cx_left = int(M_left['m10']/M_left['m00'])
						cy_left = int(M_left['m01']/M_left['m00'])
						cv2.circle(image, (cx_left, cy_left), 20, (0,0,255), -1)
						cv2.circle
						err_1 = cx_left

						if (err_1 == cx_left):
							self.twist.linear.x = 0
							self.twist.angular.z = 0.3
							self.cmd_vel_pub.publish(self.twist)

						else:
							self.twist.linear.x = 0

					elif (M_blue['m00'] > 0): #When sees Blue line stop
						cx_blue = int(M_blue['m10']/M_blue['m00'])
						cy_blue = int(M_blue['m01']/M_blue['m00'])
						cv2.circle(image, (cx_blue, cy_blue), 20, (0,255,0), -1)
						cv2.circle
						err_blue = cx_blue
						if (err_blue == cx_blue):
							rospy.Duration(5)
							self.twist.linear.x = 0
							self.twist.angular.z = 0
							self.cmd_vel_pub.publish(self.twist) 	
						else:
							self.twist.linear.x = 0

					elif (-0.4 < yaw < 0.6): #This yaw represents that the robot is again facing forward in the world frame
						forward = False
						return False

					else:
						self.twist.linear.x = 0.4
						self.twist.angular.z = 0
						self.cmd_vel_pub.publish(self.twist)
						if (0.5 < mid_range < 3):
							self.twist.linear.x = 0
							self.twist.angular.z = 0.3
							self.cmd_vel_pub.publish(self.twist)
					print("Check left %s" % check_left)
					print("Check right %s" % check_right)
					print("forward %s" % forward)
		else: #After checking left and right and not parking itself, the robot will move forward for one metre and reset its values of left, right, and forward
			print("seeing forward")
			self.twist.linear.x = 0.5
			self.twist.angular.z = 0
			self.cmd_vel_pub.publish(self.twist)
			rospy.sleep(2)
			if check_left == True:
				check_left = False
				return False
			if check_right == True:
				check_right = False
				return False
			if forward == False:
				forward = True
				return True
		print yaw
		cv2.imshow("window", image)
		cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()

rospy.spin()
