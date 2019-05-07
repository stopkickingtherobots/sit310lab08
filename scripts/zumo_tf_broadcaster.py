#!/usr/bin/env python
import rospy

import tf_conversions
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry

#for calculating position
import math
from math import sin, cos, pi

x=0
y=0
th=0.0

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

def handle_zumo_pose(vel_msg):
	global x
	global y
	global th

	#for rotation, 5 degrees is 0.0872665 rads
	if(vel_msg.linear.y==1.0): th+= 0.0872665
	elif(vel_msg.linear.y==-1.0): th-=0.0872665
	elif(vel_msg.linear.x==1.0): #this is a movement
		vx = 0.1 #velocity of x, how much we move
		# compute odometry
		delta_x = vx * cos(th)
		delta_y = vx * sin(th)
		delta_th = 0 #no change in angle

		#add the changes to the values
		x += delta_x
		y += delta_y
		th += delta_th

	elif(vel_msg.linear.x==-1.0): #this is a movement
		vx = 0.1 #velocity of x, how much we move
		# compute odometry
		delta_x = vx * cos(th)
		delta_y = vx * sin(th)
		delta_th = 0 #no change in angle

		#add the changes to the values
		x -= delta_x
		y -= delta_y
		th -= delta_th

def publish_transform():
	br = tf2_ros.TransformBroadcaster()

	t = geometry_msgs.msg.TransformStamped()

	# y+=vel_msg.linear.y
	t.header.stamp = rospy.Time.now()
	t.header.frame_id = "odom"
	t.child_frame_id = "base_link"

	#set x,y,z
	t.transform.translation.x = x
	t.transform.translation.y = y
	t.transform.translation.z = 0.0

	#set rotation
	q = tf_conversions.transformations.quaternion_from_euler(0, 0, th)
	t.transform.rotation.x = q[0]
	t.transform.rotation.y = q[1]
	t.transform.rotation.z = q[2]
	t.transform.rotation.w = q[3]

	br.sendTransform(t)

def loop():
	# next, we'll publish the odometry message over ROS
	odom = Odometry()

	#set rotation
	q = tf_conversions.transformations.quaternion_from_euler(0, 0, th)

	odom.header.frame_id = "odom"

	# set the position
	odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*q))

	# set the velocity
	odom.child_frame_id = "base_link"
	odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
	current_time = rospy.Time.now()
	odom.header.stamp = current_time

	# publish the message
	odom_pub.publish(odom)
	publish_transform()

if __name__ == '__main__':
	rospy.init_node('zumo_tf_broadcaster')
	rospy.Subscriber('/zumo/cmd_vel', Twist, handle_zumo_pose)
	while not rospy.core.is_shutdown():
		loop()
		rospy.rostime.wallsleep(0.01)
		#rospy.spin()
