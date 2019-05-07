#!/usr/bin/env python
import rospy
import math
import sys
from time import sleep
import tf

import tf2_ros
import tf2_msgs.msg
import tf_conversions

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
from std_msgs.msg import Int8
import sensor_msgs.point_cloud2 as pcl2
import geometry_msgs.msg

wall_left, wall_frontleft, wall_frontright, wall_right = 0, 0, 0, 0

transx, transy, transz = 0, 0, 0
rotx, roty, rotz, rotw = 0, 0, 0, 0

def publish_walls():
	#build the point cloud imagining we are at 0,0,0.
	cloud_points = []
	if(wall_left>0): cloud_points.append([0.5, 13.0/wall_left, 0.5])
	if(wall_frontleft>0): cloud_points.append([13.0/wall_frontleft, 0.5, 0.5])
	if(wall_frontright>0): cloud_points.append([13.0/wall_frontright, -0.5, 0.5])
	if(wall_right>0): cloud_points.append([0.5, -13.0/wall_right, 0.5])

	header = std_msgs.msg.Header()
	header.stamp = rospy.Time.now()
	header.frame_id = 'world'
	mypointcloud = pcl2.create_cloud_xyz32(header, cloud_points)

	#We need to build a transformation of our location.
	t = geometry_msgs.msg.TransformStamped()
	t.header.stamp = rospy.Time.now()
	t.header.frame_id = "world"
	t.child_frame_id = "zumo"
	t.transform.translation.x = transx
	t.transform.translation.y = transy
	t.transform.translation.z = transz
	t.transform.rotation.x = rotx
	t.transform.rotation.y = roty
	t.transform.rotation.z = rotz
	t.transform.rotation.w = rotw

	#convert the pointcloud to take into account our location.
	cloud_out = do_transform_cloud(mypointcloud, t)
	pcl_pub.publish(cloud_out)

def handle_zumo_left(wall_msg):
	global wall_left
	wall_left = wall_msg.data
	publish_walls()

def handle_zumo_frontleft(wall_msg):
	global wall_frontleft
	wall_frontleft = wall_msg.data
	publish_walls()

def handle_zumo_frontright(wall_msg):
	global wall_frontright
	wall_frontright = wall_msg.data
	publish_walls()

def handle_zumo_right(wall_msg):
	global wall_right
	wall_right = wall_msg.data
	publish_walls()

if __name__ == '__main__':
	pcl_pub = rospy.Publisher("/zumo/objectcloud", PointCloud2, queue_size=10)
	rospy.init_node('objectdetect_node')
	listener = tf.TransformListener()
	rospy.sleep(1.)
	rospy.Subscriber('/zumo/prox_left', Int8, handle_zumo_left)
	rospy.Subscriber('/zumo/prox_frontleft', Int8, handle_zumo_frontleft)
	rospy.Subscriber('/zumo/prox_frontright', Int8, handle_zumo_frontright)
	rospy.Subscriber('/zumo/prox_right', Int8, handle_zumo_right)
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/world', '/zumo', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		transx = trans[0]
		transy = trans[1]
		transz = trans[2]
		rotx = rot[0]
		roty = rot[1]
		rotz = rot[2]
		rotw = rot[3]
	rospy.spin()
