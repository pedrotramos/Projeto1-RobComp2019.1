#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import UInt8
from math import pi

bump = None
bump_happened = False

def bumper(data):
	global bump
	global bump_happened
	bump = data.data
	bump_happened = True

def forward():
    vel = Twist(Vector3(.25,0,0), Vector3(0,0,0))
    pub.publish(vel)
    rospy.sleep(.05)


def rotate_left():
  v0 = Twist(Vector3(0,0,0), Vector3(0,0,0))
  pub.publish(v0)
  rospy.sleep(.2)
  vel = Twist(Vector3(0,0,0), Vector3(0,0,pi/4))

  t0 = rospy.Time.now().to_sec()
  current_angle = 0

  while current_angle <= pi/2 and vel.linear.x == 0:
      rospy.sleep(.1)
      pub.publish(vel)
      t1 = rospy.Time.now().to_sec()
      current_angle = vel.angular.z * (t1-t0)

  vel.angular.z = 0
  pub.publish(vel)
  rospy.sleep(1.0)
  return

def back():
    v0 = Twist(Vector3(0,0,0), Vector3(0,0,0))
    pub.publish(v0)
    rospy.sleep(.2)
    vel = Twist(Vector3(-.25,0,0), Vector3(0,0,0))

    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    while current_distance > -.2 and vel.angular.z == 0:
        rospy.sleep(.1)
        pub.publish(vel)
        t1 = rospy.Time.now().to_sec()
        current_distance = vel.linear.x * (t1-t0)

    vel.linear.x = 0
    pub.publish(vel)
    rospy.sleep(1.0)
    return

def rotate_right():
  v0 = Twist(Vector3(0,0,0), Vector3(0,0,0))
  pub.publish(v0)
  rospy.sleep(.2)
  vel = Twist(Vector3(0,0,0), Vector3(0,0,-pi/4))

  t0 = rospy.Time.now().to_sec()
  current_angle = 0

  while current_angle >= -pi/2 and vel.linear.x == 0:
      rospy.sleep(.1)
      pub.publish(vel)
      t1 = rospy.Time.now().to_sec()
      current_angle = vel.angular.z * (t1-t0)

  vel.angular.z = 0
  pub.publish(vel)
  rospy.sleep(1.0)
  return

if __name__=="__main__":

	rospy.init_node("bump")
	reconhece_bump = rospy.Subscriber("/bumper", UInt8, bumper)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)

	while not rospy.is_shutdown():
		while True:
			forward()
			if bump_happened:
				if bump == 1:
					bump_happened = False
					back()
					rotate_right()
				if bump == 2:
					bump_happened = False
					back()
					rotate_left()
				if bump == 3:
					bump_happened = False
					forward()
					rotate_right()
				if bump == 4:
					bump_happened = False
					forward()
					rotate_left()