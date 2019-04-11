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

def forward(velocidade_saida):
    vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
    velocidade_saida.publish(vel)
    rospy.sleep(.2)
    vel = Twist(Vector3(.25,0,0), Vector3(0,0,0))

    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    while current_distance < .2 and vel.angular.z == 0:
        rospy.sleep(.1)
        velocidade_saida.publish(vel)
        t1 = rospy.Time.now().to_sec()
        current_distance = vel.linear.x * (t1-t0)

    vel.linear.x = 0
    velocidade_saida.publish(vel)
    rospy.sleep(1.0)
    return


def rotate_left(velocidade_saida):
  vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
  velocidade_saida.publish(vel)
  rospy.sleep(.2)
  vel = Twist(Vector3(0,0,0), Vector3(0,0,pi/4))

  t0 = rospy.Time.now().to_sec()
  current_angle = 0

  while current_angle <= pi/3 and vel.linear.x == 0:
      rospy.sleep(.1)
      velocidade_saida.publish(vel)
      t1 = rospy.Time.now().to_sec()
      current_angle = vel.angular.z * (t1-t0)

  vel.angular.z = 0
  velocidade_saida.publish(vel)
  rospy.sleep(1.0)
  return

def back(velocidade_saida):
    vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
    velocidade_saida.publish(vel)
    rospy.sleep(.2)
    vel = Twist(Vector3(-.25,0,0), Vector3(0,0,0))

    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    while current_distance > -.2 and vel.angular.z == 0:
        rospy.sleep(.1)
        velocidade_saida.publish(vel)
        t1 = rospy.Time.now().to_sec()
        current_distance = vel.linear.x * (t1-t0)

    vel.linear.x = 0
    velocidade_saida.publish(vel)
    rospy.sleep(1.0)
    return

def rotate_right(velocidade_saida):
  vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
  velocidade_saida.publish(vel)
  rospy.sleep(.2)
  vel = Twist(Vector3(0,0,0), Vector3(0,0,-pi/4))

  t0 = rospy.Time.now().to_sec()
  current_angle = 0

  while current_angle >= -pi/3 and vel.linear.x == 0:
      rospy.sleep(.1)
      velocidade_saida.publish(vel)
      t1 = rospy.Time.now().to_sec()
      current_angle = vel.angular.z * (t1-t0)

  vel.angular.z = 0
  velocidade_saida.publish(vel)
  rospy.sleep(1.0)
  return

def react(velocidade_saida):
  if bump == 1:
    bump_happened = False
    back(velocidade_saida)
    rotate_right(velocidade_saida)
    forward(velocidade_saida)
  if bump == 2:
    bump_happened = False
    back(velocidade_saida)
    rotate_left(velocidade_saida)
    forward(velocidade_saida)
  if bump == 3:
    bump_happened = False
    forward(velocidade_saida)
    rotate_right(velocidade_saida)
  if bump == 4:
    bump_happened = False
    forward(velocidade_saida)
    rotate_left(velocidade_saida)
