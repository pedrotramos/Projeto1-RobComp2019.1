#! /usr/bin/env python
# -*- coding:utf-8 -*-


import numpy as np

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from math import pi 

dist = 0
minimo = 0
maximo = 0
dists = []

def scaneou(dado):
	#print(dado.range_min, dado.range_max)
  global minimo
  global maximo
  global dists

  minimo = dado.range_min
  maximo = dado.range_max
  dists = list(dado.ranges)

def dodge(velocidade_saida, ang):
  vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
  velocidade_saida.publish(vel)
  rospy.sleep(.2)
  
  if ang >= 0 and ang < 90:
    vel = Twist(Vector3(0,0,0), Vector3(0,0,-pi/4))

    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while current_angle >= -pi/2 and vel.linear.x == 0:
        rospy.sleep(.1)
        velocidade_saida.publish(vel)
        t1 = rospy.Time.now().to_sec()
        current_angle = vel.angular.z * (t1-t0)

  if ang < 360 and ang > 270:
    vel = Twist(Vector3(0,0,0), Vector3(0,0,pi/4))

    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while current_angle <= pi/2 and vel.linear.x == 0:
        rospy.sleep(.1)
        velocidade_saida.publish(vel)
        t1 = rospy.Time.now().to_sec()
        current_angle = vel.angular.z * (t1-t0)

  vel.angular.z = 0
  velocidade_saida.publish(vel)
  rospy.sleep(1.0)
  return
