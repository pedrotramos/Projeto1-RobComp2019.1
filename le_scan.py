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
l = []

def scaneou(dado):
	#print(dado.range_min, dado.range_max)
  global minimo
  global maximo
  global dist
  global l

  minimo = dado.range_min
  maximo = dado.range_max
  l = list(dado.ranges)
  dist = l[0]

def dodge(velocidade_saida):
  vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
  velocidade_saida.publish(vel)
  rospy.sleep(.2)
  vel = Twist(Vector3(0,0,0), Vector3(0,0,-pi/4))

  t0 = rospy.Time.now().to_sec()
  current_angle = 0

  while current_angle >= -pi/2 and vel.linear.x == 0:
      rospy.sleep(.1)
      velocidade_saida.publish(vel)
      t1 = rospy.Time.now().to_sec()
      current_angle = vel.angular.z * (t1-t0)

  vel.angular.z = 0
  velocidade_saida.publish(vel)
  rospy.sleep(1.0)
  return

valor = 0

if __name__=="__main__":

  rospy.init_node("le_scan")

  velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
  recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

  while not rospy.is_shutdown():
    vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

    if dist<maximo and dist > minimo:
      print(dist)
      vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))

      # if dist != valor:
      #   vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))

      # else:
      #   vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))



    velocidade_saida.publish(vel)
    rospy.sleep(0.1)
