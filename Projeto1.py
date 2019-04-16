#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cormodule
from std_msgs.msg import UInt8
import P1_pt1 as p1
import le_scan as ls
import visao_module


bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 0.5E9 # meio segundo. Em nanossegundos
resultados_mnet = []
need_dodge = False
area = 0.0 # Variavel com a area do maior contorno
a = 100000

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
	global cv_image
	global media
	global centro
	global area
	global resultados_mnet

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro, area =  cormodule.identifica_cor(cv_image)
		centro_mnet, imagem_mnet, resultados_mnet =  visao_module.processa(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", imagem_mnet)
	except CvBridgeError as e:
		print('ex', e)
	
if __name__=="__main__":
	rospy.init_node("cor")

	topico_imagem = "/kamera"
	
	# Para renomear a *webcam*
	# 
	# 	rosrun topic_tools relay  /cv_camera/image_raw/compressed /kamera
	# 
	# Para renomear a câmera simulada do Gazebo
	# 
	# 	rosrun topic_tools relay  /camera/rgb/image_raw/compressed /kamera
	# 
	# Para renomear a câmera da Raspberry
	# 
	# 	rosrun topic_tools relay /raspicam_node/image/compressed /kamera
	# 

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	recebe_bump = rospy.Subscriber("/bumper", UInt8, p1.bumper)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, ls.scaneou)
	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	try:
		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			for i in range(len(ls.dists)):
				need_dodge = False
				if ls.dists[i] < ls.maximo and ls.dists[i] > ls.minimo and ls.dists[i] < 0.15:
					need_dodge = True
					angulo = i
					break
			if p1.bump_happened:
				p1.react(velocidade_saida)
				p1.bump_happened = False
			elif need_dodge:
				ls.dodge(velocidade_saida, angulo)
				p1.forward(velocidade_saida)
			elif len(resultados_mnet) != 0:
				vel = Twist(Vector3(-0.3,0,0), Vector3(0,0,0))
			else:
				if area > 2000:
					if area > a:
						if  media[0] < centro[0]:
							vel = Twist(Vector3(-0.1,0,0), Vector3(0,0,-0.2))
						elif media[0] > centro[0]:
							vel = Twist(Vector3(-0.1,0,0), Vector3(0,0,0.2))
					else:
						if media[0] < centro[0] and area < a:
							vel = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.2))
						elif media[0] > centro[0] and area < a:
							vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0.2))
				else:
					vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
			velocidade_saida.publish(vel)
			rospy.sleep(0.01)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
