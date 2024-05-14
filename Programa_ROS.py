#!/usr/bin/env python
license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, TeleportRelative
from pynput import keyboard as kb
import termios, sys, os
import tty
import select
posicionS1 = 60
posicionS20
def motor_control():
	rospy.init_node('motor_control', anonymous=True) # iniciar el nodo
	pub = rospy. Publisher('motor_speed1', Int16, queue_size=10) #ropic que publicara la velocidad del motor 1
	pub2 =rospy.Publisher('motor_speed2', Int16, queue_size=10) #topic que publivara la velocidad del motoe 2
	pubservo1 = rospy. Publisher('servo1', Int16, queue_size=10) # topic para publivar posicion de servo
	pubservo2 = rospy. Publisher('servo2', Int16, queue_size=10) # topic para publicar posicion del servo 2
	rate = rospy.Rate (20) #20hz
	while not rospy.is_shutdown():
		key = getKey() # obtener tevla presionada
		if key == "r":
		break
	#COM
	control movimiento (key) # variables obtenidas dependiendo de la tecla presioanda
	motor_speed1, motor_speed2, posicionS1, posicions2 = control
	# asignar variables
	rospy.loginfo("spd1: %d spd2: %d PS1=%d PS2-%d ", motor_speed1, motor_speed2, posicionS1, posicions2)
	pub.publish(motor_speed1)
	pub2.publish(motor_speed2)
	pubservo1.publish(posicionS1)
	pubservo2.publish(posicionS2)
	rate.sleep()

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios. TCSADRAIN, settings)
	return key

def movimiento(tecla):
	print('Se ha pulsado la tecla ' + str(tecla))
	global posicionS1
	global posicionS2
	motor_speed1 = 0;
	motor_spedd2 = 0;
	if tecla == 'w'
		motorspeed1 = 250
		motorspeed2 = 250
	elif tecla == 's'
		motorspeed1 = -250
		motorspeed2 = -250
	elif tecla == 'a'
		motorspeed1 = -230
		motorspeed2 = 230
	elif tecla == 'd'
		motorspeed1 = 230
		motorspeed2 = -230
	elif tecla == 'q'
		motorspeed1 = 0
		motorspeed2 = 0
	elif tecla == 't'
		posicionS1 = posicionS1+1
	elif tecla == 'g'
		posicionS1 = posicionS1-1
	elif tecla == 'y'
		posicionS2 = posicionS2+1
	elif tecla == 'h'
		posicionS2 = posicionS2-1
	elif tecla == 'r'
		motorspeed1 = 0
		motorspeed2 = 0
	return motor_speed1, motor_speed2, posicionS1, posicionS2

def talker():
	while(1):
		key = getKey()
		if key=="r":
			break
		movimiento(key)

if __name__ == '__main__':
	settings = termios.tcgetattr(sys.stdin)
	twist = Twist()
	try:
		key = getKey()
		motor_control()
	except rospy.ROSInterruptException:
		pass


