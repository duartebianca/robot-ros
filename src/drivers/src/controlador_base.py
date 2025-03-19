#! /usr/bin/python3
import rospy
import Rpi.GPIO as GPIO
import time
import geometry_msgs.msg import Twist

#Set motor pins
MOTOR_LEFT_BACK = 11 # GPIO 17
MOTOR_LEFT_FOWARD = 13 # GPIO 27
MOTOR_RIGHT_BACK = 16 # GPIO 23
MOTOR_RIGHT_FOWARD = 18 # GPIO 24
ENCODER_LEFT = #
ENCODER_RIGHT = #

# Robot params
r = 0 #raio da roda
d = 0 #distancia das rodas

#Global variables
pulses_LEFT = 0
pulses_RIFGHT = 0
prev_time = time.time()

#Set up the board layout
GPIO.setmodde(GPIO.BCM)

#Set up pins as outputs
GPIO.setup(MOTOR_LEFT_BACK, GPIO.OUT)
GPIO.setup(MOTOR_LEFT_FOWARD, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_BACK, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_FOWARD, GPIO.OUT)

#Define pins as input and set to logic level 1 by default
GPIO.setup(ENCODER_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#Init pwm
pwm_LEFT_FOWARD.start(0)
pwm_LEFT_BACK.start(0)
pwm_RIGHT_FOWARD.start(0)
pwm_RIGHT_BACK.start(0)

def main() {

	#Create ControladorBase node
	rospy.init_node('ControladorBase')
	
	#Create a publisher in the topic /cmd_vel of a Twist type message
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #to em duvida sobre o topic e o queue_size
	cmd = Twist()
	rate = rospy.Rate(10) # 10 Hz
	
	count = 0
	while not rospy.is_shutdown():
	#acho que o loop muda aqui dentro, nao faz sentido pro nosso caso
	    if count % 2 == 0:
	        cmd.linear.x = 1
	        cmd.angular.z = 0
	    else:
	        cmd.linear.x = 0
	        cmd.angular.z = 1
	
	    pub.publish(cmd)
	    count += 1
	    rate.sleep()
	
	GPIO.cleanup()
}

if __name__="__main__":
	main()