#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from std_srvs.srv import SetBool, SetBoolResponse
import signal
import sys

# Configuração do pino do servomotor
SERVO_PIN = 25
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Configuração do PWM no pino do servo (50Hz)
pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(7.5)  # Posição inicial (~90 graus)

def signal_handler(sig, frame):
    """
    Manipulador de sinal para Ctrl+C (SIGINT)
    """
    rospy.loginfo("Ctrl+C pressionado. Encerrando...")
    pwm.stop()
    GPIO.cleanup()
    sys.exit(0)

def set_servo_position(request):
    """
    Controla a posição do servomotor.
    - request.data = True → Move para 0º
    - request.data = False → Move para 180º
    """
    try:
        duty_cycle = 5 if request.data else 10  # 5 → 0º, 10 → 180º
        pwm.ChangeDutyCycle(duty_cycle)
        rospy.sleep(5)  # Tempo para o servo se mover
        return SetBoolResponse(success=True, message="Posição ajustada")
    except Exception as e:
        return SetBoolResponse(success=False, message=str(e))

def servo_server():
    rospy.init_node("servo_server")
    # Registra o manipulador de sinal para Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    service = rospy.Service("set_servo", SetBool, set_servo_position)
    rospy.loginfo("Servidor do servomotor pronto.")
    rospy.spin()

if __name__ == "__main__":
    try:
        servo_server()
    except rospy.ROSInterruptException:
        pwm.stop()
        GPIO.cleanup()
