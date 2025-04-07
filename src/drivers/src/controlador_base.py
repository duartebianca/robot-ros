#!/usr/bin/python3
"""
ControladorBase - Controlador para robô diferencial com ROS.
Implementa um controlador PID para controle de velocidade baseado em encoders.
"""

import rospy
import RPi.GPIO as GPIO
import time
import math
from geometry_msgs.msg import Twist

# -------------------------------------------------
# Configuração da ponte H (controle dos motores)
H_BRIDGE = {
    'CONECTOR1': 16,  # gpio 16, porta 36
    'CONECTOR2': 26,  # gpio 26, porta 37
    'CONECTOR3': 5,   # gpio 5, porta 29
    'CONECTOR4': 6,   # gpio 6, porta 31
    'ENA': 13,        # gpio 13, porta 33
    'ENB': 12,        # gpio 12, porta 32
}

# Configuração dos encoders (sensores de velocidade)
ENCODER = {
    'LEFT': {
        'GREEN': 17,   # GPIO 17, porta 11
        'YELLOW': 27,  # GPIO 27, porta 13
    },
    'RIGHT': {
        'GREEN': 23,   # GPIO 23, porta 16
        'YELLOW': 24,  # GPIO 24, porta 18
    }
}

# Parâmetros do robô
ROBOT = {
    'WHEEL_RADIUS': 0.03,         # Raio da roda em metros
    'HALF_AXLE_LENGTH': 0.0335,     # Metade da distância entre as rodas em metros
    'ENCODER_PPR': 506,  # 11 * 46 pulsos por revolução do encoder
    'PWM_FREQUENCY': 1000,         # Frequência PWM em Hz
    'CONTROL_RATE': 20            # Taxa de controle em Hz (10 = 100ms)
}

# Parâmetros do controlador PID
PID = {
    'KP': 0.5,          # Ganho proporcional
    'KI': 0.2,          # Ganho integral
    'KD': 0.1,          # Ganho derivativo
    'MAX_INTEGRAL': 100.0,  # Limite anti-windup para o termo integral
    'MIN_PWM': 0,           # Valor mínimo de PWM
    'MAX_PWM': 100          # Valor máximo de PWM
}


class EncoderCounter:
    """Gerencia a contagem de pulsos dos encoders."""
    def __init__(self):
        self.pulses = {
            'LEFT_GREEN': 0,
            'LEFT_YELLOW': 0,
            'RIGHT_GREEN': 0,
            'RIGHT_YELLOW': 0
        }
        self.prev_time = time.time()

    def reset(self):
        """Retorna a contagem acumulada e zera os pulsos, atualizando o intervalo dt."""
        result = self.pulses.copy()
        for key in self.pulses:
            self.pulses[key] = 0
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        return result, dt

    def increment(self, encoder_name):
        """Incrementa a contagem para o canal de um encoder."""
        if encoder_name in self.pulses:
            self.pulses[encoder_name] += 1


class PIDController:
    """Controlador PID com anti-windup."""
    def __init__(self, kp, ki, kd, max_integral=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        """Calcula a saída PID para o erro dado."""
        p_term = self.kp * error

        self.integral += error * dt
        # Anti-windup: limita o termo integral
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        i_term = self.ki * self.integral

        d_term = 0.0
        if dt > 0:
            d_term = self.kd * (error - self.prev_error) / dt
        self.prev_error = error

        return p_term + i_term + d_term

    def reset(self):
        """Reseta o controlador."""
        self.prev_error = 0.0
        self.integral = 0.0


class MotorController:
    """Controla os motores por meio da ponte H."""
    def __init__(self):
        # Configura os pinos da ponte H como saída
        GPIO.setup(H_BRIDGE['CONECTOR1'], GPIO.OUT)
        GPIO.setup(H_BRIDGE['CONECTOR2'], GPIO.OUT)
        GPIO.setup(H_BRIDGE['CONECTOR3'], GPIO.OUT)
        GPIO.setup(H_BRIDGE['CONECTOR4'], GPIO.OUT)
        GPIO.setup(H_BRIDGE['ENA'], GPIO.OUT)
        GPIO.setup(H_BRIDGE['ENB'], GPIO.OUT)

        # Inicializa o PWM para os canais dos motores
        self.pwm_left = GPIO.PWM(H_BRIDGE['ENA'], ROBOT['PWM_FREQUENCY'])
        self.pwm_right = GPIO.PWM(H_BRIDGE['ENB'], ROBOT['PWM_FREQUENCY'])
        self.pwm_left.start(0)
        self.pwm_right.start(0)

    def set_direction(self, left_direction, right_direction):
        """Define a direção dos motores."""
        # Motor esquerdo
        if left_direction >= 0:
            GPIO.output(H_BRIDGE['CONECTOR1'], GPIO.HIGH)
            GPIO.output(H_BRIDGE['CONECTOR2'], GPIO.LOW)
        else:
            GPIO.output(H_BRIDGE['CONECTOR1'], GPIO.LOW)
            GPIO.output(H_BRIDGE['CONECTOR2'], GPIO.HIGH)
        # Motor direito
        if right_direction >= 0:
            GPIO.output(H_BRIDGE['CONECTOR3'], GPIO.HIGH)
            GPIO.output(H_BRIDGE['CONECTOR4'], GPIO.LOW)
        else:
            GPIO.output(H_BRIDGE['CONECTOR3'], GPIO.LOW)
            GPIO.output(H_BRIDGE['CONECTOR4'], GPIO.HIGH)

    def set_speed(self, left_pwm, right_pwm):
        """Altera o duty cycle do PWM para definir a velocidade dos motores."""
        self.pwm_left.ChangeDutyCycle(left_pwm)
        self.pwm_right.ChangeDutyCycle(right_pwm)

    def stop(self):
        """Para os motores."""
        self.pwm_left.ChangeDutyCycle(0)
        self.pwm_right.ChangeDutyCycle(0)

    def cleanup(self):
        """Libera recursos do PWM."""
        self.stop()
        self.pwm_left.stop()
        self.pwm_right.stop()


class ControladorBase:
    """Nó ROS para controle da base do robô."""
    def __init__(self):
        # GPIO.cleanup()
        rospy.init_node('ControladorBase')
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.target_left_angular_vel = 0.0
        self.target_right_angular_vel = 0.0

        # Instancia dos componentes de controle
        self.encoder_counter = EncoderCounter()
        self.motor_controller = MotorController()
        self.pid_left = PIDController(PID['KP'], PID['KI'], PID['KD'], PID['MAX_INTEGRAL'])
        self.pid_right = PIDController(PID['KP'], PID['KI'], PID['KD'], PID['MAX_INTEGRAL'])

        # Inscrição no tópico de comando de velocidade
        self.subscriber = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=1)
        # Timer para executar o loop de controle
        self.timer = rospy.Timer(rospy.Duration(1.0 / ROBOT['CONTROL_RATE']), self.control_callback)

        rospy.loginfo("ControladorBase inicializado com sucesso.")

    def cmd_vel_callback(self, msg):
        """Recebe mensagens Twist e calcula as velocidades angulares desejadas para cada roda."""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

        # Cálculo usando cinemática inversa para robô diferencial:
        # uL = (v - w*d) / r
        # uR = (v + w*d) / r
        r = ROBOT['WHEEL_RADIUS']
        d = ROBOT['HALF_AXLE_LENGTH']
        self.target_left_angular_vel = (self.linear_velocity - self.angular_velocity * d) / r
        self.target_right_angular_vel = (self.linear_velocity + self.angular_velocity * d) / r

        rospy.loginfo(f"Velocidades alvo: Esq={self.target_left_angular_vel:.2f} rad/s, Dir={self.target_right_angular_vel:.2f} rad/s")

    def calculate_current_velocities(self):
        """Calcula as velocidades angulares atuais com base na contagem dos encoders."""
        pulses, dt = self.encoder_counter.reset()
        if dt <= 0:
            rospy.logwarn("Intervalo de tempo inválido para cálculo de velocidade")
            return 0.0, 0.0

        ppr = ROBOT['ENCODER_PPR']
        # Média dos pulsos dos dois canais de cada encoder
        pulses_left = (pulses['LEFT_GREEN'] + pulses['LEFT_YELLOW']) / 2.0
        pulses_right = (pulses['RIGHT_GREEN'] + pulses['RIGHT_YELLOW']) / 2.0

        # Converte pulsos para velocidade angular (rad/s)
        left_angular_vel = (pulses_left * 2 * math.pi) / (ppr * dt)
        right_angular_vel = (pulses_right * 2 * math.pi) / (ppr * dt)
        return left_angular_vel, right_angular_vel

    def control_callback(self, event):
        """Loop de controle: calcula erro, aplica PID e envia comandos PWM."""
        try:
            current_left, current_right = self.calculate_current_velocities()
            error_left = self.target_left_angular_vel - current_left
            error_right = self.target_right_angular_vel - current_right

            dt = 1.0 / ROBOT['CONTROL_RATE']
            output_left = self.pid_left.compute(error_left, dt)
            output_right = self.pid_right.compute(error_right, dt)

            # Garante que o PWM fique no intervalo definido
            left_pwm = max(min(abs(output_left), PID['MAX_PWM']), PID['MIN_PWM'])
            right_pwm = max(min(abs(output_right), PID['MAX_PWM']), PID['MIN_PWM'])

            left_direction = 1 if self.target_left_angular_vel >= 0 else -1
            right_direction = 1 if self.target_right_angular_vel >= 0 else -1

            self.motor_controller.set_direction(left_direction, right_direction)
            self.motor_controller.set_speed(left_pwm, right_pwm)

            rospy.loginfo(f"Pulsos: LG={self.encoder_counter.pulses['LEFT_GREEN']}, "
                          f"LY={self.encoder_counter.pulses['LEFT_YELLOW']}, "
                          f"RG={self.encoder_counter.pulses['RIGHT_GREEN']}, "
                          f"RY={self.encoder_counter.pulses['RIGHT_YELLOW']}")
            rospy.loginfo(f"Velocidades atuais: Esq={current_left:.2f} rad/s, Dir={current_right:.2f} rad/s")
            rospy.loginfo(f"Erros: Esq={error_left:.2f}, Dir={error_right:.2f}")
            rospy.loginfo(f"PWM: Esq={left_pwm:.1f}%, Dir={right_pwm:.1f}%")

        except Exception as e:
            rospy.logerr(f"Erro no callback de controle: {str(e)}")


def init_gpio():
    """Inicializa os pinos GPIO necessários."""
    GPIO.setmode(GPIO.BCM)
    # Ponte H
    GPIO.setup(H_BRIDGE['CONECTOR1'], GPIO.OUT)
    GPIO.setup(H_BRIDGE['CONECTOR2'], GPIO.OUT)
    GPIO.setup(H_BRIDGE['CONECTOR3'], GPIO.OUT)
    GPIO.setup(H_BRIDGE['CONECTOR4'], GPIO.OUT)
    GPIO.setup(H_BRIDGE['ENA'], GPIO.OUT)
    GPIO.setup(H_BRIDGE['ENB'], GPIO.OUT)
    # Encoders com pull-up
    GPIO.setup(ENCODER['LEFT']['GREEN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER['LEFT']['YELLOW'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER['RIGHT']['GREEN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER['RIGHT']['YELLOW'], GPIO.IN, pull_up_down=GPIO.PUD_UP)


def setup_encoder_callbacks(encoder_counter):
    """Configura os callbacks para contar os pulsos dos encoders."""
    def create_callback(encoder_name):
        def callback(channel):
            encoder_counter.increment(encoder_name)
        return callback
    
    # Agora, configure os callbacks
    GPIO.add_event_detect(ENCODER['LEFT']['GREEN'], GPIO.RISING, callback=create_callback('LEFT_GREEN'))
    GPIO.add_event_detect(ENCODER['LEFT']['YELLOW'], GPIO.RISING, callback=create_callback('LEFT_YELLOW'))
    GPIO.add_event_detect(ENCODER['RIGHT']['GREEN'], GPIO.RISING, callback=create_callback('RIGHT_GREEN'))
    GPIO.add_event_detect(ENCODER['RIGHT']['YELLOW'], GPIO.RISING, callback=create_callback('RIGHT_YELLOW'))



def main():
    try:
        
        init_gpio()
        # time.sleep(10)
        encoder_counter = EncoderCounter()
        setup_encoder_callbacks(encoder_counter)

        # Cria a instância do ControladorBase
        controlador = ControladorBase()
        # Vincula o encoder_counter do ControladorBase ao que foi configurado
        controlador.encoder_counter = encoder_counter

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Nó interrompido")
    except Exception as e:
        rospy.logerr(f"Erro: {str(e)}")
    finally:
        try:
            controlador.motor_controller.cleanup()
        except Exception:
            pass
        GPIO.cleanup()
        rospy.loginfo("Recursos liberados e GPIO limpo")


if __name__ == "__main__":
    main()
