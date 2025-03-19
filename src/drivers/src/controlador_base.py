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

# Constantes e configurações do sistema
# -------------------------------------------------
# Configuração da ponte H (controle dos motores)
H_BRIDGE = {
    'CONECTOR1': 5,  # gpio 5, porta 29
    'CONECTOR2': 6,  # gpio 6, porta 31
    'CONECTOR3': 16, # gpio 16, porta 36
    'CONECTOR4': 26, # gpio 26, porta 37
    'ENA': 13,       # gpio 13, porta 33
    'ENB': 12,       # gpio 12, porta 32
}

# Configuração dos encoders (sensores de velocidade)
ENCODER = {
    'LEFT': {
        'GREEN': 17,  # GPIO 17, porta 11
        'YELLOW': 27, # GPIO 27, porta 13
    },
    'RIGHT': {
        'GREEN': 23,  # GPIO 23, porta 16
        'YELLOW': 24, # GPIO 24, porta 18
    }
}

# Parâmetros do robô
ROBOT = {
    'WHEEL_RADIUS': 0.03,   # Raio da roda em metros
    'HALF_AXLE_LENGTH': 0.075, # Metade da distância entre as rodas em metros
    'ENCODER_PPR': 11,      # Pulsos por revolução do encoder
    'PWM_FREQUENCY': 100,   # Frequência PWM em Hz
    'CONTROL_RATE': 10      # Taxa de controle em Hz (10 = 100ms)
}

# Parâmetros do controlador PID
PID = {
    'KP': 0.5,  # Ganho proporcional
    'KI': 0.2,  # Ganho integral
    'KD': 0.1,  # Ganho derivativo
    'MAX_INTEGRAL': 100.0,  # Limite anti-windup para o termo integral
    'MIN_PWM': 0,           # Valor mínimo de PWM
    'MAX_PWM': 100          # Valor máximo de PWM
}

class EncoderCounter:
    """Classe para gerenciar contadores de encoder"""
    
    def __init__(self):
        """Inicializa contadores de pulsos"""
        self.pulses = {
            'LEFT_GREEN': 0,
            'LEFT_YELLOW': 0,
            'RIGHT_GREEN': 0,
            'RIGHT_YELLOW': 0
        }
        self.prev_time = time.time()
        
    def reset(self):
        """Reseta todos os contadores e atualiza o timestamp"""
        result = self.pulses.copy()
        for key in self.pulses:
            self.pulses[key] = 0
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        return result, dt
        
    def increment(self, encoder):
        """Incrementa o contador para um encoder específico"""
        if encoder in self.pulses:
            self.pulses[encoder] += 1

class PIDController:
    """Implementação de um controlador PID com anti-windup"""
    
    def __init__(self, kp, ki, kd, max_integral=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.prev_error = 0
        self.integral = 0
        
    def compute(self, error, dt):
        """Calcula a saída do controlador PID"""
        # Termo proporcional
        p_term = self.kp * error
        
        # Termo integral com anti-windup
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        i_term = self.ki * self.integral
        
        # Termo derivativo
        d_term = 0
        if dt > 0:  # Evita divisão por zero
            d_term = self.kd * (error - self.prev_error) / dt
        self.prev_error = error
        
        # Saída combinada
        output = p_term + i_term + d_term
        return output
        
    def reset(self):
        """Reseta o controlador"""
        self.prev_error = 0
        self.integral = 0

class MotorController:
    """Controla os motores através da ponte H"""
    
    def __init__(self):
        """Configura os pinos GPIO para controle dos motores"""
        # Configura pinos da ponte H
        GPIO.setup(H_BRIDGE['CONECTOR1'], GPIO.OUT)
        GPIO.setup(H_BRIDGE['CONECTOR2'], GPIO.OUT)
        GPIO.setup(H_BRIDGE['CONECTOR3'], GPIO.OUT)
        GPIO.setup(H_BRIDGE['CONECTOR4'], GPIO.OUT)
        GPIO.setup(H_BRIDGE['ENA'], GPIO.OUT)
        GPIO.setup(H_BRIDGE['ENB'], GPIO.OUT)
        
        # Configura PWM
        self.pwm_left = GPIO.PWM(H_BRIDGE['ENA'], ROBOT['PWM_FREQUENCY'])
        self.pwm_right = GPIO.PWM(H_BRIDGE['ENB'], ROBOT['PWM_FREQUENCY'])
        
        # Inicia PWM com 0%
        self.pwm_left.start(0)
        self.pwm_right.start(0)
        
    def set_direction(self, left_direction, right_direction):
        """Define a direção dos motores"""
        # Motor esquerdo
        if left_direction >= 0:  # Para frente
            GPIO.output(H_BRIDGE['CONECTOR1'], GPIO.HIGH)
            GPIO.output(H_BRIDGE['CONECTOR2'], GPIO.LOW)
        else:  # Para trás
            GPIO.output(H_BRIDGE['CONECTOR1'], GPIO.LOW)
            GPIO.output(H_BRIDGE['CONECTOR2'], GPIO.HIGH)
            
        # Motor direito
        if right_direction >= 0:  # Para frente
            GPIO.output(H_BRIDGE['CONECTOR3'], GPIO.HIGH)
            GPIO.output(H_BRIDGE['CONECTOR4'], GPIO.LOW)
        else:  # Para trás
            GPIO.output(H_BRIDGE['CONECTOR3'], GPIO.LOW)
            GPIO.output(H_BRIDGE['CONECTOR4'], GPIO.HIGH)
            
    def set_speed(self, left_pwm, right_pwm):
        """Define a velocidade dos motores via PWM"""
        self.pwm_left.ChangeDutyCycle(left_pwm)
        self.pwm_right.ChangeDutyCycle(right_pwm)
        
    def stop(self):
        """Para os motores"""
        self.pwm_left.ChangeDutyCycle(0)
        self.pwm_right.ChangeDutyCycle(0)
        
    def cleanup(self):
        """Libera recursos"""
        self.stop()
        self.pwm_left.stop()
        self.pwm_right.stop()

class ControladorBase:
    """Nó ROS para controle da base do robô"""
    
    def __init__(self):
        """Inicializa o nó ROS e configura o controlador"""
        # Inicializa o nó ROS
        rospy.init_node('ControladorBase')
        
        # Variáveis de estado
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.target_left_angular_vel = 0
        self.target_right_angular_vel = 0
        
        # Objetos para controle e monitoramento
        self.encoder_counter = EncoderCounter()
        self.motor_controller = MotorController()
        
        # Controladores PID
        self.pid_left = PIDController(
            PID['KP'], PID['KI'], PID['KD'], PID['MAX_INTEGRAL']
        )
        self.pid_right = PIDController(
            PID['KP'], PID['KI'], PID['KD'], PID['MAX_INTEGRAL']
        )
        
        # Configuração do subscriber e timer
        self.subscriber = rospy.Subscriber(
            '/cmd_vel', Twist, self.cmd_vel_callback, queue_size=1
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0/ROBOT['CONTROL_RATE']), 
            self.control_callback
        )
        
        rospy.loginfo("ControladorBase inicializado com sucesso")
        
    def cmd_vel_callback(self, msg):
        """Processa mensagens de velocidade recebidas"""
        # Armazena as velocidades recebidas
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        
        # Cálculo das velocidades angulares desejadas para cada roda
        # Usando as equações de cinemática inversa do robô diferencial:
        # v = (r/2)*uL + (r/2)*uR
        # w = -(r/2d)*uL + (r/2d)*uR
        #
        # Resolvendo para uL e uR:
        # uL = (v - w*d) / r
        # uR = (v + w*d) / r
        r = ROBOT['WHEEL_RADIUS']
        d = ROBOT['HALF_AXLE_LENGTH']
        
        self.target_left_angular_vel = (self.linear_velocity - self.angular_velocity * d) / r
        self.target_right_angular_vel = (self.linear_velocity + self.angular_velocity * d) / r
        
        rospy.loginfo(f"Velocidades alvo: Esq={self.target_left_angular_vel:.2f} rad/s, Dir={self.target_right_angular_vel:.2f} rad/s")
    
    def calculate_current_velocities(self):
        """Calcula as velocidades angulares atuais a partir dos encoders"""
        # Obtém os pulsos acumulados e o intervalo de tempo
        pulses, dt = self.encoder_counter.reset()
        
        # Prevenção contra divisão por zero
        if dt <= 0:
            rospy.logwarn("Intervalo de tempo inválido no cálculo de velocidade")
            return 0.0, 0.0
            
        # Calcula velocidades médias usando os dois canais do encoder
        ppr = ROBOT['ENCODER_PPR']
        
        # Calcula a média dos pulsos dos dois sinais (verde e amarelo)
        pulses_left = (pulses['LEFT_GREEN'] + pulses['LEFT_YELLOW']) / 2.0
        pulses_right = (pulses['RIGHT_GREEN'] + pulses['RIGHT_YELLOW']) / 2.0
        
        # Conversão para radianos por segundo
        left_angular_vel = (pulses_left * 2 * math.pi) / (ppr * dt)
        right_angular_vel = (pulses_right * 2 * math.pi) / (ppr * dt)
        
        return left_angular_vel, right_angular_vel
    
    def control_callback(self, event):
        """Executa o loop de controle para os motores"""
        try:
            # Obtém velocidades atuais dos encoders
            current_left, current_right = self.calculate_current_velocities()
            
            # Calcula erros
            error_left = self.target_left_angular_vel - current_left
            error_right = self.target_right_angular_vel - current_right
            
            # Calcula saída do controlador PID (intervalo de tempo = 1.0/ROBOT['CONTROL_RATE'])
            dt = 1.0 / ROBOT['CONTROL_RATE']
            output_left = self.pid_left.compute(error_left, dt)
            output_right = self.pid_right.compute(error_right, dt)
            
            # Limita a saída do PWM entre 0 e 100
            left_pwm = max(min(abs(output_left), PID['MAX_PWM']), PID['MIN_PWM'])
            right_pwm = max(min(abs(output_right), PID['MAX_PWM']), PID['MIN_PWM'])
            
            # Determina a direção com base na velocidade desejada
            left_direction = 1 if self.target_left_angular_vel >= 0 else -1
            right_direction = 1 if self.target_right_angular_vel >= 0 else -1
            
            # Aplica sinais PWM aos motores
            self.motor_controller.set_direction(left_direction, right_direction)
            self.motor_controller.set_speed(left_pwm, right_pwm)
            
            # Registra informações
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
    """Inicializa o sistema GPIO"""
    GPIO.setmode(GPIO.BCM)
    
    # Configura pinos da ponte H como saída
    GPIO.setup(H_BRIDGE['CONECTOR1'], GPIO.OUT)
    GPIO.setup(H_BRIDGE['CONECTOR2'], GPIO.OUT)
    GPIO.setup(H_BRIDGE['CONECTOR3'], GPIO.OUT)
    GPIO.setup(H_BRIDGE['CONECTOR4'], GPIO.OUT)
    GPIO.setup(H_BRIDGE['ENA'], GPIO.OUT)
    GPIO.setup(H_BRIDGE['ENB'], GPIO.OUT)
    
    # Configura pinos dos encoders como entrada com pull-up
    GPIO.setup(ENCODER['LEFT']['GREEN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER['LEFT']['YELLOW'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER['RIGHT']['GREEN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER['RIGHT']['YELLOW'], GPIO.IN, pull_up_down=GPIO.PUD_UP)

def setup_encoder_callbacks(encoder_counter):
    """Configura callbacks para os encoders"""
    
    def create_callback(encoder_name):
        def callback(channel):
            encoder_counter.increment(encoder_name)
        return callback
    
    # Configura detecção de eventos para os encoders
    GPIO.add_event_detect(
        ENCODER['LEFT']['GREEN'], GPIO.RISING, 
        callback=create_callback('LEFT_GREEN')
    )
    GPIO.add_event_detect(
        ENCODER['LEFT']['YELLOW'], GPIO.RISING, 
        callback=create_callback('LEFT_YELLOW')
    )
    GPIO.add_event_detect(
        ENCODER['RIGHT']['GREEN'], GPIO.RISING, 
        callback=create_callback('RIGHT_GREEN')
    )
    GPIO.add_event_detect(
        ENCODER['RIGHT']['YELLOW'], GPIO.RISING, 
        callback=create_callback('RIGHT_YELLOW')
    )

def main():
    """Função principal"""
    try:
        # Inicializa GPIO
        init_gpio()
        
        # Cria o contador de encoder
        encoder_counter = EncoderCounter()
        
        # Configura callbacks para os encoders
        setup_encoder_callbacks(encoder_counter)
        
        # Cria e inicia o controlador
        controller = ControladorBase()
        
        # Mantém o nó em execução
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Nó interrompido")
    except Exception as e:
        rospy.logerr(f"Erro: {str(e)}")
    finally:
        # Para os motores e limpa GPIO
        try:
            controller.motor_controller.cleanup()
        except:
            pass
        GPIO.cleanup()
        rospy.loginfo("Recursos liberados e GPIO limpo")

if __name__ == "__main__":
    main()