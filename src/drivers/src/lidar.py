import os
import ydlidar
import time
import rospy
from sensor_msgs.msg import LaserScan

class LidarPublisher:
    
    def __init__(self):
        # Inicializa o nó ROS
        rospy.init_node("lidar_publisher")
        
        # Cria o publisher do tipo LaserScan
        self.publisher = rospy.Publisher("/scan_raw", LaserScan, queue_size=1)
        
        # Inicializa o YDLidar
        self.laser = ydlidar.CYdLidar()
        self.initialize_lidar()

    def initialize_lidar(self):
        """Configura o Lidar YDlidar G2"""
        ydlidar.os_init()
        ports = ydlidar.lidarPortList()
        port = "/dev/ydlidar"
        for key, value in ports.items():
            port = value
            rospy.loginfo(f"Usando porta: {port}")
        
        # Configurações do Lidar
        self.laser.setlidarropt(ydlidar.LidarPropSerialPort, port)
        self.laser.setlidarropt(ydlidar.LidarPropSerialBaudrate, 115200)
        self.laser.setlidarropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.laser.setlidarropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.laser.setlidarropt(ydlidar.LidarPropScanFrequency, 10.0)  # Frequência de escaneamento
        self.laser.setlidarropt(ydlidar.LidarPropSampleRate, 3)  
        self.laser.setlidarropt(ydlidar.LidarPropSingleChannel, True)
        self.laser.setlidarropt(ydlidar.LidarPropMaxAngle, 180.0)  
        self.laser.setlidarropt(ydlidar.LidarPropMinAngle, -180.0)  
        self.laser.setlidarropt(ydlidar.LidarPropMaxRange, 16.0) 
        self.laser.setlidarropt(ydlidar.LidarPropMinRange, 0.12)  

        # Inicializa o Lidar
        ret = self.laser.initialize()
        if not ret:
            rospy.logerr("Falha ao inicializar o Lidar.")
            return
        
        # Liga o Lidar
        ret = self.laser.turnOn()
        if not ret:
            rospy.logerr("Falha ao ligar o Lidar.")
            return
        
        rospy.loginfo("Lidar inicializado e ligado com sucesso.")

    def publish_lidar_data(self):
        """Publica os dados do Lidar"""
        # Cria um objeto LaserScan
        scan = ydlidar.LaserScan()
        
        while ydlidar.os_isOk():
            # Processa o escaneamento
            ret = self.laser.doProcessSimple(scan)
            if not ret:
                rospy.logwarn("Falha ao obter dados do Lidar.")
                continue
            
            # Prepara a mensagem LaserScan para o ROS
            lidar_scan = LaserScan()
            lidar_scan.header.stamp = rospy.Time.now()
            lidar_scan.header.frame_id = ""  # A gente tem que referenciar com base no robô

            lidar_scan.angle_min = scan.config.min_angle * math.pi / 180  # Convertendo para radianos
            lidar_scan.angle_max = scan.config.max_angle * math.pi / 180  # Convertendo para radianos
            lidar_scan.angle_increment = scan.config.angle_increment * math.pi / 180  # Convertendo para radianos
            lidar_scan.time_increment = 1.0 / scan.config.scan_time
            lidar_scan.scan_time = scan.config.scan_time
            lidar_scan.range_min = 0.12  # Distância mínima
            lidar_scan.range_max = 16.0  # Distância máxima

            lidar_scan.ranges = scan.points  # Dados de distância (em metros)
            lidar_scan.intensities = [0] * len(scan.points)  # Intensidade

            # Publica os dados processados
            self.publisher.publish(lidar_scan)

            rospy.loginfo(f"Scan publicado: {len(scan.points)} pontos")
            time.sleep(0.05)  # Intervalo entre publicações

    def stop_lidar(self):
        """Desliga o Lidar"""
        self.laser.turnOff()
        self.laser.disconnecting()
        rospy.loginfo("Lidar desligado.")


if __name__ == "__main__":
    try:
        lidar_publisher = LidarPublisher()
        lidar_publisher.publish_lidar_data()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exceção do ROS, fechando o Lidar.")
    finally:
        lidar_publisher.stop_lidar()
