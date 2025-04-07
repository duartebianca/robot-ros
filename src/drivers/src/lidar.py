#!/usr/bin/env python3

import os
import ydlidar
import time
import math

# Configuração do filtro de suavização
WINDOW_SIZE = 5  # Tamanho da média móvel

def polar_to_cartesian(points):
    """ Converte coordenadas polares (ângulo, distância) em cartesianas (X, Y). """
    cartesian_points = []
    for p in points:
        if 0.12 <= p.range <= 16.0:  # Filtrando distâncias válidas
            x = p.range * math.cos(math.radians(p.angle))
            y = p.range * math.sin(math.radians(p.angle))
            cartesian_points.append((x, y))
    return cartesian_points

def smooth_data(points):
    """ Aplica um filtro de média móvel para suavizar os dados. """
    if len(points) < WINDOW_SIZE:
        return points  # Se poucos pontos, retorna como está

    smoothed = []
    for i in range(len(points) - WINDOW_SIZE + 1):
        avg_x = sum(p[0] for p in points[i:i+WINDOW_SIZE]) / WINDOW_SIZE
        avg_y = sum(p[1] for p in points[i:i+WINDOW_SIZE]) / WINDOW_SIZE
        smoothed.append((avg_x, avg_y))
    return smoothed

if __name__ == "__main__":
    ydlidar.os_init()
    laser = ydlidar.CYdLidar()
    ports = ydlidar.lidarPortList()

    # Detectando a porta do LiDAR
    port = "/dev/ydlidar"
    for key, value in ports.items():
        port = value
        print(f"LiDAR encontrado na porta: {port}")

    # Configuração do LiDAR
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)

    # Ajustes de varredura
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 7.0)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 2)
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
    laser.setlidaropt(ydlidar.LidarPropMinRange, 0.12)
    laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)

    # Inicializando e ligando o LiDAR
    if laser.initialize():
        if laser.turnOn():
            scan = ydlidar.LaserScan()
            
            while ydlidar.os_isOk():
                r = laser.doProcessSimple(scan)
                if r:
                    num_points = len(scan.points)
                    scan_freq = 1.0 / scan.config.scan_time
                    print(f"Scan recebido [{scan.stamp}]: {num_points} pontos | {scan_freq:.2f} Hz")

                    # Conversão para coordenadas cartesianas
                    cartesian_data = polar_to_cartesian(scan.points)

                    # Aplicando filtro de suavização
                    smoothed_data = smooth_data(cartesian_data)

                    # Exibindo apenas os primeiros 10 pontos filtrados e suavizados
                    print("Pontos Processados:")
                    for x, y in smoothed_data[:10]:
                        print(f"X: {x:.2f} m | Y: {y:.2f} m")

                else:
                    print("Falha ao obter dados do LiDAR.")

            laser.turnOff()
    else:
        print("Erro ao inicializar o LiDAR.")

    laser.disconnecting()
