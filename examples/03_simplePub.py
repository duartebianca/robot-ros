#! /usr/bin/python3

import rospy
# Importação da biblioteca de mensagens padrões
from std_msgs.msg import Int32

# Criação do nó com o nome simple_publisher
rospy.init_node('simple_publisher')

# Criação do publisher no topic counter de um mensagem de tipo Int32
pub = rospy.Publisher('counter', Int32, queue_size=1)

rate = rospy.Rate(2)

count = 0

while not rospy.is_shutdown():
    # Publicação da mensagem no topico
    pub.publish(count)

    count += 1
    rate.sleep()
