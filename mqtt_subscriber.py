#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
import paho.mqtt.client as mqtt

#MQTT info
hostname = "mqtt.eclipse.org" #Sandbox broker
port = 1883 #Default port for unencrypted MQTT
topic = "HeraKules_robot001/#"

pub = rospy.Publisher('robot_pose', Pose2D, queue_size = 1000)

def on_connect(client, userdata, flags, rc):
    #Successfully connection is '0'
    rospy.loginfo("Connection result: " + str(rc))
    if rc == 0:
        client.subscribe(topic)

def on_message(client, userdata, message):
    rospy.loginfo("Received message on " + message.topic +
            ":\n" + str(message.payload))
    comm_msg = Pose2D()
    comm_msg.x, comm_msg.y = tuple([float(i)
        for i in str(message.payload).split(' ')])
    comm_msg.theta = 0.0
    pub.publish(comm_msg)

def on_subscribe(client, userdata, mid):
    rospy.loginfo("Message received.")

def on_disconnect(client, userdata, rc):
    if rc != 0:
        rospy.loginfo("Disconnected unexpectedly")

#Initialize client instance
client = mqtt.Client()
#Bind events to functions
client.on_connect = on_connect
client.on_message = on_message
client.on_subscribe = on_subscribe
client.on_disconnect = on_disconnect
#Connect to the specified broker
client.connect(hostname, port = port)

def mqttSubscriber():
    
    rospy.init_node('mqtt_subsriber', anonymous = True)

    client.subscribe("topic", qos = 0)    

    #Network loop runs in the background to listen to the events
    while not rospy.is_shutdown():
        client.loop()

if __name__ == '__main__':
    try:
        mqttSubscriber()
    except rospy.ROSInterruptException:
        pass
