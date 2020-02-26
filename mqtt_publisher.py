#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
import paho.mqtt.publish as publish
import paho.mqtt.client as mqtt

#MQTT info
hostname = "mqtt.eclipse.org" #Sandbox broker
port = 1883 #Default port for unencrypted MQTT
topic = "HeraKules_robot001/test"

def callback(data):
    rospy.loginfo(rospy.get_caller_id() +
            " Get Pose info, sending to mqtt client.\n")
    #payload limited to string, bytearray, int, float or none
    #For bytearray: elements must be 0 <= x < 256
    mqttc.publish(topic, payload = str(data.x) + ' ' + str(data.y),
            qos = 0, retain = False)

def on_connect(client, userdata, flags, rc):
    #Successfully connection is '0'
    rospy.loginfo("Connection result: " + str(rc))

def on_publish(client, userdata, mid):
    rospy.loginfo("Message sent.")

def on_disconnect(client, userdata, rc):
    if rc != 0:
        rospy.loginfo("Disconnected unexpectedly")

mqttc = mqtt.Client()
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_disconnect = on_disconnect
mqttc.connect(hostname, port = port, keepalive = 60, bind_address = "")

def mqttPublisher():

    rospy.init_node('mqtt_publisher', anonymous=True)

    #First create a subscriber to acquire Pose info from
    #certain ROS topic.
    rospy.Subscriber('robot_pose', Pose2D, callback)

    rospy.spin()

if __name__ == '__main__':
    mqttPublisher()
