from paho.mqtt import client as mqtt_client
from paho.mqtt.client import MQTTMessage as MQTTMessage
import random
import logging
import time
import json
import csv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry as Odo
from tf2_msgs.msg import TFMessage as TF    
from threading import Thread

# define MQTT client information. 
broker = '10.46.28.1'
port = 1883
mqtt_topic = "capra/robot/odometry"
client_id = f'python-mqtt-{random.randint(0, 1000)}'

FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60
FLAG_EXIT = False
data = []
data.append([0.0, 
             0.0, 
             0.0, 
             0.0, 
             0.0, 
             0.0, 
             0.0, 
             0.0, 
             'frame_id', 
             'child_frame_id', 
             0.0])
DEBUG_PATH = '/home/mike/Debug/delay_test_MQTT.csv'
NANO = 1e-9

# define ROS node publisher.
ros_topic = '/my_odo'
class Publisher(Node):

    def __init__(self):
        super().__init__('odo_pub')
        self.publisher_ = self.create_publisher(Odo,
                                                ros_topic,
                                                10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, 
                                       self.timer_callback)
        self.cnt = 0
        
    def timer_callback(self):
        msg = Odo()
        # msg.header.frame_id = 'test'
        # msg.header.stamp = data[-1][0]
        msg.header.stamp = self.get_clock().now().to_msg()
        # print(len(data[-1]))
        msg.header.frame_id = data[-1][8]
        msg.child_frame_id = data[-1][9]
        msg.pose.pose.position.x = data[-1][1]
        msg.pose.pose.position.y = data[-1][2]
        msg.pose.pose.position.z = data[-1][3]
        msg.pose.pose.orientation.x = data[-1][4]
        msg.pose.pose.orientation.y = data[-1][5]
        msg.pose.pose.orientation.z = data[-1][6]
        msg.pose.pose.orientation.w = data[-1][7]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing odometry messages to topic [%s]. ' % ros_topic)
        data[-1][10] = msg.header.stamp.sec + msg.header.stamp.nanosec * NANO
        # self.get_logger().info('Publishing odometry messages to topic [%s]. \nAnd the timestamp = %.4f' % (ros_topic, data[-1][0]))
        self.cnt += 1

        # # logging the data to debug
        # with open(DEBUG_PATH, mode='a', newline='') as f:
        #     writer = csv.writer(f, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        #     writer.writerow(data[-1])


# define MQTT client. 
def connect_callback(client, userdata, flags, rc):
    if rc == 0 and client.is_connected():
        print("Connected to MQTT Broker!")
        client.subscribe(mqtt_topic)
    else:
        print(f'Failed to connect, return code {rc}')


def disconnect_callback(client, userdata, rc):
    logging.info("Disconnected with result code: %s", rc)
    reconnect_count, reconnect_delay = 0, FIRST_RECONNECT_DELAY
    while reconnect_count < MAX_RECONNECT_COUNT:
        logging.info("Reconnecting in %d seconds...", reconnect_delay)
        time.sleep(reconnect_delay)

        try:
            client.reconnect()
            logging.info("Reconnected successfully!")
            return
        except Exception as err:
            logging.error("%s. Reconnect failed. Retrying...", err)

        reconnect_delay *= RECONNECT_RATE
        reconnect_delay = min(reconnect_delay, MAX_RECONNECT_DELAY)
        reconnect_count += 1
    logging.info("Reconnect failed after %s attempts. Exiting...", reconnect_count)
    global FLAG_EXIT
    FLAG_EXIT = True


def message_callback(client:mqtt_client.Client, userdata, msg:MQTTMessage):
    # print(f'Received `{msg.payload.decode()}` from `{msg.topic}` topic')
    print('Receiving odometry messages from MQTT client......')
    json_dic = json.loads(msg.payload.decode())
    timestamp = time.time()
    px = json_dic['pose']['pose']['position']['x']
    py = json_dic['pose']['pose']['position']['y']
    pz = json_dic['pose']['pose']['position']['z']
    ox = json_dic['pose']['pose']['orientation']['x']
    oy = json_dic['pose']['pose']['orientation']['y']
    oz = json_dic['pose']['pose']['orientation']['z']
    ow = json_dic['pose']['pose']['orientation']['w']
    frame_id = json_dic['header']['frame_id']
    child_frame_id = json_dic['child_frame_id']
    data.append([timestamp, 
                 px, 
                 py, 
                 pz, 
                 ox, 
                 oy, 
                 oz, 
                 ow, 
                 frame_id, 
                 child_frame_id, 
                 0.0])
    # with open('/home/mike/RobotLog/Odolog.csv', mode='aw', newline='') as f:
    #     writer = csv.writer(f, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    #     writer.writerows(data)


def ros2_main(args=None):
    rclpy.init(args=args)
    odo_pub = Publisher()
    rclpy.spin(odo_pub)
    odo_pub.destroy_node()
    rclpy.shutdown()


def mqtt_main():
    client = mqtt_client.Client(client_id)
    client.on_connect = connect_callback
    client.on_message = message_callback
    client.connect(broker, port, keepalive=120)
    client.on_disconnect = disconnect_callback
    client.loop_forever()


def main(args=None):
    Thread(target=mqtt_main).start()
    Thread(target=ros2_main).start()


if __name__ == '__main__':
    main()
