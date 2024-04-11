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
from threading import Thread, Semaphore

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
data_receiving = [0.0, 
             0.0, 
             0.0, 
             0.0, 
             0.0, 
             0.0, 
             0.0, 
             0.0, 
             'frame_id', 
             'child_frame_id', 
             0.0]
data_publishing = [0.0, 
             0.0, 
             0.0, 
             0.0, 
             0.0, 
             0.0, 
             0.0, 
             0.0, 
             'frame_id', 
             'child_frame_id', 
             0.0]
DEBUG_PATH = '/home/mike/Debug/delay_test_MQTT.csv'
NANO = 1e-9
frameID = 0
isUpdated = False
semaphore = Semaphore(2)

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
        # while True:
        #     semaphore.acquire()
        #     try:
        #         if isUpdated:
        #             msg = Odo()
        #             msg.header.stamp = self.get_clock().now().to_msg()
        #             msg.header.frame_id = data[-1][8]
        #             msg.child_frame_id = data[-1][9]
        #             msg.pose.pose.position.x = data[-1][1]
        #             msg.pose.pose.position.y = data[-1][2]
        #             msg.pose.pose.position.z = data[-1][3]
        #             msg.pose.pose.orientation.x = data[-1][4]
        #             msg.pose.pose.orientation.y = data[-1][5]
        #             msg.pose.pose.orientation.z = data[-1][6]
        #             msg.pose.pose.orientation.w = data[-1][7]
        #             self.publisher_.publish(msg)
        #             self.get_logger().info('Publishing odometry messages to topic [%s]. ' % ros_topic)
        #             data[-1][10] = msg.header.stamp.sec + msg.header.stamp.nanosec * NANO
        #             isUpdated = False
        #     finally:
        #         semaphore.release()
        
    def timer_callback(self):
        global isUpdated, data_publishing, data_receiving
        semaphore.acquire
        try:
            if isUpdated:
                msg = Odo()
                data_publishing = data_receiving
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = data_publishing[8]
                msg.child_frame_id = data_publishing[9]
                msg.pose.pose.position.x = data_publishing[1]
                msg.pose.pose.position.y = data_publishing[2]
                msg.pose.pose.position.z = data_publishing[3]
                msg.pose.pose.orientation.x = data_publishing[4]
                msg.pose.pose.orientation.y = data_publishing[5]
                msg.pose.pose.orientation.z = data_publishing[6]
                msg.pose.pose.orientation.w = data_publishing[7]
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing odometry messages to topic [%s]. ' % ros_topic)

                # for delay test
                data_publishing[10] = msg.header.stamp.sec + msg.header.stamp.nanosec * NANO

                # msg.header.stamp = self.get_clock().now().to_msg()
                # msg.header.frame_id = data[-1][8]
                # msg.child_frame_id = data[-1][9]
                # msg.pose.pose.position.x = data[-1][1]
                # msg.pose.pose.position.y = data[-1][2]
                # msg.pose.pose.position.z = data[-1][3]
                # msg.pose.pose.orientation.x = data[-1][4]
                # msg.pose.pose.orientation.y = data[-1][5]
                # msg.pose.pose.orientation.z = data[-1][6]
                # msg.pose.pose.orientation.w = data[-1][7]
                # self.publisher_.publish(msg)
                # self.get_logger().info('Publishing odometry messages to topic [%s]. ' % ros_topic)
                # data[-1][10] = msg.header.stamp.sec + msg.header.stamp.nanosec * NANO
                isUpdated = False

                # # logging the data to test delay
                # with open(DEBUG_PATH, mode='a', newline='') as f:
                #     writer = csv.writer(f, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                #     writer.writerow(data_publishing)
        finally:
            semaphore.release()

        # # logging the data to test delay
        # with open(DEBUG_PATH, mode='a', newline='') as f:
        #     writer = csv.writer(f, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        #     writer.writerow(data_publishing)
        #     # writer.writerow(data[-1])


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
    global frameID, isUpdated, data_receiving
    semaphore.acquire()
    try:
        json_dic = json.loads(msg.payload.decode())
        timestamp = time.time()
        px = json_dic['pose']['pose']['position']['x']
        py = json_dic['pose']['pose']['position']['y']
        pz = json_dic['pose']['pose']['position']['z']
        ox = json_dic['pose']['pose']['orientation']['x']
        oy = json_dic['pose']['pose']['orientation']['y']
        oz = json_dic['pose']['pose']['orientation']['z']
        ow = json_dic['pose']['pose']['orientation']['w']
        frame_id = str(frameID)
        # frame_id = json_dic['header']['frame_id']
        child_frame_id = json_dic['child_frame_id']
        data_receiving = [timestamp, px, py, pz, ox, oy, oz, ow, frame_id, child_frame_id, 0.0]
        # data.append([timestamp, 
        #             px, 
        #             py, 
        #             pz, 
        #             ox, 
        #             oy, 
        #             oz, 
        #             ow, 
        #             frame_id, 
        #             child_frame_id, 
        #             0.0])
        frameID += 1
        isUpdated = True
    finally:
        semaphore.release()
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
    mqtt_thread = Thread(target=mqtt_main)
    node_thread = Thread(target=ros2_main)
    mqtt_thread.start()
    node_thread.start()
    mqtt_thread.join()
    node_thread.join()



if __name__ == '__main__':
    main()
