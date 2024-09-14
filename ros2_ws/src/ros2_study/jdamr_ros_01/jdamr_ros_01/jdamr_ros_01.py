'''
/*MIT License

Copyright (c) 2024 JD edu. http://jdedu.kr author: conner.jeong@gmail.com
     
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
     
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
     
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN TH
SOFTWARE.*/

'''

#!/usr/bin/env python3
#from JDamr_lib import JDamr
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from rclpy.node import Node
import rclpy
import random
import serial
import threading 
import struct
import time 


'''
In this script, we study follwings:
1. How to write basic ROS node for robot car
2. Getting encoder data from Arduino and puslish encoder value  
'''

'''
Class for vehicle control 
'''
class JDamr(object):
    def __init__(self, com="/dev/ttyUSB0"):
        self.ser = serial.Serial(com, 115200)
        self.HEAD = 0xf5
       
        self.CMD_GET_ENCODER = 0x03

        self.encoder1 = 0
        self.encoder2 = 0
        self.encoder3 = 0
        self.encoder4 = 0

        if self.ser.isOpen():
            print("JDamr serial port opened!")
        else:
            print("Can't open JDamr serial port!")

        time.sleep(1)

    '''
    Protocol 
    - Packets have following bytes.
      - Header byte 
      - length byte
      - command byte
      - payload bytes 
      - checksum byte 
    '''

    # Starting receiving thread 
    def receive_data(self):     
        self.ser.flushInput()
        while True:
            head = bytearray(self.ser.read())[0]
            if head == self.HEAD:
                length = bytearray(self.ser.read())[0]  
                payload = [] 
                for i in range(length-1):
                    value = bytearray(self.ser.read())[0]
                    payload.append(value)
                self.parse_cmd(payload)

    # Python thread for receiving packet 
    def receive_thread(self):
        try:
            taks_name = "serial_thread"
            rx_task = threading.Thread(target=self.receive_data, name=taks_name)
            rx_task.setDaemon(True)
            rx_task.start()
            print("Start serial receive thread ")
            time.sleep(.05)
        except:
            pass

    # parsing command from incoming packet 
    def parse_cmd(self, payload):
        if self.CMD_GET_ENCODER == payload[0]:
            print(payload)
            encode1_str = payload[1:5]
            encode2_str = payload[5:9]
            encode3_str = payload[9:13]
            encode4_str = payload[13:17]
            self.encode1 = int.from_bytes(encode1_str, byteorder="big")
            print(self.encode1)
            self.encode2 = int.from_bytes(encode2_str, byteorder="big")
            print(self.encode2)
            self.encode3 = int.from_bytes(encode3_str, byteorder="big")
            print(self.encode3)
            self.encode4 = int.from_bytes(encode4_str, byteorder="big")
            print(self.encode4)

# ROS node class 
class jdamr_driver(Node): 
    def __init__(self):
        super().__init__('encoder_pub')
        self.pub = self.create_publisher(String, 'topic_encoder', 10)
        self.jdamr = JDamr()
        time.sleep(1)
        self.jdamr.receive_thread()

    def reset_amr(self):
        pass 

def main(args=None):
    rclpy.init(args=args)
    driver = jdamr_driver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()
            
if __name__ == '__main__':
    main()