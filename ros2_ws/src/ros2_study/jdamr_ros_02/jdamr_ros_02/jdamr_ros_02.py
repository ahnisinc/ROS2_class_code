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
from rclpy.clock import ROSClock
from rclpy.time import Time
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import random
import serial
import threading 
import struct
import time 


'''
In this script, we study follwings:
1. How to write basic ROS node for robot car
2. This node subscribes teleop node using "cmd_vel" topic.
3. This node control robot through serial according to teleop input. 
'''

class JDamr(object):
    def __init__(self, com="/dev/ttyUSB0"):
        self.ser = serial.Serial(com, 115200)
        self.HEAD = 0xf5
        self.CMD_SET_MOTOR = 0x01
        self.CMD_GET_SPEED = 0x02
        self.CMD_GET_ENCODER = 0x03
        self.CMD_CAR_RUN = 0x04

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

    def set_motor(self, speed_1, speed_2, speed_3, speed_4):
        try:
            speed_a = bytearray(struct.pack('b', speed_1))
            speed_b = bytearray(struct.pack('b', speed_2))
            speed_c = bytearray(struct.pack('b', speed_3))
            speed_d = bytearray(struct.pack('b', speed_4))
            cmd = [self.HEAD, 0x00, self.CMD_SET_MOTOR,
                    speed_a[0], speed_b[0], speed_c[0], speed_d[0]]
            cmd[1] = len(cmd) - 1
            checksum = 0xff #sum(cmd) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            print("motor:", cmd)
            time.sleep(0.1)
        except:
            print("set_motor error")
            pass
    
    '''
    drive_mode: 
    1: go foreward 
    2: go backward 
    3: turn left 
    4: turn right 
    speed: 1 ~ 100 
    '''
    def set_car_run(self, drive_mode, speed):
        try:
            speed_0 = bytearray(struct.pack('b', speed))
            drive_mode_0 = bytearray(struct.pack('b', drive_mode))
            cmd = [self.HEAD, 0x00, self.CMD_CAR_RUN, drive_mode_0[0], speed_0[0]]
            cmd[1] = len(cmd) - 1
            checksum = 0xff #sum(cmd) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            print("car_run:", cmd)
            time.sleep(0.1)
        except:
            print('---set_car_run error!---')
            pass

class jdamr_driver(Node): 
    def __init__(self):
        super().__init__('encoder_pub')
        self.pub = self.create_publisher(String, 'topic_encoder', 10)
        self.jdamr = JDamr()
        # This variable hold prefixs for joint state 
        
        self.declare_parameter('prefix', '')
        self.Prefix = self.get_parameter('prefix').get_parameter_value().string_value
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 100)
        self.states_pub = self.create_publisher(JointState, 'joint_states', 100)
        time.sleep(1)
        self.clock = ROSClock()
        self.jdamr.receive_thread()

    def reset_amr(self):
        self.cmd_vel_sub.unregister()
        self.states_pub.unregister()
    i = -3.14
    def pub_data(self):
        '''
        In this step, we add only joinst state. 
        '''
        count = 0
        i = -3.14
        while not rclpy.ok():
            # preparing joint state 
            state = JointState()
            now = self.clock.now()  # Get current ROS time
            state.header.stamp = Time(now, 0)  # Set header stamp
            state.header.frame_id = 'joint_states'
            if len(self.Prefix)==0:
                state.name = ["wheel1_joint", "wheel2_joint",
                              "wheel3_joint", "wheel4_joint"]
            else:
                state.name = [self.Prefix+"/wheel1_joint", self.Prefix+"/wheel2_joint",
                              self.Prefix+"/wheel3_joint", self.Prefix+"/wheel4_joint"]
            # In this step, we make fake joint states. 
            count += 1

            if count > 100000:
                i += 0.1 #i = random.uniform(-3.14, 3.14)
                if i > 3.14:
                    i = -3.14
                state.position = [i, i, i, i]
                print(i)
                self.states_pub.publish(state)
                count = 0

    def cmd_vel_callback(self, msg):
        if not isinstance(msg, Twist):
            return 
        x = msg.linear.x
        y = msg.linear.y
        angle = msg.angular.z
        self.get_logger().info("cmd_velx: {}, cmd_vely: {}, cmd_ang: {}".format(x, y, angle))
        '''
        If you press 'U' key at ROS teleop, so angle value is above 0.5, speed of 4 motors are set to 100.
        It will let Arduino LED 13 turned on. 
        '''
        if angle >= 0.5:
            self.jdamr.set_motor(100, 100, 100, 100)
        else:
            self.jdamr.set_motor(-127, -127, -127, -127)

def main(args=None):
    rclpy.init(args=args)
    driver = jdamr_driver()
    driver.pub_data()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()