#!/usr/bin/env python

#################################################################################
#   Copyright Lars Ludvigsen. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

"""
imu_node.py

"""

import math
import threading
import serial
import time
import struct
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

#from BMI160_i2c import Driver
#from BMI160_i2c import definitions

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Pose
from imu_pkg import (constants)

from rcl_interfaces.msg import ParameterDescriptor, ParameterType


# Define the SerialPack structure
class SerialPack:
    def __init__(self):
        self.state = 0
        self.count = 0
        self.payload = [0]*512
        self.id = 0
        self.length = 0
        self.check = 0
        self.id_temp = 0
        self.length_temp = 0
        self.check_temp1 = 0
        self.check_temp2 = 0
        self.check_temp3 = 0
        self.check_temp4 = 0
        self.crcAccum = 1

    def Reset(self):
        self.state = 0
        self.count = 0
        self.crcAccum = 1


class IMUNode(Node):
    """Node responsible for collecting the camera and LiDAR messages and publishing them
       at the rate of the camera sensor.
    """

    def __init__(self):
        """Create a IMUNode.
        """
        super().__init__("imu_node")
        self.get_logger().info("IMU node initializing.")
        self.stop_queue = threading.Event()

        self.declare_parameter('com_id', constants.COM_ID,
                               ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('baud_rate', constants.BAUD_RATE,
                               ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('publish_rate', constants.IMU_MSG_RATE,
                               ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('zero_motion_odometer', False,
                               ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))

        self._com_id = self.get_parameter('com_id').value
        self._baud_rate = self.get_parameter('baud_rate').value
        self._publish_rate = self.get_parameter('publish_rate').value
        self._zero_motion = self.get_parameter('zero_motion_odometer').value

        self.get_logger().info("Connecting to IMU at bus {} baud rate {}".format(self._com_id, self._baud_rate))

        # Publisher that sends combined sensor messages with IMU acceleration and gyroscope data.
        self.imu_message_pub_cb_grp = ReentrantCallbackGroup()
        self.imu_message_publisher = self.create_publisher(Imu,
                                                           constants.IMU_MSG_TOPIC,
                                                           1,
                                                           callback_group=self.imu_message_pub_cb_grp)

        if self._zero_motion:
            self.odom_message_pub_cb_grp = ReentrantCallbackGroup()
            self.odom_message_publisher = self.create_publisher(Odometry,
                                                                constants.ODOM_MSG_TOPIC,
                                                                1,
                                                                callback_group=self.odom_message_pub_cb_grp)

        # Heartbeat timer.
        self.timer_count = 0
        self.timer = self.create_timer(5.0, self.timer_callback)


        self._parse = SerialPack()


        self.get_logger().info("IMU node created.")

    def timer_callback(self):
        """Heartbeat function to keep the node alive.
        """
        self.get_logger().debug(f"Timer heartbeat {self.timer_count}")
        self.timer_count += 1

    def __enter__(self):
        """Called when the node object is created using the 'with' statement.
        Returns:
           IMUNode : self object returned.
        """
        try:

            self.ser = serial.Serial(self._com_id, self._baud_rate, timeout=0.5)
            time.sleep(0.5)
   
        except Exception as ex:
            self.get_logger().info(f"Failed to create IMU monitor: {ex}")
            self.observer = None
            raise ex

        self.get_logger().info('Initialization and calibration of IMU sensor done.')

        self.thread = threading.Thread(target=self.processor)
        self.thread.start()

        # Start IMU event monitor.
        return self

    def __exit__(self, ExcType, ExcValue, Traceback):
        """Called when the object is destroyed.
        """
        self.get_logger().info('Exiting.')
        self.stop_queue.set()
        self.rate.destroy()
        self.thread.join()

    def processor(self):

        self.get_logger().info(f"Publishing messages at {self._publish_rate} Hz.")


        if self._zero_motion:
            self.get_logger().info(f"Publishing zero-motion odometry.")


        while not self.stop_queue.is_set() and rclpy.ok():

            data = self.ser.readline()

            #self.get_logger().info(f"read from tty: {data}")


            if len(data) > 0:

                parsed_data = self.parse_data(data)
                if parsed_data:
                    try:
                        self.publish_imu_message(parsed_data)
                    except Exception as ex:
                        self.get_logger().error(f"Failed to create IMU message: {ex}")

            else:
                time.sleep(1)




    def crc_crc32(self, crc, data):
        """实现crc_crc32函数
        """
        crc_table = [
            0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
            0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
            0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
            0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
            0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172, 0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
            0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
            0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
            0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
            0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
            0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
            0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
            0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
            0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
            0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0, 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
            0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
            0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
            0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A, 0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
            0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
            0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
            0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC, 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
            0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
            0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
            0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236, 0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
            0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
            0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
            0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38, 0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
            0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
            0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
            0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2, 0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
            0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
            0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
            0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94, 0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
        ]

        
        temp1 = (crc >> 8) & 0x00FFFFFF
        temp2 = ((crc ^ data) & 0xFF)
        crc = temp1 ^ crc_table[temp2]

        return crc



    def parse_data(self, raw):

        # Define the states
        IMU_PARSE_STATE_WAIT_SYNC1 = 0
        IMU_PARSE_STATE_WAIT_SYNC2 = 1
        IMU_PARSE_STATE_WAIT_ID1 = 2
        IMU_PARSE_STATE_WAIT_ID = 3
        IMU_PARSE_STATE_WAIT_LENGTH1 = 4
        IMU_PARSE_STATE_WAIT_LENGTH2 = 5
        IMU_PARSE_STATE_PAYLOAD = 6
        IMU_PARSE_STATE_CHECK1 = 7
        IMU_PARSE_STATE_CHECK2 = 8
        IMU_PARSE_STATE_CHECK3 = 9
        IMU_PARSE_STATE_CHECK4 = 10

        # Define the sync IDs
        IMU_PARSE_STATE_SYNC1_ID = 0xAA
        IMU_PARSE_STATE_SYNC2_ID = 0x55

        # 将字符串转化为字节
        #raw_bytes = bytes(raw, 'utf-8')
        # 循环处理每个字节
        for data in raw:

            #data = bin(byte)


            #self.get_logger().info(f"parse {hex(data)}  ")

            # Parse the raw data
            try:
                # Analyze the data
                if self._parse.state == IMU_PARSE_STATE_WAIT_SYNC1:

                    if data == IMU_PARSE_STATE_SYNC1_ID:
                        #self.get_logger().info("state machine start get IMU_PARSE_STATE_SYNC1_ID")
                        self._parse.state = IMU_PARSE_STATE_WAIT_SYNC2
                        self._parse.crcAccum = 1
                        self._parse.crcAccum = self.crc_crc32(self._parse.crcAccum, data)
                elif self._parse.state == IMU_PARSE_STATE_WAIT_SYNC2:
                    if data == IMU_PARSE_STATE_SYNC2_ID:
                        self._parse.crcAccum = self.crc_crc32(self._parse.crcAccum, data)
                        self._parse.state = IMU_PARSE_STATE_WAIT_ID1
                    else:
                        self._parse.state = IMU_PARSE_STATE_WAIT_SYNC1
                elif self._parse.state == IMU_PARSE_STATE_WAIT_ID1:
                    self._parse.id_temp = data
                    self._parse.crcAccum = self.crc_crc32(self._parse.crcAccum, data)
                    self._parse.state = IMU_PARSE_STATE_WAIT_ID
                elif self._parse.state == IMU_PARSE_STATE_WAIT_ID:
                    self._parse.id = data << 8 | self._parse.id_temp
                    self._parse.crcAccum = self.crc_crc32(self._parse.crcAccum, data)
                    self._parse.state = IMU_PARSE_STATE_WAIT_LENGTH1
                elif self._parse.state == IMU_PARSE_STATE_WAIT_LENGTH1:
                    self._parse.length_temp = data
                    self._parse.crcAccum = self.crc_crc32(self._parse.crcAccum, data)
                    self._parse.state = IMU_PARSE_STATE_WAIT_LENGTH2
                elif self._parse.state == IMU_PARSE_STATE_WAIT_LENGTH2:
                    self._parse.length = data << 8 | self._parse.length_temp
                    self._parse.crcAccum = self.crc_crc32(self._parse.crcAccum, data)
                    if self._parse.length > 0 and self._parse.length < len(self._parse.payload):
                        self._parse.count = 0
                        self._parse.state = IMU_PARSE_STATE_PAYLOAD
                    else:
                        self._parse.state = IMU_PARSE_STATE_WAIT_SYNC1
                elif self._parse.state == IMU_PARSE_STATE_PAYLOAD:
                    if self._parse.count < len(self._parse.payload):
                        self._parse.payload[self._parse.count] = data
                        self._parse.crcAccum = self.crc_crc32(self._parse.crcAccum, data)
                        self._parse.count += 1
                        if self._parse.count == self._parse.length:
                            self._parse.state = IMU_PARSE_STATE_CHECK1
                        
                    else:
                        self._parse.state = IMU_PARSE_STATE_WAIT_SYNC1
                elif self._parse.state == IMU_PARSE_STATE_CHECK1:
                    self._parse.check_temp1 = data
                    self._parse.state = IMU_PARSE_STATE_CHECK2
                elif self._parse.state == IMU_PARSE_STATE_CHECK2:
                    self._parse.check_temp2 = data
                    self._parse.state = IMU_PARSE_STATE_CHECK3
                elif self._parse.state == IMU_PARSE_STATE_CHECK3:
                    self._parse.check_temp3 = data
                    self._parse.state = IMU_PARSE_STATE_CHECK4
                elif self._parse.state == IMU_PARSE_STATE_CHECK4:
                    #self.get_logger().info("parse state change to CHECK4")
                    self._parse.check = (data << 24) | (self._parse.check_temp3 << 16) | (self._parse.check_temp2 << 8) | self._parse.check_temp1
                    pack = ''
                    if self._parse.check == self._parse.crcAccum:
                        
                        #self.get_logger().info(f"one parse done with {len(self._parse.)} bytes payload")

                        if self._parse.id == 0x166:
                            pack = self._parse.payload
                        else:
                            self.get_logger().error("not 0x166")

                    else:
                       self.get_logger().info(f"parse.check:{hex(self._parse.check)} crxAccum:{hex(self._parse.crcAccum)}")

                    self._parse.Reset()    
                    self._parse.state = IMU_PARSE_STATE_WAIT_SYNC1

                    return pack if len(pack) > 0 else None

            except Exception as ex:
                self.get_logger().error(f"Error in parsing data: {ex}")
                self._parse.Reset()
                return None



        return None

            
    def publish_imu_message(self, data):
        

        #self.get_logger().info(f"data: {bytes(data[:82])}")


        # 将data转化为NavRtPack数据结构
        nav_struct = struct.unpack('<IhiiifffffffffffffffBBBBHHII', bytes(data[:94]))



        #self.get_logger().info(f"nav struct: {nav_struct}")

        # GPS周内毫秒
        itow = nav_struct[0]
        # GPS周计数
        week_num = nav_struct[1]
        # 纬度
        lat_cor = nav_struct[2] / 10000000.0
        # 经度
        lon_cor = nav_struct[3] / 10000000.0
        # 高度
        alt_cor = nav_struct[4] / 1000.0
        # 北向速度m/s
        vn = nav_struct[5]
        # 东向速度m/s
        ve = nav_struct[6]
        # 垂向速度m/s
        vd = nav_struct[7]
        # 横滚角
        roll = nav_struct[8]
        # 俯仰角
        pitch = nav_struct[9]
        # 航向角——单天线
        yaw_ant_s = nav_struct[10]
        # 航向角——双天线
        yaw = nav_struct[11]
        # 预留
        wheel_angle = nav_struct[12]
        # 加速度计X(单位g)
        acc_x = nav_struct[13]
        # 加速度计Y(单位g)
        acc_y = nav_struct[14]
        # 加速度计Z(单位g)
        acc_z = nav_struct[15]
        # 滚转角速率X(单位deg/s)
        gyro_x = nav_struct[16]
        # 滚转角速率Y(单位deg/s)
        gyro_y = nav_struct[17]
        # 滚转角速率Z(单位deg/s)
        gyro_z = nav_struct[18]
        # 温度(℃)
        temp_imu = nav_struct[19]
        # 定位状态
        fix_type = nav_struct[20]
        # 星数
        sv_num = nav_struct[21]
        # 差分延时
        diff_age = nav_struct[22]
        # 定向状态
        heading_type = nav_struct[23]
        # 位置精度因子（cm）
        pos_acc = nav_struct[24]
        # 状态位
        status = nav_struct[25]
        # rev[0]预留1,rev[1]预留2
        rev = nav_struct[26:28]

        try:
            imu_msg = Imu()
            #data = self.sensor.getMotion6()

            if self._zero_motion:
                nomotion = self.sensor.getIntZeroMotionStatus()

                if nomotion:
                    odom_msg = Odometry()
                    odom_msg.header.stamp = self.get_clock().now().to_msg()
                    odom_msg.header.frame_id = 'odom'
                    odom_msg.child_frame_id = 'base_link'
                    odom_msg.pose.pose = Pose()
                    odom_msg.pose.covariance = constants.EMPTY_ARRAY_36
                    odom_msg.pose.covariance[0] = -1.0
                    odom_msg.twist.twist.linear = Vector3()
                    odom_msg.twist.covariance = constants.EMPTY_ARRAY_36

                    self.odom_message_publisher.publish(odom_msg)

            # fetch all gyro values - return in rad / sec
            gyro = Vector3()
            # swap x and y
            gyro.x = gyro_x
            # swap x and y
            gyro.y = gyro_y
            # upside-down
            gyro.z = gyro_z

            self.get_logger().info(f"update gyro to ({gyro_x}, {gyro_y}, {gyro_z})")

            # fetch all accel values - return in m/s²
            accel = Vector3()
            # swap x and y
            accel.x = acc_x
            # swap x and y
            accel.y = acc_y
            # upside-down
            accel.z = acc_z

            self.get_logger().info(f"update acc to ({acc_x}, {acc_y}, {acc_z})")

            imu_msg.angular_velocity = gyro
            imu_msg.angular_velocity_covariance = constants.COVAR_ARRAY_9

            imu_msg.linear_acceleration = accel
            imu_msg.linear_acceleration_covariance = constants.COVAR_ARRAY_9

            imu_msg.orientation_covariance = constants.EMPTY_ARRAY_9
            imu_msg.orientation_covariance[0] = -1.0

            # add header
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'base_link'

            self.get_logger().debug('gz: {:+.0f}'.format(gyro.z))

            self.imu_message_publisher.publish(imu_msg)

        except Exception as ex:
            self.get_logger().error(f"Error in publishing sensor message: {ex}")


def main(args=None):

    try:
        rclpy.init(args=args)
        with IMUNode() as imu_node:
            executor = MultiThreadedExecutor()
            rclpy.spin(imu_node, executor)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        imu_node.destroy_node()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
