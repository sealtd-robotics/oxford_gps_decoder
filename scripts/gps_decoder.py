#!/usr/bin/env python
# Author: An Nguyen
# Email: anguyen@sealimited.com
# Maintainer: An Nguyen

import socket
import struct
import rospy
import os
from math import pi
from oxford_gps_decoder.simple_gps_msg_decode import decodeGPSRawMsg, GPS_specs
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from oxford_gps_decoder.msg import OxfordIMU, StatusGPS, VelocityGPS
from geometry_msgs.msg import Twist
from custom_tools.utils import yaw_angle, velocity_resultant, velocity_longitudinal

UDP_PORT = 3000  # gps default port in all Oxforts GPS
UDP_IP = "0.0.0.0"
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.03)

SCRIPT_PATH = os.path.abspath(os.path.dirname(__file__))
GPS_CONFIG_FILE = os.path.abspath(os.path.join(
    SCRIPT_PATH, '..', 'config', 'GPS_params.csv'))

no_gps = {}
no_gps["PositionMode"] = 0
no_gps["NavStat"] = 0


class GPS_ethernet():
    heading = 0
    def __init__(self):
        """ Initate GPS_pub node """
        rospy.init_node("GPS_pub")
        sock.bind((UDP_IP, UDP_PORT))

        # Load configuration for GPS ethernet
        self.param_bytesOrder, self.param_scaleFactor, self.param_names = GPS_specs(GPS_CONFIG_FILE)
        self.ARP_IP = rospy.get_param('arp_gps_ip', default='195.0.0.96')

        # Subscriber
        rospy.Subscriber('/update_config', Bool, self.update_config)

        # Establish publisher
        pub = {}
        pub["pos"] = rospy.Publisher('gps/global_position', NavSatFix, queue_size=1)
        pub["vel"] = rospy.Publisher('gps/velocity', VelocityGPS, queue_size=1)
        pub["imu"] = rospy.Publisher('gps/imu', OxfordIMU, queue_size=1)
        pub["status"] = rospy.Publisher('gps/status', StatusGPS, queue_size=1)

        while not rospy.is_shutdown():
            try:
                data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
                if addr[0] == self.ARP_IP:  # look for arp_ip
                    self.data_eval(data, pub)
            except socket.timeout:
                pub["status"].publish(self.GPS_status(no_gps))

    def data_eval(self, data, pub):
        """ Publishing loop """
        gps_result = decodeGPSRawMsg(
            data, self.param_bytesOrder, self.param_scaleFactor, self.param_names)
        if (gps_result["NavStat"] == 4):  # Check for valid status, page 11 on NCOM manual
            pub["pos"].publish(self.GPS_global_position(gps_result))
            pub["vel"].publish(self.GPS_velocity(gps_result))
            pub["imu"].publish(self.GPS_imu(gps_result))
        elif (gps_result["NavStat"] == 1):  # RAW IMU
            pub["imu"].publish(self.GPS_imu(gps_result))

        # Check gps status
        # byte 62, Status channel, page 15 on NCOM Manual
        if gps_result["StatusChannel"] == 0:
            pub["status"].publish(self.GPS_status(gps_result))

    def update_config(self, msg):
        """Update configuration"""
        if msg.data:
            self.ARP_IP = rospy.get_param('arp_gps_ip')

    def GPS_global_position(self, data):
        """Assign global position message"""
        pos = NavSatFix()
        pos.latitude = data['Latitude']
        pos.longitude = data['Longitude']
        pos.altitude = data['Altitude']
        return pos

    def GPS_velocity(self, data):
        """ Assign velocity message """
        msg_vel = VelocityGPS()
        msg_vel.x = data['Velocity North']
        msg_vel.y = data['Velocity East']
        msg_vel.z = data['Velocity Down']
#	msg_vel.Vel2D = velocity_resultant(msg_vel.y, msg_vel.x, self.heading)
        msg_vel.Vel2D = velocity_longitudinal(msg_vel.y, msg_vel.x, self.heading)[0]
        return msg_vel

    def GPS_imu(self, data):
        """ Assign IMU message """
        imu_msg = OxfordIMU()
        self.heading = data['Heading']
        if data['Heading'] < 0:
            self.heading = 2*pi + data['Heading']
        imu_msg.orientation.x = data['Roll']
        imu_msg.orientation.y = data['Pitch']
        imu_msg.orientation.z = self.heading
        imu_msg.angular_velocity.x = data['Roll Rate']
        imu_msg.angular_velocity.y = data['Pitch Rate']
        imu_msg.angular_velocity.z = data['Yaw Rate']
        imu_msg.linear_acceleration.x = data['Ax']
        imu_msg.linear_acceleration.y = data['Ay']
        imu_msg.linear_acceleration.z = data['Az']
        return imu_msg

    def GPS_status(self, data):
        """Check GPS status"""
        status = StatusGPS()

        if "NumOfSatellites" in data:
            status.no_of_satellites = data["NumOfSatellites"]

        if "TimeMin" in data:
            status.min_since_1980 = data["TimeMin"]

        if 3 <= data["PositionMode"] <= 7:
            status.gps_status = True
        else:
            status.gps_status = False

        if data["PositionMode"] == 6 or data["PositionMode"] == 5:
            status.dgps_status = True
        else:
            status.dgps_status = False

        if data["NavStat"] != 0:
            status.imu_status = True
        else:
            status.imu_status = False
        return status

    def GPS_odometry(self, data):
        pass


if __name__ == '__main__':
    try:
        GPS_ethernet()
    except rospy.ROSInterruptException:
        rospy.logerr('Error on GPS reading node')
    finally:
        sock.close()
        rospy.logwarn('Closed port 3000 socket')
