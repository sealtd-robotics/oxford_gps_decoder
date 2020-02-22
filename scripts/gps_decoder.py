#!/usr/bin/env python
# Author: An Nguyen
# Email: anguyen@sealimited.com
# Maintainer: An Nguyen

from __future__ import division
import rospy
import socket
import struct
import os
from math import pi, sin, cos
from oxford_gps_decoder.gps_msg_decode import decodeGPSRawMsg, GPS_specs
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix, TimeReference
from oxford_gps_decoder.msg import OxfordIMU, StatusGPS, VelocityGPS

# To measure UTC
GPS_LEAP_SECOND = 18 # between 01/01/1980 and 12/31/2016
GPS_EPOCH_OFFSET = 315964800 # difference between 01/01/1970 00:00:00 and 01/01/1980 00:00:00

# UDP socket setting
UDP_PORT = 3000  # gps default port in all Oxforts GPS
UDP_IP = "0.0.0.0"
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.03)

# Config path
SCRIPT_PATH = os.path.abspath(os.path.dirname(__file__))
GPS_CONFIG_FILE = os.path.abspath(os.path.join(
    SCRIPT_PATH, '..', 'config', 'GPS_params.csv'))

no_gps = {}
no_gps["PositionMode"] = 0
no_gps["NavStat"] = 0


def velocity_level(v_east, v_north, heading_rad):
    """Output velocity forward and lateral"""
    v_forward = v_north*cos(heading_rad) + v_east*sin(heading_rad)
    v_lateral = v_north*sin(heading_rad) + v_east*cos(heading_rad)
    return v_forward, v_lateral


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
        pub["time"] = rospy.Publisher('gps/time', TimeReference, queue_size=1)
        pub["pos"] = rospy.Publisher('gps/global_position', NavSatFix, queue_size=1)
        pub["vel"] = rospy.Publisher('gps/velocity', VelocityGPS, queue_size=1)
        pub["imu"] = rospy.Publisher('gps/imu', OxfordIMU, queue_size=1)
        pub["status"] = rospy.Publisher('gps/status', StatusGPS, queue_size=1)

        while not rospy.is_shutdown():
            try:
                buf, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
                if addr[0] == self.ARP_IP:  # look for arp_ip
                    self.data_eval(buf, pub)
            except socket.timeout:
                pub["status"].publish(self.GPS_status(no_gps))

    def data_eval(self, buf, pub):
        """ Publishing loop """
        dat = decodeGPSRawMsg(buf, self.param_bytesOrder, \
                                self.param_scaleFactor, \
                                self.param_names)
        if (dat["NavStat"] == 4):  # Check for valid status, page 11 on NCOM manual
            pub["time"].publish(self.GPS_time(dat))
            pub["pos"].publish(self.GPS_global_position(dat))
            pub["vel"].publish(self.GPS_velocity(dat))
            pub["imu"].publish(self.GPS_imu(dat))
        elif (dat["NavStat"] == 1):  # RAW IMU
            pub["imu"].publish(self.GPS_imu(dat))

        # Check gps status
        # byte 62, Status channel, page 15 on NCOM Manual
        if dat["StatusChannel"] == 0:
            pub["status"].publish(self.GPS_status(dat))
            if dat["TimeMin"] >= 1000:
                self.current_time_chan0 = GPS_EPOCH_OFFSET - GPS_LEAP_SECOND + dat["TimeMin"]*60

    def update_config(self, msg):
        """Update configuration"""
        if msg.data:
            self.ARP_IP = rospy.get_param('arp_gps_ip')

    def GPS_time(self, data):
        """Assign UTC time"""
        gps_time_ref = TimeReference()
        gps_time_ref.header.stamp = rospy.Time.now() 
        gps_time_ref.time_ref = rospy.Time(self.current_time_chan0 + data["TimeMilliSec"]*0.001)
        gps_time_ref.source = "gps"
        return gps_time_ref

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
        msg_vel.forward, msg_vel.lateral = velocity_level(msg_vel.y, msg_vel.x, self.heading)
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


if __name__ == '__main__':
    try:
        GPS_ethernet()
    except rospy.ROSInterruptException:
        rospy.logerr('Error on GPS reading node')
    finally:
        sock.close()
        rospy.logwarn('Closed port 3000 socket')
