#!/usr/bin/env python
# Author: An Nguyen
# Email: anguyen@sealimited.com
# Maintainer: An Nguyen

from __future__ import print_function
from __future__ import division
import rospy
import can
from datetime import datetime
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix, TimeReference
from oxford_gps_decoder.msg import OxfordIMU, StatusGPS, VelocityGPS
from oxford_gps_decoder.gps_msg_decode import CanGPS
from math import radians

UTC_START = datetime(1970,1,1,0,0,0)
GPS_LEAP_SECOND = 18
GPS_EPOCH_OFFSET = 0

bustype = 'socketcan'
channel = 'can0'
try:
	bus = can.interface.Bus(channel=channel, bustype=bustype)
	print ("setup canbus")
except IOError:
	canbus_available = False
	print ("{} is not availabe".format(channel))


def user():
	print ("User press Ctrl + C")


no_gps = {}
no_gps["GPSPosMode"] = 0
no_gps["NavStat"] = 0
no_gps["ImuStatus"] = False
no_gps["NumOfSatellites"] = 0

class OxTSGPS(CanGPS):
	def __init__(self):
		global GPS_EPOCH_OFFSET
		super(OxTSGPS, self).__init__()

		rospy.init_node('can1_gps_reader')
		self.data["ImuStatus"] = True
		current_date = None

		pub = {}
		pub["utc_time"] = rospy.Publisher('gps/utc_time', TimeReference, queue_size=1)
		pub["pos"] = rospy.Publisher('gps/global_position', NavSatFix, queue_size=1)
		pub["vel"] = rospy.Publisher('gps/velocity', VelocityGPS, queue_size=1)
		pub["imu"] = rospy.Publisher('gps/imu', OxfordIMU, queue_size=1)
		pub["status"] = rospy.Publisher('gps/status', StatusGPS, queue_size=1)

		while not rospy.is_shutdown():
			try:
				can_msg = bus.recv(0.1)
				if can_msg == None:
					pub["status"].publish(self.GPS_status(no_gps))
					# print ("Can1 timeout")
				else:
					self.process(can_msg.arbitration_id, can_msg.data)
					if can_msg.arbitration_id == 1536:
						if not current_date:
							current_date = datetime(self.data["Century"] + self.data["Year"], \
													self.data["Month"],\
													self.data["Day"], 0, 0, 0)

							GPS_EPOCH_OFFSET = (current_date - UTC_START).total_seconds()
						pub["utc_time"].publish(self.GPS_time(self.data))
					elif can_msg.arbitration_id == 1537:
						pub["pos"].publish(self.GPS_global_position(self.data))
					elif can_msg.arbitration_id == 1540:
						pub["vel"].publish(self.GPS_velocity(self.data))
					elif can_msg.arbitration_id == 1543:
						pub["imu"].publish(self.GPS_imu(self.data))
					elif can_msg.arbitration_id == 1280:
						pub["status"].publish(self.GPS_status(self.data))
			except can.CanError:
				pub["status"].publish(self.GPS_status(no_gps))
				print ("Can1 is down.")

	@staticmethod
	def GPS_time(data):
		""" Calculate UTC time """
		t = data["Hour"]*3600 \
			+ data["Minute"]*60 \
			+ data["Second"] \
			+ data["HSecond"]
		gps_time_ref = TimeReference()
		gps_time_ref.header.stamp = rospy.Time.now()
		gps_time_ref.time_ref = rospy.Time(GPS_EPOCH_OFFSET - GPS_LEAP_SECOND + t)
		gps_time_ref.source = "gps_utc"
		return gps_time_ref

	@staticmethod
	def GPS_global_position(data):
		"""Assign global position message"""
		pos = NavSatFix()
		pos.latitude = data['Latitude']
		pos.longitude = data['Longitude']
		return pos

	@staticmethod
	def GPS_velocity(data):
		""" Assign velocity message """
		msg_vel = VelocityGPS()
		msg_vel.forward = data["VelForward"]
		msg_vel.lateral = data["VelLateral"]
		return msg_vel

	@staticmethod
	def GPS_imu(data):
		""" Assign IMU message """
		imu_msg = OxfordIMU()
		imu_msg.orientation.x = data['AngleRoll']
		imu_msg.orientation.y = data['AnglePitch']
		imu_msg.orientation.z = radians(data['AngleHeading'])
		imu_msg.angular_velocity.x = data['RollRate']
		imu_msg.angular_velocity.y = data['PitchRate']
		imu_msg.angular_velocity.z = data['YawRate']
		imu_msg.linear_acceleration.x = data['AccelX']
		imu_msg.linear_acceleration.y = data['AccelY']
		imu_msg.linear_acceleration.z = data['AccelZ']
		return imu_msg

	@staticmethod
	def GPS_status(data):
		"""Check GPS status"""
		status = StatusGPS()
		status.no_of_satellites = data["NumOfSatellites"]
		status.imu_ready = data["ImuStatus"]

		if 3 <= data["GPSPosMode"] <= 7:
			status.gps_ready = True
		else:
			status.gps_ready = False

		status.gps_status = data["GPSPosMode"]
		return status

if __name__ == "__main__":
	try:
		OxTSGPS()
	except rospy.ROSInterruptException:
		rospy.logerr('Error on CAN1 reading node')
	except KeyboardInterrupt:
		rospy.signal_shutdown("Shutting down")
		rospy.on_shutdown(user) 
