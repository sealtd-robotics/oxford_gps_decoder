#!/usr/bin/env python
import json
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, Twist
from common_tools.ultils import euler2quaternion
from collections import OrderedDict
from data_conversions import unpack16bitSignedInt, unpack16bitUnsignedInt, unpack32bitSignedInt, unpack8bitUnsignedInt 

file = "GPS_CAN_Database.json"

with open(file, 'rb') as f:
	data = json.load(f, object_pairs_hook=OrderedDict)

id_list = [frame["arbitration_id"] for frame in data["oxts_can_frames"]]

class gps_reading(object):
	def __init__(self):
		self.data_dict = dict()
		self.pos = NavSatFix()
		self.vel = TwistWithCovarianceStamped()
		self.imu = Imu()

	def call(self, input_id, data_array):
		self.process(input_id, data_array)
		self.vehicle_global_position()
		self.vehile_velocity()
		self.vehicle_imu()

	def check_id(self, input_id):
		for idx, i in enumerate(id_list):
			if input_id == i:
				return idx
		return None

	def decode(self, data_type, data):
		if data_type == "I16":
			return unpack16bitSignedInt(data)
		elif data_type == "U16":
			return unpack16bitUnsignedInt(data)
		elif data_type == "I32":
			return unpack32bitSignedInt(data)
		elif data_type == "U8":
			return unpack8bitUnsignedInt(data)

	def process(self, input_id, data_array):
		id_index = check_id(input_id)
		if id_index is not None:
			frame = data["oxts_can_frames"][id_index]
			for signal in frame["signals"]:
				start = signal["start_bit"]/8
				stop = start + signal["no_of_bits"]/8
				self.data_dict[signal["name"]] = decode(signal["data_type"], data_array[start:stop]) * signal["scale"]

	def vehicle_global_position(self):
		try:
			self.pos.latitude = self.data_dict["Latitude"]
			self.pos.longitude = self.data_dict["longitude"]
		except KeyError:
			pass

	def vehile_velocity(self):
		vel = Twist()
		try:
			vel.linear.x = self.data_dict["VelForward"]
			vel.linear.y = self.data_dict["VelLateral"]
		except KeyError:
			pass
		self.vel.twist.twist = vel

	def vehicle_imu(self):
		try:
			self.imu.orientation = euler2quaternion(self.data_dict["AngleRoll"], self.data_dict["AnglePitch"], self.data_dict["AngleHeading"])
			self.imu.angular_velocity.x = self.data_dict["RollRate"]
			self.imu.angular_velocity.y = self.data_dict["PitchRate"]
			self.imu.angular_velocity.z = self.data_dict["YawRate"]
			self.imu.linear_acceleration.x = self.data_dict["AccelX"]
			self.imu.linear_acceleration.y = self.data_dict["AccelY"]
			self.imu.linear_acceleration.z = self.data_dict["AccelZ"]
		except KeyError:
			pass

	
