#!/usr/bin/env python
import json
import os
from collections import OrderedDict
from oxford_gps_decoder.data_conversions import unpack16bitSignedInt, unpack16bitUnsignedInt, unpack32bitSignedInt, unpack8bitUnsignedInt 

SCRIPT_PATH = os.path.abspath(os.path.dirname(__file__))
GPS_CONFIG_FILE = os.path.abspath(os.path.join(
    SCRIPT_PATH, '..', 'config', 'GPS_CAN_Database.json'))

with open(GPS_CONFIG_FILE, 'rb') as f:
	data = json.load(f, object_pairs_hook=OrderedDict)

database = {}
_data = {}
for frame in data["oxts_can_frames"]:
	database[frame["arbitration_id"]] = frame
	for signal in frame["signals"]:
		_data[signal["name"]] = 0

decode = {
	"I16": unpack16bitSignedInt,
	"U16": unpack16bitUnsignedInt,
	"I32": unpack32bitSignedInt,
	"U8": unpack8bitUnsignedInt
}

class OxfordGPSCAN(object):
	def __init__(self):
		self.data = dict()
		self.database = dict()

		for frame in data["oxts_can_frames"]:
			self.database[frame["arbitration_id"]] = frame
			for signal in frame["signals"]:
				self.data[signal["name"]] = 0

	def process(self, can_id, data_array):
		""" Processing can id in databse """
		if can_id in self.database:
			self.data["imu_status"] = True
			frame = database[can_id]
			for signal in frame["signals"]:
				start = signal["start_bit"]/8
				stop = start + signal["no_of_bits"]/8
				self.data[signal["name"]] = decode[signal["data_type"]](data_array[start:stop]) \
											* signal["scale"]