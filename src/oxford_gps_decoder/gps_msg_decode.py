import data_conversions as dc
import os
import json
from collections import OrderedDict

SCRIPT_PATH = os.path.abspath(os.path.dirname(__file__))

readers = {
    "Longitude": dc.unpack64bitDouble,
    "Latitude": dc.unpack64bitDouble,
    "Altitude": dc.unpack32bitFloat,
    "TimeMin": dc.unpack32bitFloat,
    "NavSat": dc.unpack8bitUnsignedInt,
    "StatusChannel": dc.unpack8bitUnsignedInt,
    "PositionMode": dc.unpack8bitUnsignedInt,
    "NumOfSatellites": dc.unpack8bitUnsignedInt,
    "TimeMilliSec": dc.unpack16bitUnsignedInt
}

decode = {
	"I16": dc.unpack16bitSignedInt,
	"U16": dc.unpack16bitUnsignedInt,
	"I32": dc.unpack32bitSignedInt,
	"U8": dc.unpack8bitUnsignedInt
}
# Process file with GPS parameters from RT manual
class EthenetGPS(object):
    def __init__(self):
        GPS_CONFIG_FILE = os.path.abspath(os.path.join(
                        SCRIPT_PATH, '..', '..', 'config', 'GPS_params.csv'))
        self.param_bytesOrder = dict() 
        self.param_scaleFactor = dict()
        self.param_names = []
        self.data = dict()
        if os.path.isfile(GPS_CONFIG_FILE): 
            with open(GPS_CONFIG_FILE, 'r') as f:
                next(f), next(f) #skip two first rows
                lines = [line.replace('\n', '').split(',') for line in f]
                for line in lines:
                    self.param_bytesOrder[line[1]] = [int(line[2]), int(line[3])] # line1: name, range bytes: line2 to line3
                    self.param_scaleFactor[line[1]] = float(line[7]) # line1: name, scale: line7
                    self.param_names.append(line[1])
                    self.data[line[1]] = 0
        else:
            print('GPS file spec is not found. Please check!')

    def process(self, buf):
        for i, name in enumerate(self.param_names):
            dataIndices = self.param_bytesOrder[name]
            scaleFactor = self.param_scaleFactor[name]
            if name in readers:
                value = readers[name](buf[dataIndices[0]:dataIndices[1]+1])*scaleFactor
            else:
                value = dc.unpack24bitSignedInt(buf[dataIndices[0]:dataIndices[1]+1])*scaleFactor
            self.data[name] = value


def GPS_specs(filename):
    """ Obtaining params from GPS specification file """
    param_bytesOrder = dict() 
    param_scaleFactor = dict()
    param_names = []
    if os.path.isfile(filename): 
        with open(filename, 'r') as f:
            next(f), next(f) #skip two first rows
            lines = [line.replace('\n', '').split(',') for line in f]
            for line in lines:
                param_bytesOrder[line[1]] = [int(line[2]), int(line[3])] # line1: name, range bytes: line2 to line3
                param_scaleFactor[line[1]] = float(line[7]) # line1: name, scale: line7
                param_names.append(line[1])
                
        return param_bytesOrder, param_scaleFactor, param_names
    else:
        print('GPS file spec is not found. Please check!')

# Decode data and add to dictionary by their names.
def decodeGPSRawMsg(data, param_bytesOrder, param_scaleFactor, param_names):

    # Decode
    result = dict()
    for i, name in enumerate(param_names):
        dataIndices = param_bytesOrder[name]
        scaleFactor = param_scaleFactor[name]

        if name in readers:
            value = readers[name](data[dataIndices[0]:dataIndices[1]+1])*scaleFactor
        else:
            value = dc.unpack24bitSignedInt(data[dataIndices[0]:dataIndices[1]+1])*scaleFactor
    
        # if name == "Longitude" or name == "Latitude":
        #     value = unpack64bitDouble(data[dataIndices[0]:dataIndices[1]+1])*scaleFactor
        # elif name == "Altitude" or name == "TimeMin":
        #     value = unpack32bitFloat(data[dataIndices[0]:dataIndices[1]+1])*scaleFactor
        # elif name == "NavStat" or name == "StatusChannel" or name == "PositionMode" or name == "NumOfSatellites":
        #     value = unpack8bitUByte(data[dataIndices[0]])
        # elif name == "TimeMilliSec":
        #     value = unpack16bitUnsignedInt(data[dataIndices[0]:dataIndices[1]+1])
        # else:
        #     value = unpack24bitSignedInt(data[dataIndices[0]:dataIndices[1]+1])*scaleFactor

        result[name] = value

    return result 

class CanGPS(object):
    def __init__(self):
        GPS_CONFIG_FILE = os.path.abspath(
            os.path.join(SCRIPT_PATH, '..', '..', 'config', 'GPS_CAN_Database.json'))

        with open(GPS_CONFIG_FILE, 'rb') as f:
            database = json.load(f, object_pairs_hook=OrderedDict)

        self.data = dict()
        self.frame_database = dict()

        for frame in database["oxts_can_frames"]:
            self.frame_database[frame["arbitration_id"]] = frame
            for signal in frame["signals"]:
                self.data[signal["name"]] = 0

    def process(self, can_id, data_array):
        """ Processing can id in databse """
        if can_id in self.frame_database:
            frame = self.frame_database[can_id]
            for signal in frame["signals"]:
                start = signal["start_bit"]/8
                stop = start + signal["no_of_bits"]/8
                self.data[signal["name"]] = decode[signal["data_type"]](data_array[start:stop]) \
                                            * signal["scale"]

