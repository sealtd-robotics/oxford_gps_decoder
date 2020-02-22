from data_conversions import unpack24bitSignedInt, unpack64bitDouble, unpack32bitFloat, unpack8bitUByte, unpack16bitUnsignedInt
import os


readers = {
    "Longitude": unpack64bitDouble,
    "Latitude": unpack64bitDouble,
    "Altitude": unpack32bitFloat,
    "TimeMin": unpack32bitFloat,
    "NavSat": unpack8bitUByte,
    "StatusChannel": unpack8bitUByte,
    "PositionMode": unpack8bitUByte,
    "NumOfSatellites": unpack8bitUByte,
    "TimeMilliSec": unpack16bitUnsignedInt
}
# Process file with GPS parameters from RT manual
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
            value = unpack24bitSignedInt(data[dataIndices[0]:dataIndices[1]+1])*scaleFactor
    
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

