from data_conversions import dec2hex, hex2dec, dec2bin, bin2dec, bin2double, bin2float

# Process byte data and add to lists
def rawDataToList(data):
    dec_list = []
    hex_list = []
    bin_list = []
    for i in range(len(data)):
        # dec_list.append(data[i][]) # this is only used in python 3
        dec = hex2dec(data[i])
        dec_list.append(dec)
        hex_list.append(dec2hex(dec))
        bin_list.append(dec2bin(dec))
    return dec_list, hex_list, bin_list

# Process file with GPS parameters from RT manual
def GPS_specs(filename):
    """ Obtaining params from GPS specification file """
    param_bytesOrder = dict() 
    param_scaleFactor = dict()  # Final scale factor
    param_names = []
    with open(filename, 'r') as f:
        next(f), next(f) #skip two first rows
        lines = [line.replace('\n', '').split(',') for line in f]
        for line in lines:
            param_bytesOrder[line[1]] = [int(line[2]), int(line[3])] #range byte [from, to]
            param_scaleFactor[line[1]] = float(line[7])
            param_names.append(line[1])
            
    return param_bytesOrder, param_scaleFactor, param_names

# Decode data and add to dictionary by their names.
def decodeGPSRawMsg(data, param_bytesOrder, param_scaleFactor, param_names):
    """ Translate data """
    dec_list, hex_list, bin_list = rawDataToList(data)

    time = bin2dec("".join(bin_list[1:3][::-1]))

    # Data List
    result = dict()
    for i, name in enumerate(param_names):
        dataIndices = param_bytesOrder[name]
        bin_params = bin_list[dataIndices[0]:dataIndices[1]+1]
        scaleFactor = param_scaleFactor[name]
        binConcatenate = "".join(bin_params[::-1]) # concatenate reverse
    
        # Decode Conditions
        if name == "Longitude" or name == "Latitude":
            value = bin2double(binConcatenate)*scaleFactor
        elif name == "Altitude":
            value = bin2float(binConcatenate)*scaleFactor
        elif (binConcatenate[0] is "1"): # Two complements for binary 
            value = (bin2dec(binConcatenate[1::]) - 2**(len(binConcatenate)-1))*scaleFactor
        else:
            value = bin2dec(binConcatenate)*scaleFactor

        result[name] = value

    return result 

