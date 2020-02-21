import struct
"""
Help functions
"""
def dec2hex(dec_value):
    return hex(dec_value)[2:] # example: hex(75) = 0x4b -> hex(75)[2:] = 4b

def hex2dec(hex_value):
    #return int(hex_value, 16) # works on python 3, does not work on python 2 if \xe in it
    return bytearray(hex_value)[0]

def dec2bin(dec_value):
    return "{0:08b}".format(dec_value) # show all leading zeros

# Purpose: Convert uint24 to integer
def bin2dec(bin_value):
    return int(bin_value, 2)

# Purpose: Convert uint64 to double
def bin2double(bin_value):
    q = int('0b'+ bin_value, 0)
    b8 = struct.pack('Q', q)
    return struct.unpack('d', b8)[0]

# Purpose: Convert uint32 to float
def bin2float(bin_value):
    q = int('0b'+ bin_value, 2)
    return struct.unpack('f', struct.pack('I', q))[0]
    
# Purpose: Unpack 24bit to signed integer & little endian
def unpack24bitSignedInt(bytes):
    return struct.unpack('<i', bytes + ('\0' if bytes[2] < '\x80' else '\xff'))[0]

# Purpose: Unpack 64bit to double & little endian
def unpack64bitDouble(bytes):
    return struct.unpack('<d', bytes)[0]

# Purpose: Unpack 32bit to float & little endian
def unpack32bitFloat(bytes):
    return struct.unpack('<f', bytes)[0]

# Purpose: Unpack 8bit to Unsigned Integer & little endian
def unpack8bitUnsignedInt(bytes):
    return struct.unpack('<B', bytes)[0]

# Purpose: Unpack 16bit to Integer & little endian
def unpack16bitSignedInt(bytes):
    return struct.unpack('<h', bytes)[0]

# Purpose: Unpack 16bit to unsigned Integer & little endian
def unpack16bitUnsignedInt(bytes):
    return struct.unpack('<H', bytes)[0]

# Purpose: Unpack 32bit to Integer & little endian
def unpack32bitSignedInt(bytes):
    return struct.unpack('<i', bytes)[0]

