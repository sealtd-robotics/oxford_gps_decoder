import socket
from gps_msg_decode import decodeGPSRawMsg, GPS_specs

# UDP_IP = "195.0.0.35"
# UDP_PORT = 3000
# GPS specifications
GPS_filename = './GPS_params.csv'
param_bytesOrder, param_scaleFactor, param_names = GPS_specs(GPS_filename)

## sample
data = b'\xe7\x04=X\xc2\xff\xe0v\x00\xdax\xfe\xc8\xfe\xffG\xf7\xff\x89\x9a\x00\x04)~\xb5\x18.\xcef\xe6?\x00\xfd\xe7B\x05-\xf7\xbf\x88\xaf\x8cC\xa9\xe7\x00\xea\xcb\xff\x9b\xfd\xff\xa0\xf1\xfb\x15\x18\x00\r\xe5\xff\xbd\x07\xc5\xff1\x00\x1b\x00\x01\xff\x91'
decodeGPSRawMsg(data, param_bytesOrder, param_scaleFactor, param_names)

# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# sock.bind((UDP_IP, UDP_PORT))
while True:
    # data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print ("received message:")
    result = decodeGPSRawMsg(data, param_bytesOrder, param_scaleFactor, param_names)
    for i, name in enumerate(param_names):
        print (name, result[name])

