#!/usr/bin/env python

import rospy, can
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, Twist
from common_tools.ultils import euler2quaternion
from gps_tools import gps_reading

bustype = 'socketcan'
channel = 'can1'
bus = can.interface.Bus(channel=channel, bustype=bustype)

def hook():
	print "It has been 30 seconds, there is no message coming from bus. Shutting down..."

def user():
	print "User press Ctrl + C"

class gps_can_reader():
	def __init__(self):
		self.gps = gps_reading()
		rospy.init_node('can1_gps_reader')
		pub_global_pos = rospy.Publisher('gps/global_position', NavSatFix, queue_size=1)
		pub_velocity = rospy.Publisher('gps/velocity', TwistWithCovarianceStamped, queue_size=1)
		pub_imu = rospy.Publisher('gps/imu', Imu, queue_size=1)

        while not rospy.is_shutdown():
        	can_msg = bus.recv(30)
        	if can_msg == None:
        		rospy.signal_shutdown("Shutting down")
        		rospy.on_shutdown(hook)
        	else:
        		self.gps.call(can_msg.arbitration_id, can_msg.data)
        		pub_global_pos.publish(self.gps.pos)
        		pub_velocity.publish(self.gps.vel)
        		pub_imu.publish(self.gps.imu)

if __name__ == "__main__":
	try:
		gps_can_reader()
	except rospy.ROSInterruptException:
		rospy.logerr('Error on CAN1 reading node')
	except KeyboardInterrupt:
		rospy.signal_shutdown("Shutting down")
		rospy.on_shutdown(user)