#!/usr/bin/env python



from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import rospy
import time
import tf


class Position():

	def __init__(self):
		rospy.init_node('position_controller')


        self.edrone_cmd.rcRoll=0.0
        self.edrone_cmd.rcPitch=0.0
        self.edrone_cmd.rcYaw=0.0
        self.edrone_cmd.rcThrottle=0.0
        self.edrone_cmd.aux1=0.0
        self.edrone_cmd.aux2=0.0
        self.edrone_cmd.aux3=0.0
        self.edrone_cmd.aux4=0.0
        self.sample_time=0.060
        self.command=rospy.Publisher('/drone_command',edrone_cmd,queue_size=1)
        rospy.Subscriber('/edrone/gps',NavSatFix,self.gps_data)
        rospy.Subscriber('/input_gps',custom_gps,self.input_gps_callback)



    def gps_data(self, msg):
       self.lat=msg.latitude
       self.long=msg.longitude
       self.alt=msg.altitude



    def input_gps_callback(self,msg):
	   self.setpoint_lat=msg.set_latitude
	   self.setpoint_long=msg.set_longitude
	   self.setpoint_alt=msg.set_altitude




    def gps(self):

	
	    self.lat_error=self.setpoint_lat-self.lat
	    self.long_error=self.setpoint_long-self.long
	    self.alt_error=self.setpoint_alt-self.alt


        self.out_pitch=self.lat_error*50.0
        self.out_yaw=self.long_error*50.0
        self.command.rcPitch=self.out_pitch
        self.command.rcYaw=self.out_yaw
        self.command.publish(edrone_cmd)

if __name__== '__main__':
	      pose=Position()
	      r=rospy.Rate(pose.sample_time)
while not rospy.is_shutdown():
  pose.gps()
  r.sleep()


