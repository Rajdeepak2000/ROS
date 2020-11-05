#!/usr/bin/env python



from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import rospy
import time
import tf




class Position():

    def __init__(self):
        rospy.init_node('position_controller')

        self.nav_cmd=edrone_cmd()
        self.nav_cmd.rcRoll=0.0
        self.nav_cmd.rcPitch=0.0
        self.nav_cmd.rcYaw=0.0
        self.nav_cmd.rcThrottle=0.0
        self.nav_cmd.aux1=0.0
        self.nav_cmd.aux2=0.0
        self.nav_cmd.aux3=0.0
        self.nav_cmd.aux4=0.0
        self.sample_time=0.060
        self.setpoint_lat=0.0
        self.setpoint_long=0.0
        self.setpoint_alt=0.0
        self.lat=0.0
        self.long=0.0
        self.alt=0.0



        self.command=rospy.Publisher('/drone_command',edrone_cmd,queue_size=1)
        rospy.Subscriber('/edrone/gps',NavSatFix,self.gps_data)
        rospy.Subscriber('/input_gps',custom_gps,self.input_gps_callback)

        #addtional
        self.pub_lat_error=rospy.Publisher('/n_lat_error',Float64,queue_size=1)
        self.pub_long_error=rospy.Publisher('/n_long_error',Float64,queue_size=1)
        self.pub_alt_error=rospy.Publisher('/n_alt_error',Float64,queue_size=1)
        self.pub_out_pitch=rospy.Publisher('/n_out_pitch',Float64,queue_size=1)
        self.pub_out_yaw=rospy.Publisher('/n_out_yaw',Float64,queue_size=1)



    def gps_data(self, msg):
        self.lat=msg.latitude
        self.long=msg.longitude
        self.alt=msg.altitude



    def input_gps_callback(self, msg):
        self.setpoint_lat=msg.gps_lat
        self.setpoint_long=msg.gps_long
        self.setpoint_alt=msg.gps_alt




    def gps(self):

    
        self.lat_error=self.setpoint_lat-self.lat
        self.long_error=self.setpoint_long-self.long
        self.alt_error=self.setpoint_alt-self.alt


        self.out_pitch=self.lat_error*50.0
        self.out_yaw=self.long_error*50.0
        self.nav_cmd.rcPitch=self.out_pitch
        self.nav_cmd.rcYaw=self.out_yaw
        self.command.publish(self.nav_cmd)

        #additional
        self.pub_lat_error.publish(self.lat_error)
        self.pub_long_error.publish(self.long_error)
        self.pub_alt_error.publish(self.alt_error)
        self.pub_out_pitch.publish(self.nav_cmd.rcPitch)
        self.pub_out_yaw.publish(self.nav_cmd.rcYaw)

if __name__== '__main__':
          pose=Position()
          r=rospy.Rate(pose.sample_time)
while not rospy.is_shutdown():
  pose.gps()
  r.sleep()


