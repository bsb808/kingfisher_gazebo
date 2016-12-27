#!/usr/bin/env python
'''
Node to emulate integrated imu/gps sensor (e.g., microstrain) using P3D plugin.

The P3D plugin provides an Odometry ground truth (with noise).  The position is provided relative to the the Gazebo origin.  We define this origin in lat/lon and then transform the position to a latitude/longitude message.

Uses the navsat_conversion module
'''

import copy

import rospy
import tf
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

import navsat_conversions as nc

class Node():
    def __init__(self):
        self.outmsg = Odometry()
        self.outpub = rospy.Publisher("odometry/nav",Odometry,queue_size=10)
        self.sub = rospy.Subscriber("p3d/odom",Odometry,self.p3dcallback)
        rospy.loginfo("Subscribing to %s"%self.sub.name)
        self.rxodom = False
        self.originLat = 36.6
        self.originLong = -121.9
        utmx, utmy, utmzone = nc.LLtoUTM(self.originLat,self.originLong)
        self.originX = utmx
        self.originY = utmy
        self.originZone = utmzone
        rospy.loginfo("Origin at Lat/Lon %.6f/%.6f which is UTM X: %.4f Y: %.4f Zone: %s"%(self.originLat, self.originLong, self.originX, self.originY, self.originZone))
        self.seq = 0

    def p3dcallback(self,data):
        self.rxodom = True
        # copy the message - only x and y change
        self.outmsg = copy.deepcopy(data)
        # Find lat/long based on origin
        utmx = self.originX + data.pose.pose.position.x
        utmy = self.originY + data.pose.pose.position.y
        lat, lon = nc.UTMtoLL(utmx,utmy,self.originZone)
        #print("Lat %.10f, Long %.10f"%(lat,lon))
        self.outmsg.pose.pose.position.x = lon
        self.outmsg.pose.pose.position.y = lat

    def publishodom(self):
        if self.rxodom:
            self.outmsg.header.stamp = rospy.get_rostime()
            self.outmsg.header.seq = self.seq
            self.seq += 1
            self.outpub.publish(self.outmsg)

        
if __name__ == '__main__':
    
    rospy.init_node('p3d2nav', anonymous=True)
    
    # Initiate node object
    node=Node()
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        node.publishodom()
        r.sleep()

