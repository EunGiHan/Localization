#!/usr/bin/env python
#-*- coding:utf-8 -*-

import math
import rospy
import time
import numpy as np
# from std_msgs.msg import Float32
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu
# from geometry_msgs.msg import Vector3
from odometric_localization.msg import OdometricLocation
import numpy

class OdometricLocalization:
    def __init__(self):
        #ROS
        rospy.Subscriber("/imu/data", Imu, self.imu_data_callback)
        rospy.Subscriber("/imu/mag", MagneticField, self.imu_magnetic_callback)
        #########3# rospy.Subscriber("", , self.encoder_callback)

        self.odometric_loc_pub = rospy.Publisher("/odo_loc", OdometricLocation, queue_size=10)

        #record
        self.t_old = time.time()
        self.t_new = time.time()
        self.t_delta = self.t_new - self.t_old #sec 단위, float형
        self.new_loc = []
        self.routes = [] 
            # [ 0: time_now
            #   1: x
            #   2: y
            #   3: theta
            #   4: transitional_velocity
            #   5: rotational_velocity
            #   6: t_delta ]
        self.theta = 0
        self.transitional_velocity = 0 #전진속도 v_k
        self.rotational_velocity = 0 #각속도 w_k

        #imu
        self.magnetic_x = 0
        self.magnetic_y = 0
        self.magnetic_z = 0

        self.mag_x = 0
        self.mag_y = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        #encoder
        self.rotate_right = 12 ###########우측 바퀴 엔코더 회전값
        self.rotate_left = 10 ##########좌측 바퀴 엔코더 회전값
        self.radius_wheel = 0.2 #바퀴 반지름[m]
        self.distance_btw_wheel = 1 #좌우 바퀴 간 거리[m]

        self.routes.append([0, 0, 0, 0, 0, 0, 0]) #startpoint

    def imu_magnetic_callback(self, msg):
        self.magnetic_x = msg.magnetic_field.x
        self.magnetic_y = msg.magnetic_field.y
        self.magnetic_z = msg.magnetic_field.z

    def imu_data_callback(self, msg):
        self.pitch = 180 * math.atan2(msg.linear_acceleration.x, math.sqrt(msg.linear_acceleration.y**2 + msg.linear_acceleration.z**2))/math.pi
        self.roll = 180 * math.atan2(msg.linear_acceleration.y, math.sqrt(msg.linear_acceleration.y**2 + msg.linear_acceleration.z**2))/math.pi

        self.mag_x = self.magnetic_x*math.cos(self.pitch) + self.magnetic_y*math.sin(self.roll)*math.sin(self.pitch) + self.magnetic_z*math.cos(self.roll)*math.sin(self.pitch)
        self.mag_y = self.magnetic_y * math.cos(self.roll) - self.magnetic_z * math.sin(self.roll)
        self.yaw = 180 * math.atan2(-self.mag_y,self.mag_x)/math.pi

        self.theta = self.yaw

    # def encoder_callback(self, msg):
    #     self.rotate_left = msg.#################
    #     self.rotate_right = msg.################

    def calc_location(self):
        self.t_old = self.t_new
        self.t_new = time.time()
        self.t_delta = float(self.t_new - self.t_old)
        t = time.localtime(self.t_new)
        self.t_now = time.strftime('%Y-%m-%d %I:%M:%S %p', t) #현 시각 문자열로
        # self.t_now = time.ctime(self.t_new) #현 시각 문자열로

        self.transitional_velocity = self.radius_wheel * (self.rotate_right + self.rotate_left) / (2 * self.t_delta)
        self.rotational_velocity = self.radius_wheel * (self.rotate_right - self.rotate_left) / (2 * self.distance_btw_wheel)
        print("v=", self.transitional_velocity)
        print("w=", self.rotational_velocity)

        self.theta = self.routes[-1][3]
        print("theta=", self.theta)

        if self.rotational_velocity==0:
            #runge-kutta integration
            print("w=0")
            x = self.routes[-1][1] + self.transitional_velocity*self.t_delta*math.cos(self.theta + (self.rotational_velocity*self.t_delta)/2)
            y = self.routes[-1][2] + self.transitional_velocity*self.t_delta*math.sin(self.theta + (self.rotational_velocity*self.t_delta)/2)
        else:
            #exact integration
            print("w!=0")
            x = self.routes[-1][1] + (self.transitional_velocity/self.rotational_velocity) * (math.sin(self.routes[-1][3]) - math.sin(self.theta))
            y = self.routes[-1][2] - (self.transitional_velocity/self.rotational_velocity) * (math.cos(self.routes[-1][3]) - math.cos(self.theta))

        self.new_loc = [self.t_now, x, y, self.theta, self.transitional_velocity, self.rotational_velocity, self.t_delta]
        self.routes.append(self.new_loc)
        self.location_pub()
    
    def location_pub(self):
        loc = OdometricLocation()
        loc.t_now, loc.x, loc.y, loc.theta, loc.transitional_velocity, loc.rotational_velocity, loc.t_delta \
            = self.new_loc[0], self.new_loc[1], self.new_loc[2], self.new_loc[3], self.new_loc[4], self.new_loc[5], self.new_loc[6]
        self.odometric_loc_pub.publish(loc)

def main():
    rospy.init_node('OdometricLocalization', anonymous=False)
    rate = rospy.Rate(10)

    loc = OdometricLocalization()
    while not rospy.is_shutdown():
        loc.calc_location()
        rate.sleep()
    nparray = numpy.asarray(loc.routes)
    numpy.savetxt("routes.csv", nparray, fmt='%s', delimiter=",")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass

