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
from odometric_localization.msg import OdometricLocation  #custom msg


class OdometricLocalization:
    def __init__(self):
        #ROS
        rospy.Subscriber("/imu/data", Imu, self.imu_data_callback) ### 바꿀 것
        rospy.Subscriber("/imu/mag", MagneticField, self.imu_magnetic_callback)  ### 바꿀 것
        #########3# rospy.Subscriber("", , self.encoder_callback)

        self.odometric_loc_pub = rospy.Publisher("/odo_loc", OdometricLocation, queue_size=10)  # 위치 정보를 리스트로 담아 publish함

        #record
        self.t_old = time.time()
        self.t_new = time.time()
        self.t_delta = self.t_new - self.t_old  # 측정시간. sec 단위, float형
        self.new_loc = []   # 새로 계산된 현재 위치 정보
        self.routes = []    # 누적된 위치 정보
            # [ 0: time_now
            #   1: x
            #   2: y
            #   3: theta
            #   4: transitional_velocity
            #   5: rotational_velocity
            #   6: t_delta ]
        self.theta = 0      # 회전각
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
        self.rotate_right = 12  # 우측 바퀴 엔코더 회전값
        self.rotate_left = 10   # 좌측 바퀴 엔코더 회전값
        self.radius_wheel = 0.2 # 바퀴 반지름[m]
        self.distance_btw_wheel = 1 # 좌우 바퀴 간 거리[m]

        self.routes.append([0, 0, 0, 0, 0, 0, 0]) #startpoint

    # IMU 지자기 센서 콜백 함수
    def imu_magnetic_callback(self, msg):
        self.magnetic_x = msg.magnetic_field.x
        self.magnetic_y = msg.magnetic_field.y
        self.magnetic_z = msg.magnetic_field.z

    # IMU 각속도 센서 콜백 함수
    def imu_data_callback(self, msg):
        self.pitch = 180 * math.atan2(msg.linear_acceleration.x, math.sqrt(msg.linear_acceleration.y**2 + msg.linear_acceleration.z**2))/math.pi
        self.roll = 180 * math.atan2(msg.linear_acceleration.y, math.sqrt(msg.linear_acceleration.y**2 + msg.linear_acceleration.z**2))/math.pi

        self.mag_x = self.magnetic_x*math.cos(self.pitch) + self.magnetic_y*math.sin(self.roll)*math.sin(self.pitch) + self.magnetic_z*math.cos(self.roll)*math.sin(self.pitch)
        self.mag_y = self.magnetic_y * math.cos(self.roll) - self.magnetic_z * math.sin(self.roll)
        self.yaw = 180 * math.atan2(-self.mag_y,self.mag_x)/math.pi

        #self.theta = self.yaw

    # 엔코더 콜백 함수
    def encoder_callback(self, msg):
        #self.rotate_left = msg.#################
        #self.rotate_right = msg.################

    # 위치 계산 함수
    def calc_location(self):
        ## 시간 업데이트
        self.t_old = self.t_new
        self.t_new = time.time()
        self.t_delta = float(self.t_new - self.t_old)

        ## timestamp 기록하기 위해 시간 형태 가공
        t = time.localtime(self.t_new)
        self.t_now = time.strftime('%Y-%m-%d %I:%M:%S %p', t) #현 시각 문자열로 // 또는 self.t_now = time.ctime(self.t_new)

        ## 전진속도와 회전속도 계산 with 엔코더
        self.transitional_velocity = self.radius_wheel * (self.rotate_right + self.rotate_left) / (2 * self.t_delta)
        self.rotational_velocity = self.radius_wheel * (self.rotate_right - self.rotate_left) / (2 * self.distance_btw_wheel)
        print("v=", self.transitional_velocity)
        print("w=", self.rotational_velocity)


        ## 회전각 계산
        self.theta = self.routes[-1][3] + self.rotational_velocity*self.t_delta
        print("theta=", self.theta)

        ## x, y 좌표값 계산
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

        ## 계산해낸 현재 새 위치
        self.new_loc = [self.t_now, x, y, self.theta, self.transitional_velocity, self.rotational_velocity, self.t_delta]
        self.routes.append(self.new_loc)
        self.location_pub()
    
    # 현재 위치를 publish 함
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
    
    # 누적된 위치 정보를 csv로 저장
    nparray = numpy.asarray(loc.routes)
    numpy.savetxt("routes.csv", nparray, fmt='%s', delimiter=",")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass

