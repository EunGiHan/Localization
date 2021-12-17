#!/usr/bin/env python
#-*- coding:utf-8 -*-

import math
import serial
import struct
import rospy
import time
import numpy as np
# from std_msgs.msg import Float32
# from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu, PointCloud
# from geometry_msgs.msg import Vector3
from odometric_localization.msg import OdometricLocation  #custom msg
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32

class OdometricLocalization:
    def __init__(self):
        ### ROS
        self.ser = serial.Serial("/dev/gigacha/erp42", 115200)
        rospy.Subscriber("/imu", Imu, self.imu_data_callback)  #IMU의 가속도, 각속도 값을 받아옴
        rospy.Subscriber("/Displacement_right",Int64, self.encoder_right_callback)
        rospy.Subscriber("/Displacement_left",Int64, self.encoder_left_callback)
        rospy.Subscriber("/pose1",Odometry, self.location_pub_from_gps)
        self.odometric_loc_pub = rospy.Publisher("/odo_loc", OdometricLocation, queue_size=10)  # 현재의 위치 정보를 publish함

        ### Record & Values
        self.t_old = time.time()
        self.t_new = time.time()
        self.t_delta = self.t_new - self.t_old  # 측정시간. sec 단위, float형
        self.new_loc = []   # 새로 계산된 현재 위치 정보
        self.routes = []    # 위치 정보 누적
            # [ 0: time_now - 측정 타임스탬프
            #   1: x - x좌표
            #   2: y - y좌표
            #   3: theta - 회전 반경(IMU 기준 코드에서는 참고하지 말 것)
            #   4: transitional_velocity - 전진 속도
            #   5: rotational_velocity - 회전 속도(IMU 기준 코드에서는 yaw 값으로 넣음)
            #   6: t_delta - 측정 시간 간격]
        self.theta = 0  #회전각(회전반경)(엔코더 버전에서 사용)
        self.yaw = 0  #조향각(IMU 버전에서 사용)
        self.transitional_velocity = 0 #전진속도 v_k
        self.rotational_velocity = 0 #각속도 w_k

        ### IMU data
        self.linear_accel_x_callback = 0  #콜백함수에서 받아오는 값
        self.linear_accel_x = 0  #연산 시점의 callback을 받아올 변수
        self.linear_accel_y_callback = 0
        self.linear_accel_y = 0
        self.delta_theta_callback = 0
        self.delta_theta = 0

        ### Encoder data
        self.rotate_right = 0  # 우측 바퀴 엔코더 회전값
        self.rotate_left = 0   # 좌측 바퀴 엔코더 회전값
        self.radius_wheel = 0.25 # 바퀴 반지름[m]
        self.distance_btw_wheel = 0.97 # 좌우 바퀴 간 거리[m]

        ### 시작지점에서 모두 0으로 기록 시작함
        self.routes.append([0, 0, 0, 0, 0, 0, 0]) #startpoint


        self.cur_points = PointCloud()
        self.cur_points.header.frame_id = 'world'
        self.cur_points.header.stamp = rospy.Time.now()

        self.gps_points = PointCloud()
        self.gps_points.header.frame_id = 'world'
        self.gps_points.header.stamp = rospy.Time.now()

        self.pub_c = rospy.Publisher('/cur_xy', PointCloud, queue_size=1)
        self.pub_g = rospy.Publisher('/gps_xy', PointCloud, queue_size=1)


    ### IMU 가속도, 각속도 센서 콜백 함수
    def imu_data_callback(self, msg):
        #### DUMP!! (지우기 아까워서 내버려둔 부분)
        # self.pitch = 180 * math.atan2(msg.linear_acceleration.x, math.sqrt(msg.linear_acceleration.y**2 + msg.linear_acceleration.z**2))/math.pi
        # self.roll = 180 * math.atan2(msg.linear_acceleration.y, math.sqrt(msg.linear_acceleration.y**2 + msg.linear_acceleration.z**2))/math.pi
        # self.mag_x = self.magnetic_x*math.cos(self.pitch) + self.magnetic_y*math.sin(self.roll)*math.sin(self.pitch) + self.magnetic_z*math.cos(self.roll)*math.sin(self.pitch)
        # self.mag_y = self.magnetic_y * math.cos(self.roll) - self.magnetic_z * math.sin(self.roll)
        # self.yaw = 180 * math.atan2(-self.mag_y,self.mag_x)/math.pi

        ### x, y 가속도와 z 각속도를 받아옴
        self.linear_accel_x_callback = msg.linear_acceleration.x
        self.linear_accel_y_callback = msg.linear_acceleration.y
        self.delta_theta_callback = msg.angular_velocity.z  #[rad/s]


    ### 엔코더 우측 바퀴 콜백 함수
    def encoder_right_callback(self, msg):
        self.rotate_right_callback = msg.data/100
    

    ### 엔코더 좌측 바퀴 콜백 함수
    def encoder_left_callback(self):
        #while문 제거함
        #범위를 [11:14]에서 [11:15]로 변경함. 맨 끝 값이 안들어오고 있었음
        #따라서 tmp1 = [packet[11], packet[12], packet[13], packet[14]]으로 들어감
        #만약에 안되면 포맷을 "i"에서 "BBBB"로 변경해봤으면 좋겠음!
        packet = self.ser.read_until(b'\x0d\x0a')   
        tmp1 = []
        tmp1 = struct.unpack("i", packet[11:15]) 
        print(tmp1)

        self.rotate_left_callback = tmp1[0] + tmp1[1]*256 + tmp1[2]*256**2 + tmp1[3]*256**3
        self.rotate_left_callback = self.rotate_left_callback/100


    def location_pub_from_gps(self,msg):
        self.x  = msg.pose.pose.position.x
        self.y  = msg.pose.pose.position.y
        self.k  = msg.twist.twist.angular.z

        gps_point = Point32()
        gps_point.x = self.x
        gps_point.y = self.y
        self.gps_points.points.append(gps_point)

        self.pub_g.publish(self.gps_points)
        

    ### 현재 위치 계산 함수
    def calc_location(self):
        ### 현재 시간에서의 가속도와 각속도 값을 불러옴 (나중에 엔코더 부분도 추가해야 함)
        self.linear_accel_x = self.linear_accel_x_callback
        self.linear_accel_y = self.linear_accel_y_callback
        self.rotate_right = self.rotate_right_callback
        self.rotate_left = self.rotate_left_callback
        self.delta_theta = self.delta_theta_callback

        ### 시간 업데이트
        self.t_old = self.t_new
        self.t_new = time.time()
        self.t_delta = float(self.t_new - self.t_old)

        ### timestamp 기록하기 위해 시간 형태 가공
        t = time.localtime(self.t_new)
        self.t_now = time.strftime('%Y-%m-%d %I:%M:%S %p', t) #현 시각 문자열로 예쁘게 변환

        ### 전진속도와 회전속도 계산(엔코더 ver)
        self.transitional_velocity = self.radius_wheel * (self.rotate_right + self.rotate_left) / (2 * self.t_delta)
        self.rotational_velocity = self.radius_wheel * (self.rotate_right - self.rotate_left) / (2 * self.distance_btw_wheel)
              # IMU 버전일 때 x, y 계산 중 무조건 else로 가야 해서(w!=0으로) 주석 풀어놓음
        # print("v=", self.transitional_velocity)
        # print("w=", self.rotational_velocity)

        ### 전진속도와 회전속도 계산(IMU ver) - IMU에서는 회전속도 계산 안함. 대신 회전각 계산을 아래 코드에서 함
        # self.transitional_velocity = math.sqrt((self.linear_accel_x*self.t_delta)**2 + (self.linear_accel_y*self.t_delta)**2)
        # print("v=", self.transitional_velocity)

        ### 회전각 계산(IMU ver)
        # self.yaw += self.delta_theta * self.t_delta #yaw 값은 시작지점부터 누적되야 함
        # print("yaw", self.yaw)

        ### 회전반경 계산(엔코더 ver)
        self.theta = self.routes[-1][3] + self.rotational_velocity*self.t_delta
        print("theta=", self.theta)

        ### x, y 좌표값 계산
        if self.rotational_velocity==0:
            ### 회전 속도가 0일 때, runge-kutta integration
            print("w=0")
            x = self.routes[-1][1] + self.transitional_velocity*self.t_delta*math.cos(self.theta + (self.rotational_velocity*self.t_delta)/2)
            y = self.routes[-1][2] + self.transitional_velocity*self.t_delta*math.sin(self.theta + (self.rotational_velocity*self.t_delta)/2)
        else:
            ### 회전 속도가 0이 아닐 때, exact integration
            print("w!=0")
            ### 엔코더 버전
            x = self.routes[-1][1] + (self.transitional_velocity/self.rotational_velocity) * (math.sin(self.routes[-1][3]) - math.sin(self.theta))
            y = self.routes[-1][2] - (self.transitional_velocity/self.rotational_velocity) * (math.cos(self.routes[-1][3]) - math.cos(self.theta))
            ### imu 버전
            # x = self.routes[-1][1] + self.transitional_velocity * math.sin(self.yaw) * self.t_delta
            # y = self.routes[-1][2] + self.transitional_velocity * math.cos(self.yaw) * self.t_delta

        ### 계산해낸 현재 새 위치(IMU ver)
        # self.new_loc = [self.t_now, x, y, self.theta, self.transitional_velocity, self.yaw, self.t_delta]

        ### 계산해낸 현재 새 위치(엔코더 ver)
        self.new_loc = [self.t_now, x, y, self.theta, self.transitional_velocity, self.rotational_velocity, self.t_delta]
        
        ### 새 위치를 경로 목록에 추가하고 현재 위치 publish 하는 함수 호출
        self.routes.append(self.new_loc)
        self.location_pub()
        self.location_pub_rviz(x,y)
    

    ### 현재 위치를 publish 함
    def location_pub(self):
        loc = OdometricLocation()
        loc.t_now, loc.x, loc.y, loc.theta, loc.transitional_velocity, loc.rotational_velocity, loc.t_delta \
            = self.new_loc[0], self.new_loc[1], self.new_loc[2], self.new_loc[3], self.new_loc[4], self.new_loc[5], self.new_loc[6]
        self.odometric_loc_pub.publish(loc)

    def location_pub_rviz(self,x,y):
        cur_point = Point32()
        cur_point.x = x
        cur_point.y = y
        self.cur_points.points.append(cur_point)
        self.pub_c.publish(self.cur_points)

def main():
    rospy.init_node('OdometricLocalization', anonymous=False)
    rate = rospy.Rate(10)

    loc = OdometricLocalization()
    while not rospy.is_shutdown():
        loc.encoder_left_callback()
        loc.calc_location()
        rate.sleep()
    
    ### 누적된 위치 정보를 csv로 저장
    t = time.localtime(time.time()) #파일 제목에 현재 시간 포함하기 위해, 현재 시간 불러옴
    t_str = time.strftime('%m-%d %I:%M:%S %p', t) #현재 시간을 문자열로 예쁘게 변환
    table = np.array([['loc.t_now', 'loc.x', 'loc.y', 'loc.theta', 'loc.transitional_velocity', 'loc.rotational_velocity', 'loc.t_delta']])  #데이터 헤드
    route = np.asarray(loc.routes)  #전체 이동 경로
    table = np.append(table, route, axis=0)  #헤드 + 루트 데이터 합침
    np.savetxt("routes("+t_str+").csv", table, fmt='%s', delimiter=",") #파일 저장


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass
