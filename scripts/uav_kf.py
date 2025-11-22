#!/usr/bin/env python3
import time
import numpy as np
from filterpy.kalman import KalmanFilter as KF
## ROS part
import rospy
from geometry_msgs.msg import PoseStamped
## filter analysis
from numpy.linalg import inv
from math import sqrt
# import matplotlib.pyplot as plt # 用来画方差和残差

'''
Feature:
1. 测试natnet包到filter包的延迟
2. 用natnet的时间——物理量出来那一刻的时间、而不用当前时间
3. 测试filter耗时
4. 两个kf对比
新：
5. 用方差和残差定量分析滤波器效果
6. 退出rosnode开始分析数据
'''

class filterTuning():
	
	def __init__(self):
		rospy.loginfo('uav_kf initializing')

		self.device_name = rospy.get_param('~device_name', 'jackQuad')
		self.input_sub = rospy.Subscriber("/vrpn_client_node/"+self.device_name+"/pose", PoseStamped, self.input_callback)
		self.output_pub = rospy.Publisher("/uav_kf", PoseStamped, queue_size = 1)
		self.zRes, self.zP, self.zs, self.xs, self.v = [], [], [], [], [] # for eval
		self.lastT, self.lastZ = None, None
		self.filter = KF(dim_x=6, dim_z=3, dim_u=0)
		self.filter.P *= 10.
		self.filter.R *= 0.1
		self.filter.H = np.concatenate((np.diag([1., 1., 1.]), np.zeros((3,3))), axis=1) 
		self.filter.Q = np.diag([0.001, 0.001, 0.001, 10, 10, 10]) # 速度的过程噪声大，因为更新方法不对
		# print('输出方程H', self.filter.H.shape, '\n', self.filter.H)
		# print('观测误差R', self.filter.R.shape, '\n', self.filter.R)
		# print('协方差矩阵P', self.filter.P.shape, '\n', self.filter.P)
		# print('过程噪声Q', self.filter.Q.shape, '\n', self.filter.Q)
		# print(self.filter.x.shape)
		
	def input_callback(self, data):
		timeBefore = time.time()
		pos = np.array([data.pose.position.x, -data.pose.position.y, -data.pose.position.z])
		## dt
		SendTime = data.header.stamp.to_time()
		if self.lastT is None:
			self.lastT, self.lastZ = SendTime, pos # 用真实物理量发生的时间 SendTime
			self.filter.x = np.concatenate((pos, np.zeros((3,))), axis=0)
			print('uav_kf初值', self.filter.x.shape, self.filter.x)
			return
		dt = SendTime - self.lastT
		## dealbreaker
		if (abs(pos[2] - self.lastZ[2]) > 7 * dt or
			abs(pos[0] - self.lastZ[0]) > 3 * dt or
			abs(pos[1] - self.lastZ[1]) > 3* dt):
			return
		self.lastT, self.lastZ = SendTime, pos
		## predict
		self.filter.F[0:3, 3:6] = dt * np.eye(3)
		self.filter.predict()
		## update
		self.filter.update(pos)
		## pub
		data = PoseStamped()
		data.header.stamp = rospy.Time.from_sec(SendTime) # keep input timestamp
		data.pose.position.x = self.filter.x[0]
		data.pose.position.y = self.filter.x[1]
		data.pose.position.z = self.filter.x[2]
		data.pose.orientation.x = self.filter.x[3]
		data.pose.orientation.y = self.filter.x[4]
		data.pose.orientation.z = self.filter.x[5]
		data.pose.orientation.w = dt
		self.output_pub.publish(data)
		timeAfter = time.time()
		# print("kf"+str(self.num), 'processing time%6.2fms' % ((timeAfter - timeBefore) * 1000))
		## eval residual
		y, S = self.filter.y, self.filter.S
		eps = np.dot(y.T, inv(S)).dot(y)
		self.v.append(self.filter.x[5])
		self.zs.append(pos[2])
		self.xs.append(self.filter.x[2])
		self.zRes.append(self.filter.y[2])
		self.zP.append(self.filter.P[2,2])

if __name__ == '__main__':
	rospy.init_node("uavKf", anonymous=True)

	kf0 = filterTuning()

	rospy.spin()


