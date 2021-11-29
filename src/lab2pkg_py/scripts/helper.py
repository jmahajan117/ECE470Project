#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from math import pi as PI

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	
	
	x = -150 + 540
	y = 150 + 120 - 93 + 83  + 82 + 59
	z = 10 + 152 + 53.5

	# 
	M = [[0, -1, 0, x], [0, 0, -1, y], [1, 0, 0, z], [0, 0, 0, 1]]
	w1 = np.matrix([0, 0, 1]).T
	w2 = np.matrix([0, 1, 0]).T
	w3 = np.matrix([0, 1, 0]).T
	w4 = np.matrix([0, 1, 0]).T
	w5 = np.matrix([1, 0, 0]).T
	w6 = np.matrix([0, 1, 0]).T

	#print(np.shape(w1))
	#print(w1)
	W = [w1, w2, w3, w4, w5, w6]

	q1 = np.matrix([ -150, 150, 10]).T
	q2 = np.matrix([-150, 150 + 120, 10 + 152]).T
	q3 = np.matrix([q2[0, 0] + 244, q2[1, 0], q2[2, 0]]).T
	q4 = np.matrix([q3[0, 0] + 213, q3[1, 0] - 93, q3[2, 0]]).T
	q5 = np.matrix([q4[0, 0], q4[1, 0] + 83, q4[2, 0]]).T
	q6 = np.matrix([q5[0, 0] + 83, q5[1, 0], q5[2, 0]]).T

	#print(np.shape(q4))

	v1 = np.cross(-w1, q1, axis = 0)
	v2 = np.cross(-w2, q2, axis = 0)
	v3 = np.cross(-w3, q3, axis = 0)
	v4 = np.cross(-w4, q4, axis = 0)
	v5 = np.cross(-w5, q5, axis = 0)
	v6 = np.cross(-w6, q6, axis = 0)

	#print(v1)

	V = [v1, v2, v3, v4, v5, v6]


	S = []
	
	# S matrixes need to be 4x4
	for i in range(6):
		skew = []
		for j in range(4):
			add = []
			if j == 0:
				add = [0, -1 * W[i][2][0], W[i][1][0], V[i][0][0]]
			elif j == 1:
				add = [W[i][2][0], 0, -1 * W[i][0][0], V[i][1][0]]
			elif j == 2:
				add = [-1 * W[i][1][0], W[i][0][0], 0, V[i][2][0]]
			elif j == 3:
				add = [0, 0, 0, 0]
			skew.append(add)
		S.append(np.matrix(skew))
	
	# ==============================================================#
	return np.matrix(M), S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	M, S = Get_MS()
	
	T = np.matmul(expm(S[0] * theta1), expm(S[1] * theta2))
	T = np.matmul(T, expm(S[2] * theta3))
	T = np.matmul(T, expm(S[3] * theta4))
	T = np.matmul(T, expm(S[4] * theta5))
	T = np.matmul(T, expm(S[5] * theta6))
	T = np.matmul(T, M)

	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	# xcen = np.cos(yaw_WgripDegree)*53.5
	# xcen = xWgrip - xcen

	# ycen = np.sin(yaw_WgripDegree)*53.5
	# ycen = yWgrip - ycen
	
	# zcen = zWgrip

	# theta1 = np.arctan(ycen/xcen) - np.arcsin(110/np.sqrt(xcen**2 + ycen**2))
	# theta6 = theta1+90-yaw_WgripDegree
	
	# l7 = 83

	# # x3end1 = (l7 + 27) * np.sin(theta1)
	# #y3end1 = l7 * np.cos(theta1)

	# # y3end1 = np.tan(np.radians(90) - theta1) * x3end1

	# #y3end2 = l7 * np.cos(np.radians(90) - theta1)
	# #y3end2 = np.sin(np.arccos(x3end/83)) * 83

	# # x3end = xcen- x3end1
	# # y3end = ycen- y3end1 - y3end2
	# z3end = zcen + 83 + 59


	# x1 = 83 * np.cos(theta1)
	# x2 = 110 * np.sin(theta1)
	# y1 = 110 * np.cos(theta1)
	# y2 = 83 * np.sin(theta1)

	# y3end = ycen - y1 - y2
	# x3end = xcen - x1 + x2

	l3 = 244
	l5 = 213
	l1 = 152

	l2 = 120
	l4 = 93
	l6 = 83
	l7 = 83
	l8 = 82
	l9 = 53.5
	l10 = 59

	# specialz = z3end - l1
	# c2 = np.sqrt(x3end**2+y3end**2)/specialz**2

	# theta5 = np.radians(-90)
	# theta3 = 180 - np.arccos((c2-l3**2-l5**2)/(2*l3*l5))
	# theta2 = np.arctan(specialz/(np.sqrt(x3end**2+y3end**2))) + np.arccos((l5**2 - l3**2 - c2)/(2*l3*np.sqrt(c2)))
	# theta4 = 180 - (180 - theta3 + theta2)

	xcen = xWgrip - 53.5*(np.cos(np.radians((yaw_WgripDegree))))+150
	ycen = yWgrip - 53.5*(np.sin(np.radians((yaw_WgripDegree))))-150
	zcen = zWgrip - 10

	theta1 = np.arctan2(ycen,xcen)-(np.arcsin((l2-l4+l6)/np.sqrt(xcen**2+ycen**2)))

	x3end = xcen - (l7*np.cos(theta1)) + ((l6+27)*np.sin(theta1))
	y3end = ycen - ((l6+27)*np.cos(theta1)) - (l7*np.sin(theta1))
	z3end = zcen + l10 + l8

	# print(PI)
	csqared = ((x3end**2)+(y3end**2))+((z3end-l1)**2)
	c = np.sqrt(csqared)
	theta2 = (-1*np.arccos(((l5**2)-csqared-(l3**2))/(-2*l3*c))) - np.arcsin((z3end-l1)/c)
	theta3 = PI - np.arccos(((l3**2)+(l5**2)-csqared)/(2*l3*l5))
	theta4 = -(PI-(PI-theta3-(theta2)))
	theta5 = -PI/2
	theta6 = PI-(PI/2-theta1)-np.radians(yaw_WgripDegree)
	
	return lab_fk(float(theta1),float(theta2),float(theta3),float(theta4),float(theta5),float(theta6))
	# lab_fk(10,-25,35,-45,-90,-10)


# ```
# xcen = xWgrip - 53.5*(np.cos(np.radians((yaw_WgripDegree))))+150
# ycen = yWgrip - 53.5*(np.sin(np.radians((yaw_WgripDegree))))-150
# zcen = zWgrip - 10

# theta1 = np.arctan2(ycen,xcen)-(np.arcsin((l2-l4+l6)/np.sqrt(xcen**2+ycen**2)))

# x3end = xcen - (l7*np.cos(theta1)) + ((l6+27)*np.sin(theta1))
# y3end = ycen - ((l6+27)*np.cos(theta1)) - (l7*np.sin(theta1))
# z3end = zcen + l10 + l8

# csqared = ((x3end**2)+(y3end**2))+((z3end-l1)**2)
# c = np.sqrt(csqared)
# theta2 = (-1*np.arccos(((l5**2)-csqared-(l3**2))/(-2*l3*c))) - np.arcsin((z3end-l1)/c)
# theta3 = PI - np.arccos(((l3**2)+(l5**2)-csqared)/(2*l3*l5))
# theta4 = -(PI-(PI-theta3-(theta2)))
# theta5 = -PI/2
# theta6 = PI-(PI/2-theta1)-np.radians(yaw_WgripDegree) ```

# 	# ==============================================================#
# 	pass
   

