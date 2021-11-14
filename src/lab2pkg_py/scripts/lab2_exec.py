#!/usr/bin/env python

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from lab2_header import *
from scipy.linalg import expm

PI = 3.1415926535

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

Q = None

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def getStateSuction(msg):
    global suction_state
    suction_state = msg.DIGIN




############### Your Code End Here ###############
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

	xcen = xWgrip - 53.5*(np.cos(np.radians((yaw_WgripDegree))))+150
	ycen = yWgrip - 53.5*(np.sin(np.radians((yaw_WgripDegree))))-150
	zcen = zWgrip

	theta1 = np.arctan2(ycen,xcen)-(np.arcsin((l2-l4+l6)/np.sqrt(xcen**2+ycen**2)))

	x3end = xcen - (l7*np.cos(theta1)) + ((l6+27)*np.sin(theta1))
	y3end = ycen - ((l6+27)*np.cos(theta1)) - (l7*np.sin(theta1))
	z3end = zcen + l10 + l8

	csqared = ((x3end**2)+(y3end**2))+((z3end-l1)**2)
	c = np.sqrt(csqared)
	theta2 = (-1*np.arccos(((l5**2)-csqared-(l3**2))/(-2*l3*c))) - np.arcsin((z3end-l1)/c)
	theta3 = PI - np.arccos(((l3**2)+(l5**2)-csqared)/(2*l3*l5))
	theta4 = -(PI-(PI-theta3-(theta2)))
	theta5 = -PI/2
	theta6 = PI-(PI/2-theta1)-np.radians(yaw_WgripDegree)
	
	return [float(theta1),float(theta2),float(theta3),float(theta4),float(theta5),float(theta6)]

"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    #print(lab_fk(dest[0],dest[1],dest[2],dest[3],dest[4],dest[5]))

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, end_loc, end_height, thetas):
    global Q

    ### Hint: Use the Q array to map out your towers by location and "height".

    # 2D layers (top view)

    # Layer (Above blocks)
    # | Q[0][2][1] Q[1][2][1] Q[2][2][1] |   Above third block
    # | Q[0][1][1] Q[1][1][1] Q[2][1][1] |   Above point of second block
    # | Q[0][0][1] Q[1][0][1] Q[2][0][1] |   Above point of bottom block

    # Layer (Gripping blocks)
    # | Q[0][2][0] Q[1][2][0] Q[2][2][0] |   Contact point of third block
    # | Q[0][1][0] Q[1][1][0] Q[2][1][0] |   Contact point of second block
    # | Q[0][0][0] Q[1][0][0] Q[2][0][0] |   Contact point of bottom block

    move_arm(pub_cmd, loop_rate, Q[start_loc][start_height][1], 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, thetas, 4.0, 4.0)
    time.sleep(0.5)
    val = suction_state
    if val == False:
        print("Unable to grip")
        gripper(pub_cmd, loop_rate, suction_off)
        exit(-1)


    move_arm(pub_cmd, loop_rate, Q[start_loc][start_height][1], 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, Q[end_loc][end_height][1], 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, Q[end_loc][end_height][0], 4.0, 4.0)

    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(0.5)

    move_arm(pub_cmd, loop_rate, Q[end_loc][end_height][1], 4.0, 4.0)


    error = 0



    return error


############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Parser
    parser = argparse.ArgumentParser(description='Please specify if using simulator or real robot')
    parser.add_argument('--simulator', type=str, default='True')
    args = parser.parse_args()

    # Definition of our tower

    # 2D layers (top view)

    # Layer (Above blocks)
    # | Q[0][2][1] Q[1][2][1] Q[2][2][1] |   Above third block
    # | Q[0][1][1] Q[1][1][1] Q[2][1][1] |   Above point of second block
    # | Q[0][0][1] Q[1][0][1] Q[2][0][1] |   Above point of bottom block

    # Layer (Gripping blocks)
    # | Q[0][2][0] Q[1][2][0] Q[2][2][0] |   Contact point of third block
    # | Q[0][1][0] Q[1][1][0] Q[2][1][0] |   Contact point of second block
    # | Q[0][0][0] Q[1][0][0] Q[2][0][0] |   Contact point of bottom block

    # First index - From left to right position A, B, C
    # Second index - From "bottom" to "top" position 1, 2, 3
    # Third index - From gripper contact point to "in the air" point

    # How the arm will move (Suggestions)
    # 1. Go to the "above (start) block" position from its base position
    # 2. Drop to the "contact (start) block" position
    # 3. Rise back to the "above (start) block" position
    # 4. Move to the destination "above (end) block" position
    # 5. Drop to the corresponding "contact (end) block" position
    # 6. Rise back to the "above (end) block" position

    # Initialize rospack
    rospack = rospkg.RosPack()
    # Get path to yaml
    lab2_path = rospack.get_path('lab2pkg_py')
    yamlpath = os.path.join(lab2_path, 'scripts', 'lab2_data.yaml')

    with open(yamlpath, 'r') as f:
        try:
            # Load the data as a dict
            data = yaml.load(f)
            if args.simulator.lower() == 'true':
                Q = data['sim_pos']
            elif args.simulator.lower() == 'false':
                Q = data['real_pos']
            else:
                print("Invalid simulator argument, enter True or False")
                sys.exit()
            
        except:
            print("YAML not found")
            sys.exit()

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function
    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, getStateSuction)




    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    input_done = 0
    loop_count = 0

    while(not input_done):
        input_string = raw_input("Enter number of loops <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_string + "\n")

        if(int(input_string) == 1):
            loop_count = 1
        elif (int(input_string) == 2):
            loop_count = 2
        elif (int(input_string) == 3):
            loop_count = 3
        elif (int(input_string) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")
            continue

        input_str = raw_input("Start Loc (0, 1, 2): ")
        print("You entered " + input_str + "\n")

        if(int(input_str) == 0):
            start = 0
        elif (int(input_str) == 1):
            start = 1
        elif (int(input_str) == 2):
            start = 2
        else:
            print("Error try again")
            continue


        input_str = raw_input("End loc (0, 1, 2): ")
        print("You entered " + input_str + "\n")

        if(int(input_str) == 0):
            end = 0
            input_done = 1
        elif (int(input_str) == 1):
            end = 1
            input_done = 1
        elif (int(input_str) == 2):
            end = 2
            input_done = 1
        else:
            print("Error Try again")
            continue





    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

    if start + end == 3:
        middle = 0
    if start + end == 2:
        middle = 1
    if start + end == 1:
        middle = 2
    
    thetas = []

    while(loop_count > 0):
        thetas = lab_invk(-510,258,530,0)
        move_block(pub_command, loop_rate, start, 2, end, 0,thetas)
        thetas = lab_invk(-510,258,278,0)
        move_block(pub_command, loop_rate, start, 2, start, 0, thetas)
        # move_block(pub_command, loop_rate, end, 2, middle, 1)
        # move_block(pub_command, loop_rate, start, 2, end, 2)
        # move_block(pub_command, loop_rate, middle, 1, start, 2)
        # move_block(pub_command, loop_rate, middle, 2, end, 1)
        # move_block(pub_command, loop_rate, start, 2, end, 0)

        loop_count = loop_count - 1

    gripper(pub_command, loop_rate, suction_off)



    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
