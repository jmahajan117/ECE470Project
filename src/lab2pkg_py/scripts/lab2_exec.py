#!/usr/bin/env python
from helper import *

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

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
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
    move_arm(pub_cmd, loop_rate, Q[start_loc][start_height][0], 4.0, 4.0)
    
    gripper(pub_cmd, loop_rate, suction_on)
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
    # | Q[0][2][0] Q[1][2][1] Q[2][2][1] |   Above third block
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

    numPlayers = 0
    numCards = 0

    while(numPlayers == 0):
        input_string = raw_input("Welcome to Dealer Bot! Press the number corresponding to how many players (2-5), or 0 to quit> ")
        print("You entered " + input_string + ".\n")

        if (int(input_string) == 2):
            numPlayers = 2
        elif (int(input_string) == 3):
            numPlayers = 3
        elif (int(input_string) == 4):
            numPlayers = 4
        elif (int(input_string) == 5):
            numPlayers = 5
        elif (int(input_string) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Invalid Input, please try again... \n\n")
            continue

    while(numCards == 0):
        input_string = raw_input("Press the number corresponding to how many cards each player should get, or 0 to quit> ")
        print("You entered " + input_string + ".\n")

        if(int(input_string) == 1):
            numCards = 1
        elif (int(input_string) == 2):
            numCards = 2
        elif (int(input_string) == 3):
            numCards = 3
        elif (int(input_string) == 4):
            numCards = 4
        elif (int(input_string) == 5):
            numCards = 5
        elif (int(input_string) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Invalid Input, please try again... \n\n")
            continue




    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

    print ("Starting setup for " + str(numPlayers) + " players.\n")

    x = 150
    y = 45
    

    player_3 = [150, 200, 60]
    player_2 = [300, 100, 60]
    player_1 = [300, 0, 100]

    #move_arm(pub_command, loop_rate, home, 4, 4)

    z = 50
    thetas = lab_invk(x, y, z, 0)
    move_arm(pub_command, loop_rate, thetas, 4, 4)
    gripper(pub_command, loop_rate, suction_on)
    time.sleep(0.5)

    z = 37
    thetas = lab_invk(x, y, z, 0)
    time.sleep(1)
    move_arm(pub_command, loop_rate, thetas, 4, 4)

    
    
    # MOVE TO PLAYER 1
    thetas = lab_invk(player_1[0], player_1[1], player_1[2], 0)
    time.sleep(1)
    move_arm(pub_command, loop_rate, thetas, 4, 4)
    gripper(pub_command, loop_rate, suction_off)
    time.sleep(0.5)


    
    #SECOND CARD
    z = 34
    thetas = lab_invk(x, y, z, 0)
    time.sleep(1)
    move_arm(pub_command, loop_rate, thetas, 4, 4)
    gripper(pub_command, loop_rate, suction_on)
    time.sleep(0.5)


    z = 50
    thetas = lab_invk(x, y, z, 0)
    time.sleep(1)
    move_arm(pub_command, loop_rate, thetas, 4, 4)


    # MOVE TO PLAYER 2
    thetas = lab_invk(player_2[0], player_2[1], player_2[2], 0)
    time.sleep(1)
    move_arm(pub_command, loop_rate, thetas, 4, 4)
    gripper(pub_command, loop_rate, suction_off)
    time.sleep(0.5)

    
    # THIRD CARD
    z = 32
    thetas = lab_invk(x, y, z, 0)
    time.sleep(1)
    move_arm(pub_command, loop_rate, thetas, 4, 4)
    gripper(pub_command, loop_rate, suction_on)
    time.sleep(0.5)

    z = 50
    thetas = lab_invk(x, y, z, 0)
    time.sleep(1)
    move_arm(pub_command, loop_rate, thetas, 4, 4)

    # MOVE TO PLAYER 3
    thetas = lab_invk(player_3[0], player_3[1], player_3[2], 0)
    time.sleep(1)
    move_arm(pub_command, loop_rate, thetas, 4, 4)
    gripper(pub_command, loop_rate, suction_off)
    time.sleep(0.5)

    z = 50
    thetas = lab_invk(x, y, z, 0)
    time.sleep(1)
    move_arm(pub_command, loop_rate, thetas, 4, 4)


    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
