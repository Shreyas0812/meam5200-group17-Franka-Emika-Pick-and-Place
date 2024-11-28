import sys
import numpy as np
from copy import deepcopy
from math import pi
from lib.rrt import rrt

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds
from lib.IK_position_null import IK
from lib.calculateFK import FK


def grab_block():
    # print(arm.get_gripper_state())
    
    arm.exec_gripper_cmd(0.05, 50)
    
    # print(arm.get_gripper_state())

def drop_block():
    # print(arm.get_gripper_state())
    
    arm.exec_gripper_cmd(0.07, 10)
    
    # print(arm.get_gripper_state())

def rotation_matrix_to_angle_axis(R):
   
    assert R.shape == (3, 3)
    
    
    angle = np.arccos((np.trace(R) - 1) / 2)
    
   
    if np.isclose(angle, 0):
        return np.array([1, 0, 0]), 0  
    
    
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ])
    
    
    axis = axis / np.linalg.norm(axis)
    
    return angle, axis



def get_block_world(q_current):
    '''detector = ObjectDetector()
    fk =  FK()'''
    H_ee_camera = detector.get_H_ee_camera()
    
    
    H_camera_block = detector.get_detections()
    
    
    ee_block = H_ee_camera @ H_camera_block[0][1]
    
    
    _, T0e = fk.forward(q_current)
    
    block_world = T0e @ ee_block
    
    
    return block_world

def move_to_place(q_align, T, team):
    if team == 'red':
        place_location = np.array(([1,0,0, 0.562],
	    			[0,-1,0, 0.2], 
	    			[0,0,-1,0.22 + T*0.055],
	    			[0,0,0,1]))
    else:
        place_location = np.array(([1,0,0, 0.562],
	    			[0,-1,0, -0.2], 
	    			[0,0,-1,0.22 + T*0.055],
	    			[0,0,0,1]))
        
    q_place,_,_, message = ik.inverse(place_location, q_align, method='J_pseudo', alpha = 0.5)

    return q_place

def move_to_static(block_world, q_current):
    
    ee_rot = np.array(([1,0,0],
	    			[0,-1,0], 
	    			[0,0,-1],
	    			[0,0,0]))
    block_pos = block_world[:,3]
    block_pos = block_pos.reshape(4,1)
    ee_goal = np.hstack((ee_rot,block_pos))

    angle, axis = rotation_matrix_to_angle_axis(block_world[:3,:3])
    
    print("angle is: ", angle)
    print("axis is: ", axis)
   
    while angle > 2.897 or angle < -2.896:
        print("Adjusting the angle")
        if angle > 2.897:
            angle -= pi/2
        if angle < -2.896:
            angle +=pi/2

    angle = angle-pi/4

    '''
    for i in range(3):
        if np.isclose(block_world[2,i], 1, 0.001) or np.isclose(block_world[2,i], 1, 0.001):
    '''

    #axis-angle start

    print("Calculating Aligning the end effector")
    
    ee_align = ee_goal.copy()
    ee_align[2, 3] = 0.4

    q_align,_,_, message = ik.inverse(ee_align, q_current, method='J_pseudo', alpha = 0.5)   
    q_align[-1] = angle

    print("Calculating Goal of end effector")
    q_goal,_,_, message = ik.inverse(ee_goal, q_align, method='J_pseudo', alpha = 0.5)
    q_goal[-1] = angle

    return q_align, q_goal


def pick_place_static(q_above_pickup, q_above_drop, team):
    print("Check if static blocks are on the table")

    T = 0
    q_above_pickup

    q_now = q_above_pickup

    while detector.get_detections() != []:

        print("Pickup Sequence")
        print("Opening Gripper")
        drop_block()  

        print("Moving to static block")

        block_world = get_block_world(q_above_pickup)
        print("block world: ", block_world)

        q_align, q_goal = move_to_static(block_world, q_now)

        print("Moving to the correct alignment")
        arm.safe_move_to_position(q_align)

        print("Moving to the block")
        arm.safe_move_to_position(q_goal)

        print("Grabbing the block")
        grab_block()

        # print("Move above block")
        # arm.safe_move_to_position(q_align)

        print("Move above drop")
        arm.safe_move_to_position(q_above_drop)

        # print("Place Sequence")
        # q_place = move_to_place(q_align, T, team)

        print("Place Sequence")
        q_place = move_to_place(q_goal, T, team)

        print("Go to place position")
        arm.safe_move_to_position(q_place)
        
        print("Drop the block")
        drop_block()

        print("Move above drop location")
        arm.safe_move_to_position(q_above_drop)

        q_now = q_above_pickup

        T+=1

def get_static_view(q_current, team):

    if team == 'red':
        pos_above_pickup = np.array(([1,0,0,0.52],
                    [0,-1,0,-0.2], 
                    [0,0,-1,0.43],
                    [0,0,0,1]))

        pos_above_drop = np.array(([1,0,0,0.52],
                    [0,-1,0,0.2], 
                    [0,0,-1,0.6],
                    [0,0,0,1]))
    else:
        pos_above_pickup = np.array(([1,0,0,0.52],
                    [0,-1,0,0.2], 
                    [0,0,-1,0.43],
                    [0,0,0,1]))
        
        pos_above_drop = np.array(([1,0,0,0.52],
                    [0,-1,0,-0.2], 
                    [0,0,-1,0.6],
                    [0,0,0,1]))
        
    q_above_pickup,_,_, message = ik.inverse(pos_above_pickup, q_current, method='J_pseudo', alpha = 0.5)

    q_above_drop,_,_, message = ik.inverse(pos_above_drop, q_current, method='J_pseudo', alpha = 0.5)

    return q_above_pickup, q_above_drop

if __name__ == "__main__":

    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")

    arm = ArmController()
    detector = ObjectDetector()
    fk =  FK()
    ik = IK()
    start_position = np.array([0,0,0,-pi/2, 0,pi/2, pi/4])
    arm.safe_move_to_position(start_position) # on your mark!
    
    

    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
    else:
        print("**  RED TEAM  **")
    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!

    # STUDENT CODE HERE   

    # Task: Pick and Place the Static Blocks
    
    q_above_pickup, q_above_drop = get_static_view(start_position, team)
    
    print("Moving above static block pickup")
    arm.safe_move_to_position(q_above_pickup)

    pick_place_static(q_above_pickup, q_above_drop, team)

    # Move back to the start position
    # arm.safe_move_to_position(start_position)

    # Detect some blocks...
    
    # Uncomment to get middle camera depth/rgb images
    # mid_depth = detector.get_mid_depth()
    # mid_rgb = detector.get_mid_rgb()

    # Move around...

    # END STUDENT CODE
