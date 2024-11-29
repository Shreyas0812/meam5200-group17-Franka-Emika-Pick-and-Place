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
    
    arm.exec_gripper_cmd(0.048, 52)
    
    # print(arm.get_gripper_state())

def drop_block():
    # print(arm.get_gripper_state())
    
    arm.exec_gripper_cmd(0.09, 10)
    
    # print(arm.get_gripper_state())

def rotation_matrix_to_angle_axis(R):
   
    assert R.shape == (3, 3)
    


    '''angle = np.arccos((np.trace(R) - 1) / 2)

    
   
    if np.isclose(angle, 0):
        return np.array([1, 0, 0]), 0  
    
    
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ])
    
    
    axis = axis / np.linalg.norm(axis)
    
    
    print("angle is: ", angle)
    print("axis is: ", axis)
   
    while angle > 2.897 or angle < -2.896:
        print("Adjusting the angle")
        if angle > 2.897:
            angle -= pi/2
        if angle < -2.896:
            angle +=pi/2
            
    angle = angle-pi/4'''
    
    
    axis = 0
    angsin = 0
    angcos = 0
    for i in range(3):
        if np.isclose(R[2,i], 1, 1e-04):
            axis = i
    if axis ==0:
        angcos = R[1,1]
        angsin = R[0,1]
    else:
        angcos = R[1,0]
        angsin = R[0,0]
        
        
        
    angle1 = np.arccos(angcos)
    angle2 = np.arccos(angsin)
    angle = angle1
    if angle1 > angle2:
        angle = angle2
    while angle > 2.897 or angle < -2.896:
        print("Adjusting the angle")
        if angle > 2.897:
            angle -= pi/2
        if angle < -2.896:
            angle +=pi/2
        
    


    
    return angle
    
    
    



def get_block_world(q_current):

    block_world = np.zeros((4, 4))

    H_ee_camera = detector.get_H_ee_camera()

    H_camera_block = detector.get_detections()

    if H_camera_block != []:
    

        ee_block = H_ee_camera @ H_camera_block[0][1]
    
        _, T0e = fk.forward(q_current)
    
        block_world = T0e @ ee_block

        return len(H_camera_block), block_world
    else:
        return len(H_camera_block), block_world



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
    print("block_world: ", block_world)
    angle = rotation_matrix_to_angle_axis(block_world[:3,:3])
    

    print("angle is: ", angle)
   


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

    numBlocksDetected, block_world = get_block_world(q_above_pickup)
    print("block world: ", block_world)

    num_original_block_detected = numBlocksDetected

    while numBlocksDetected:

        T = num_original_block_detected - numBlocksDetected

        print("Pickup Sequence")
        print("Opening Gripper")
        drop_block()  

        print("Moving to static block")

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

        arm.safe_move_to_position(q_above_pickup)

        numBlocksDetected, block_world = get_block_world(q_above_pickup)

    else:
        print("Pick Place Static Completed")

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
    

    start_position = np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866])
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

    fk = FK()
    ik = IK()

    # Task: Pick and Place the Static Blocks
    
    # Get the static positions above pickup to detect blocks and above place to place them
    q_above_pickup, q_above_drop = get_static_view(start_position, team)
    
    print("Moving above static block pickup")
    arm.safe_move_to_position(q_above_pickup)

    pick_place_static(q_above_pickup, q_above_drop, team)

    # get the transform from camera to panda_end_effector
    # H_ee_camera = detector.get_H_ee_camera()

    # Detect some blocks...
    # for (name, pose) in detector.get_detections():
    #      print(name,'\n',pose)

    # Uncomment to get middle camera depth/rgb images
    # mid_depth = detector.get_mid_depth()
    # mid_rgb = detector.get_mid_rgb()

    # Move around...

    # END STUDENT CODE
