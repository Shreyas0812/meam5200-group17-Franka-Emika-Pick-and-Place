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
    print(arm.get_gripper_state())
    
    arm.exec_gripper_cmd(0.05, 50)
    
    print(arm.get_gripper_state())
    

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
    
    return angle

def drop_block():
    print(arm.get_gripper_state())
    
    arm.open_gripper()
    
    print(arm.get_gripper_state())

def get_block_world(q_current):
 
    '''detector = ObjectDetector()
    fk =  FK()'''
    H_ee_camera = detector.get_H_ee_camera()
    
    
    H_camera_block = detector.get_detections()
    
    
    ee_block = H_ee_camera @ H_camera_block[0][1]
    
    
    _, T0e = fk.forward(q_current)
    
    block_world = T0e @ ee_block
    
    
    return block_world

def move_to_static_view(q_current):
    pos = np.array(([1,0,0,0.5],
    			[0,-1,0,-0.2], 
    			[0,0,-1,0.5],
    			[0,0,0,1]))
    q_start,_,_, message = ik.inverse(pos, q_current, method='J_pseudo', alpha = 0.5) 
    arm.safe_move_to_position(q_start)
    return q_start



def pick_static(q_current):
    
    q_start = move_to_static_view(q_current)

    print("Check if static blocks are on the table")

    H_camera_block = detector.get_detections()
    T = 0
    q_now = q_start
    while H_camera_block != []:

        print("Opening Gripper")
        arm.open_gripper()

        print("Detect blocks")

        block_world = get_block_world(q_start)


        pos = np.array(([1,0,0],
	    			[0,-1,0], 
	    			[0,0,-1],
	    			[0,0,0]))
        block_pos = block_world[:,3]
        block_pos = block_pos.reshape(4,1)
        ee_goal = np.hstack((pos,block_pos))
        angle = rotation_matrix_to_angle_axis(block_world[:3,:3])

        ee_goal_align = ee_goal.copy()
        ee_goal_align[2][3] = 0.5

        print("Aligning the end effector")
        print(ee_goal_align)
        q_align,_,_, message = ik.inverse(ee_goal_align, q_start, method='J_pseudo', alpha = 0.5)
        q_align[-1] = angle

        arm.safe_move_to_position(q_align)

        print("Moving to Block")
        print(ee_goal)
        
        q_goal,_,_, message = ik.inverse(ee_goal, q_align, method='J_pseudo', alpha = 0.5)
        q_goal[-1] = q_goal[-1] + angle

        arm.safe_move_to_position(q_goal)

        print("Grabbing the block")
        grab_block()
        
        q_now = q_goal
        
        arm.safe_move_to_position(q_start)

        T+=1

        print("Moving to goal")
        pos = np.array(([0,-1,0,0.55],
                    [-1,0,0,0.2], 
                    [0,0,-1,T*0.2],
                    [0,0,0,1]))
        q_finish,_,_, message = ik.inverse(pos, q_start, method='J_pseudo', alpha = 0.5)

        arm.safe_move_to_position(q_finish)

        print("Dropping the block")
        drop_block()

        arm.safe_move_to_position(q_start)

        if T == 4:
            break
    return q_start



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
    

    ik = IK()
    
    q_start = pick_static(start_position)

    # print("Moving above blocks")
    # pos = np.array(([0,-1,0,0.6],
    # 			[-1,0,0,-0.2], 
    # 			[0,0,-1,0.5],
    # 			[0,0,0,1]))
    # q_start,_,_, message = ik.inverse(pos, start_position, method='J_pseudo', alpha = 0.5)
    
    
    arm.safe_move_to_position(q_start)

    print("Code Ends")

    #arm.safe_move_to_position(q_goal)
    # get the transform from camera to panda_end_effector
    
    # block_world = get_block_world(q_start)
    
    # print("Moving to block")

    # pos = np.array(([1,0,0],
    # 			[0,-1,0], 
    # 			[0,0,-1],
    # 			[0,0,0]))
    # block_pos = block_world[:,3]
    # block_pos = block_pos.reshape(4,1)
    # ee_goal = np.hstack((pos,block_pos))
    
    # q_goal,_,_, message = ik.inverse(ee_goal, q_start, method='J_pseudo', alpha = 0.5)
    
    # arm.safe_move_to_position(q_goal)

    # print("Grabbing the block")
    # grab_block()

    # print("Moving above blocks")
    # pos = np.array(([0,-1,0,0.6],
    # 			[-1,0,0,-0.2], 
    # 			[0,0,-1,0.5],
    # 			[0,0,0,1]))
    # q_above,_,_, message = ik.inverse(pos, q_goal, method='J_pseudo', alpha = 0.5)

    # arm.safe_move_to_position(q_above)

    # print("Moving to goal")
    # pos = np.array(([0,-1,0,0.55],
    # 			[-1,0,0,0.2], 
    # 			[0,0,-1,0.2],
    # 			[0,0,0,1]))
    # q_finish,_,_, message = ik.inverse(pos, q_above, method='J_pseudo', alpha = 0.5)

    # arm.safe_move_to_position(q_finish)

    # print("Dropping the block")
    # drop_block()
    
    # Detect some blocks...
    
    # Uncomment to get middle camera depth/rgb images
    # mid_depth = detector.get_mid_depth()
    # mid_rgb = detector.get_mid_rgb()

    # Move around...

    # END STUDENT CODE
