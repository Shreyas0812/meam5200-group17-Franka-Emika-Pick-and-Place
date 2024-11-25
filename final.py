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
    
    arm.exec_gripper_cmd(0.05, 46)
    
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
    
    return angle, axis

def drop_block():
    print(arm.get_gripper_state())
    
    arm.open_gripper()
    
    print(arm.get_gripper_state())

def get_block_world(q_current, T):
  
    '''detector = ObjectDetector()
    fk =  FK()'''
    H_ee_camera = detector.get_H_ee_camera()
    
    
    H_camera_block = detector.get_detections()
    
    
    ee_block = H_ee_camera @ H_camera_block[T][1]
    
    
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

def move_to_static(q_current,T):
    block_world = get_block_world(q_current,T)
    pos = np.array(([1,0,0],
	    			[0,-1,0], 
	    			[0,0,-1],
	    			[0,0,0]))
    block_pos = block_world[:,3]
    block_pos = block_pos.reshape(4,1)
    ee_goal = np.hstack((pos,block_pos))
    angle, axis = rotation_matrix_to_angle_axis(block_world[:3,:3])
        
    print("angle is: ", angle)
    print("axis is: ", axis)
    q_goal,_,_, message = ik.inverse(ee_goal, q_current, method='J_pseudo', alpha = 0.5)
    circles = angle/(2*pi)
    while angle > 2.897 or angle < -2.896:
        print("adjusting the angle")
        if angle > 2.897:
            angle -= pi/4
        if angle < -2.896:
            angle +=pi/4
        
        
        
        
    q_goal[-1] = angle
    arm.safe_move_to_position(q_goal)
    return q_goal

def pick_place_static(q_current):
    
    q_start = move_to_static_view(q_current)
    H_camera_block = detector.get_detections()
    T = 0
    q_now = q_start
    while H_camera_block != []:
        
        drop_block()  
        q_goal = move_to_static(q_start, T)
        
        grab_block()
        
        
        pos = np.array(([1,0,0, 0.562],
	    			[0,-1,0, 0.2], 
	    			[0,0,-1,0.2 + T*0.05],
	    			[0,0,0,1])) 
        
        arm.safe_move_to_position(q_start)
        
        q_place,_,_, message = ik.inverse(pos, q_start, method='J_pseudo', alpha = 0.5)
        
        arm.safe_move_to_position(q_place)
        
        drop_block()
        arm.safe_move_to_position(q_start)
        
        T+=1
        if T == 5:
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
    q_start = pick_place_static(start_position)
   
    
    ''' 
    block_world = get_block_world(q_start)
    pos = np.array(([1,0,0],
    			[0,-1,0], 
    			[0,0,-1],
    			[0,0,0]))
    block_pos = block_world[:,3]
    block_pos = block_pos.reshape(4,1)
    ee_goal = np.hstack((pos,block_pos))
    
    
    q_goal,_,_, message = ik.inverse(ee_goal, q_start, method='J_pseudo', alpha = 0.5)
    
    arm.safe_move_to_position(q_goal)'''
    # Detect some blocks...
    
    # Uncomment to get middle camera depth/rgb images
    # mid_depth = detector.get_mid_depth()
    # mid_rgb = detector.get_mid_rgb()

    # Move around...

    # END STUDENT CODE
