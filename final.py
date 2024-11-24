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


def get_block_world(q_current):
  
    '''detector = ObjectDetector()
    fk =  FK()'''
    H_ee_camera = detector.get_H_ee_camera()
    
    
    H_camera_block = detector.get_detections()
    
    
    ee_block = H_ee_camera @ H_camera_block[1][1]
    
    
    _, T0e = fk.forward(q_current)
    
    block_world = T0e @ ee_block
    
    
    return block_world



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
    ik = IK()
    
    pos = np.array(([0,-1,0,0.6],
    			[-1,0,0,-0.2], 
    			[0,0,-1,0.5],
    			[0,0,0,1]))
    q_start,_,_, message = ik.inverse(pos, start_position, method='J_pseudo', alpha = 0.5)
    
    
    arm.safe_move_to_position(q_start)
    #arm.safe_move_to_position(q_goal)
    # get the transform from camera to panda_end_effector
    
    block_world = get_block_world(q_start)
    pos = np.array(([0,-1,0],
    			[-1,0,0], 
    			[0,0,-1],
    			[0,0,0]))
    block_pos = block_world[:,3]
    block_pos = block_pos.reshape(4,1)
    ee_goal = np.hstack((pos,block_pos))
    
    
    q_goal,_,_, message = ik.inverse(ee_goal, q_start, method='J_pseudo', alpha = 0.5)
    
    arm.safe_move_to_position(q_goal)
    # Detect some blocks...
    for (name, pose) in detector.get_detections():
         print(name,'\n',pose)

    # Uncomment to get middle camera depth/rgb images
    mid_depth = detector.get_mid_depth()
    mid_rgb = detector.get_mid_rgb()

    # Move around...

    # END STUDENT CODE
