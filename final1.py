import sys
import numpy as np
from copy import deepcopy
from math import pi

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

from lib.IK_position_null import IK
from lib.calculateFK import FK

def IK_follow_path(pathList, q_cur, q_start):
    
    for path in pathList:
        #Using pseudo-inverse
        q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(path, q_cur, method='J_pseudo', alpha=.5)

        arm.safe_move_to_position(q_pseudo)
        
        q_cur = q_pseudo

        arm.safe_move_to_position(q_start)
    		    
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
        # Rewrite this 
        start_box = np.array(([0,-1,0,0.6],
    			[-1,0,0,-0.2], 
    			[0,0,-1,0.5],
    			[0,0,0,1]))
        
        target_box = np.array([
            [0,-1,0,0.562],
            [-1,0,0,0.2],
            [0,0,-1,.25],
            [0,0,0, 1],
        ])
    else:
        print("**  RED TEAM  **")
        start_box = np.array(([0,-1,0,0.6],
    			[-1,0,0,-0.2], 
    			[0,0,-1,0.5],
    			[0,0,0,1]))
        
        target_box = np.array([
            [0,-1,0,0.562],
            [-1,0,0,0.2],
            [0,0,-1,.25],
            [0,0,0, 1],
        ])

    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!

    # STUDENT CODE HERE

    # Initialize class objects
    ik = IK()
    fk = FK()

    # Go above start position
    q_start,_,_, message = ik.inverse(start_box, start_position, method='J_pseudo', alpha = 0.5)

    # get the transform from camera to panda_end_effector
    H_ee_camera = detector.get_H_ee_camera()

    # Detect some blocks...
    for (name, pose) in detector.get_detections():
        print(name,'\n',pose)

    # Uncomment to get middle camera depth/rgb images
    mid_depth = detector.get_mid_depth()
    mid_rgb = detector.get_mid_rgb()
    
    print("mid_depth", mid_depth)
    print("mid_rgb", mid_rgb)
    
    pos = np.array(([0,-1,0,0.6],
    			[-1,0,0,-0.2], 
    			[0,0,-1,0.5],
    			[0,0,0,1]))
    q_start,_,_, message = ik.inverse(pos, start_position, method='J_pseudo', alpha = 0.5)
    
    target = np.array([
        [0,-1,0,0.562],
        [-1,0,0,-0.2],
        [0,0,-1,.25],
        [0,0,0, 1],
    ])

    # Using pseudo-inverse
    #q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target, start_position, method='J_pseudo', alpha=.5)

    #arm.safe_move_to_position(q_pseudo)

    black_target = np.array([
        [0,-1,0,0.562],
        [-1,0,0,-0.2],
        [0,0,-1,.25],
        [0,0,0, 1],
        ])
    
    red_target = np.array((
        [0,-1,0,0.562],
        [-1,0,0,0.2],
        [0,0,-1,.25],
        [0,0,0, 1],
    ))

    position_list = []
    position_list.append(black_target)
    position_list.append(red_target)
    
    # Move around...
    IK_follow_path(position_list, start_position, start_position)
    
    # END STUDENT CODE