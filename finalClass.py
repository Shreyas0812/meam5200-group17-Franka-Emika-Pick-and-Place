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

class Pick_and_Place:
    def __init__(self, team, arm, detector):
        self.team = team
        self.arm = arm
        self.detector = detector
        self.fk = FK()
        self.ik = IK()

        self.current_pose = None

        # Static Block
        self.q_above_pickup_drop = None

        # Dynamic Block
        self.q_above_rotate = None
        self.q_above_drop = None

    # Staic Block Pick and Place
    def get_static_view(self, q_current):

        if self.team == 'red':
            pos_above_pickup_drop = np.array(([1, 0, 0, 0.52],
                                         [0,-1, 0, 0   ], 
                                         [0, 0,-1, 0.6 ],
                                         [0, 0, 0, 1   ]))
                                                    
            
        else:
            pos_above_pickup_drop = np.array(([1, 0, 0, 0.52],
                                         [0,-1, 0, 0.2 ], 
                                         [0, 0,-1, 0.43],
                                         [0, 0, 0, 1   ]))
            
        q_above_pickup_drop, rollout, success, message = self.ik.inverse(pos_above_pickup_drop, q_current, method='J_pseudo', alpha = 0.5)

        if success:
            self.q_above_pickup_drop = q_above_pickup_drop
        else:
            print('Failed to find IK solution for q_above_pickup_drop')
            print(message)
            print('Rollout:', rollout)

    def get_block_world(self, q_current):

        block_world = []

        H_ee_camera = detector.get_H_ee_camera()

        for (name, pose) in detector.get_detections():

            ee_block = H_ee_camera @ pose

            _, T0e = self.fk.forward(q_current)

            cur_block = T0e @ ee_block

            print(cur_block)
            print(cur_block[1,3])

            block_world.append(T0e @ ee_block)
        
        for block in block_world:
            print(block)
            print()

        return len(block_world), block_world
        

    def pick_place_static(self):
        # Move to the above pickup position
        self.arm.safe_move_to_position(self.q_above_pickup_drop)

        # Get the block world position
        block_count, block_world = self.get_block_world(self.q_above_pickup_drop)


    # Dynamic Block Pick and Place
    def get_dynamic_block_view(self, q_current):

        if team == 'red':
            q_above_rotate = np.array(([1, 0, 0, 0   ],
                                       [0,-1, 0, 0.7 ],
                                       [0, 0,-1, 0.1 ],
                                       [0, 0, 0, 1   ]))
            
            pos_above_drop = np.array(([1, 0, 0, 0.52],
                                       [0,-1, 0, 0.2 ], 
                                       [0, 0,-1, 0.6 ],
                                       [0, 0, 0, 1   ]))
        else:
            q_above_rotate = np.array(([1, 0, 0, 0   ],
                                       [0,-1, 0, -0.7],
                                       [0, 0,-1, 0.1 ],
                                       [0, 0, 0, 1   ]))

            pos_above_drop = np.array(([1, 0, 0, 0.52],
                                       [0,-1, 0,-0.2 ], 
                                       [0, 0,-1, 0.6 ],
                                       [0, 0, 0, 1   ]))
        
        q_above_rotate, rollout, success, message = self.ik.inverse(q_above_rotate, q_current, method='J_pseudo', alpha = 0.5)

        if success:
            self.q_above_rotate = q_above_rotate
        else:
            print('Failed to find IK solution for q_above_rotate')
            print(message)
            print('Rollout:', rollout)
        
        q_above_drop, rollout,success, message = self.ik.inverse(pos_above_drop, q_current, method='J_pseudo', alpha = 0.5)

        if success:
            self.q_above_drop = q_above_drop
        else:
            print('Failed to find IK solution for q_above_drop')
            print(message)
            print('Rollout:', rollout)

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

    # Create the Pick_and_Place object
    pick_and_place = Pick_and_Place(team, arm, detector)

    pick_and_place.get_static_view(start_position)

    pick_and_place.pick_place_static()

    pick_and_place.get_dynamic_block_view(start_position)

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