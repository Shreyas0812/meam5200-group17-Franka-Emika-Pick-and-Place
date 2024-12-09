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
        self.q_above_pickup = None
        self.q_above_drop = None

        # Dynamic Block
        self.q_above_rotate = None

    # Move to given position via inverse kinematics
    # Note: This function is not used for now
    def calculate_q_via_ik(self, pos, q_start):
        q_end, rollout, success, message = self.ik.inverse(pos, q_start, method='J_pseudo', alpha = 0.5)
        
        if success:
            return q_end
        else:
            print('Failed to find IK Solution: ')
            print('pos: ', pos)
            print('q_start: ', q_start)
            return None

    def get_block_world(self, q_current):

        block_world = []

        H_ee_camera = detector.get_H_ee_camera()

        for (name, pose) in detector.get_detections():

            ee_block = H_ee_camera @ pose

            _, T0e = self.fk.forward(q_current)

            cur_block = T0e @ ee_block

            if cur_block[1, 3] < -0.07 and self.team == 'red':
                block_world.append(cur_block)
                # Sort the blocks based on the y position in reverse order
                block_world = sorted(block_world, key=lambda x: x[1,3], reverse=True)

            if cur_block[1, 3] > 0.07 and self.team == 'blue':
                block_world.append(cur_block)
                # Sort the blocks based on the y position
                block_world = sorted(block_world, key=lambda x: x[1,3])


        return len(block_world), block_world

    def drop_block(self):
        self.arm.exec_gripper_cmd(0.09, 10)
    
    def grab_block(self):
        self.arm.exec_gripper_cmd(0.048, 52)
    
    def rotation_matrix_to_angle_axis(self, R):

        assert R.shape == (3, 3)

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
        while angle > 2.897 or angle < -2.896:
            if angle > 2.897:
                angle -= pi/2
            if angle < -2.896:
                angle +=pi/2
    
        return angle

    def move_to_place(self, T):
        if self.team == 'red':
            place_location = np.array(([1,0,0, 0.562],
                        [0,-1,0, 0.2], 
                        [0,0,-1,0.22 + T*0.055],
                        [0,0,0,1]))
        else:
            place_location = np.array(([1,0,0, 0.562],
                        [0,-1,0, -0.2], 
                        [0,0,-1,0.22 + T*0.055],
                        [0,0,0,1]))

        q_place = self.calculate_q_via_ik(place_location, self.q_above_drop)

        return q_place


    def move_to_static_block(self, block, q_current):

        ee_rot = np.array(([1,0,0],
	    			[0,-1,0], 
	    			[0,0,-1],
	    			[0,0,0]))
        block_pos = block[:,3]
        block_pos = block_pos.reshape(4,1)
        ee_goal = np.hstack((ee_rot,block_pos))

        angle = self.rotation_matrix_to_angle_axis(block[:3,:3])       

        ee_align = deepcopy(ee_goal)
        ee_align[2, 3] = 0.4
        
        q_align = self.calculate_q_via_ik(ee_align, q_current)
        if q_align is not None:
            q_align[-1] = q_align[-1] - angle

        q_block = self.calculate_q_via_ik(ee_goal, q_align)
        if q_block is not None:
            q_block[-1] = q_block[-1] - angle 

        return q_align, q_block

    def pick_place_static(self):
        # Move to the above pickup position
        self.arm.safe_move_to_position(self.q_above_pickup)

        # Get the block world position
        block_count, block_world = self.get_block_world(self.q_above_pickup)
        
        # Open the gripper
        self.drop_block()

        q_paths = []

        # Calculate the align and goal position for each block along with the place position
        for i, block in enumerate(block_world):
            if i == 0:
                q_align, q_block = self.move_to_static_block(block, self.q_above_pickup)
            else:
                q_align, q_block = self.move_to_static_block(block, self.q_above_drop)
            
            q_place = self.move_to_place(i)

            q_paths.append([q_align, q_block, q_place])


        for q_align, q_block, q_place in q_paths: 
            # Pickup Sequence
            # Move to the align position
            self.arm.safe_move_to_position(q_align)

            # Move to the block
            self.arm.safe_move_to_position(q_block)

            # Close the gripper
            self.grab_block()

            # Move above the drop position
            self.arm.safe_move_to_position(self.q_above_drop)

            # Place Sequence
            # Move to place position
            self.arm.safe_move_to_position(q_place)

            # Open the gripper
            self.drop_block()

            # Move to the central position
            self.arm.safe_move_to_position(self.q_above_drop)

    # Staic Block Pick and Place
    def set_static_view(self, q_current):

        if self.team == 'red':
            pos_above_pickup = np.array(([1, 0, 0, 0.52 ],
                                         [0,-1, 0, -0.2 ], 
                                         [0, 0,-1, 0.43 ],
                                         [0, 0, 0, 1    ]))

            pos_above_drop = np.array(([1, 0, 0, 0.52 ],
                                       [0,-1, 0, 0.2  ], 
                                       [0, 0,-1, 0.6  ],
                                       [0, 0, 0, 1    ]))
        
        else:
            pos_above_pickup = np.array(([1, 0, 0, 0.52 ],
                                         [0,-1, 0, 0.2  ], 
                                         [0, 0,-1, 0.43 ],
                                         [0, 0, 0, 1    ]))
            
            pos_above_drop = np.array(([1, 0, 0, 0.52 ],
                                       [0,-1, 0,-0.2  ], 
                                       [0, 0,-1, 0.6  ],
                                       [0, 0, 0, 1    ]))

        self.q_above_pickup = self.calculate_q_via_ik(pos_above_pickup, q_current)

        self.q_above_drop = self.calculate_q_via_ik(pos_above_drop, q_current)

    # Dynamic Block Pick and Place
    def get_dynamic_block_view(self, q_current):

        if team == 'red':
            q_above_rotate = np.array(([1, 0, 0, 0   ],
                                       [0,-1, 0, 0.7 ],
                                       [0, 0,-1, 0.1 ],
                                       [0, 0, 0, 1   ]))
            
            
        else:
            q_above_rotate = np.array(([1, 0, 0, 0   ],
                                       [0,-1, 0, -0.7],
                                       [0, 0,-1, 0.1 ],
                                       [0, 0, 0, 1   ]))
        
        self.q_above_rotate = self.calculate_q_via_ik(q_above_rotate, q_current)

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

    start_time = time_in_seconds()
    print(start_time, " Starting Static Pick and Place")

    # Create the Pick_and_Place object
    pick_and_place = Pick_and_Place(team, arm, detector)

    pick_and_place.set_static_view(start_position)

    pick_and_place.pick_place_static()

    finish_time = time_in_seconds()
    print(finish_time, " Finished Static Pick and Place")

    run_time = finish_time - start_time
    print("Run Time: ", run_time)
    # pick_and_place.get_dynamic_block_view(start_position)

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