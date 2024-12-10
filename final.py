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

#Student imports
from lib.IK_position_null import IK
from lib.calculateFK import FK

##################################################--HELPER FUNCTIONS--##################################################

def calculate_q_via_ik(pos, q_start, verify=True):
    q_end, rollout, success, message = ik.inverse(pos, q_start, method='J_pseudo', alpha = 0.5)
    
    if verify:
        if success:
            return q_end
        else:
            print('Failed to find IK Solution: ')
            print('pos: ', pos)
            print('q_start: ', q_start)
            return q_start
    
    return q_end

##################################################--STATIC--##################################################

def pick_place_static(q_above_pickup, q_above_drop, stack_block_num=4):
    pass

##################################################--DYNAMIC--##################################################

def pick_place_dynamic(q_above_rotate, q_above_drop_stacked, stack_block_num=4):
    pass

##################################################--SETUP FUNCTIONS--##################################################

def set_static_view(q_current):

    if team == 'red':
        pos_above_pickup = np.array(([1, 0, 0, 0.52 ],
                                        [0,-1, 0, -0.2 ], 
                                        [0, 0,-1, 0.52 ], 
                                        [0, 0, 0, 1    ]))

        pos_above_drop = np.array(([1, 0, 0, 0.52 ],
                                    [0,-1, 0, 0.2  ], 
                                    [0, 0,-1, 0.45  ],
                                    [0, 0, 0, 1    ]))
    
    else:
        pos_above_pickup = np.array(([1, 0, 0, 0.52 ],
                                        [0,-1, 0, 0.2  ], 
                                        [0, 0,-1, 0.52 ],
                                        [0, 0, 0, 1    ]))
        
        pos_above_drop = np.array(([1, 0, 0, 0.52 ],
                                    [0,-1, 0,-0.2  ], 
                                    [0, 0,-1, 0.45  ],
                                    [0, 0, 0, 1    ]))

    q_above_pickup = calculate_q_via_ik(pos_above_pickup, q_current)

    if (q_above_pickup == q_current).all():
        q_above_pickup = calculate_q_via_ik(pos_above_pickup, q_current)


    q_above_drop = calculate_q_via_ik(pos_above_drop, q_current)

    if (q_above_drop == q_current).all():
        q_above_drop = calculate_q_via_ik(pos_above_pickup, q_current)

    return q_above_pickup, q_above_drop


def set_dynamic_block_view(q_current):

    if team == 'red':
        pos_above_rotate = np.array(([1, 0, 0, 0   ],
                                    [0,-1, 0, 0.7 ],
                                    [0, 0,-1, 0.4 ],
                                    [0, 0, 0, 1   ]))
        
        pos_above_drop_stacked = np.array(([1, 0, 0, 0.52 ],
                                    [0,-1, 0, 0.2  ], 
                                    [0, 0,-1, 0.65  ],
                                    [0, 0, 0, 1    ]))
    
    else:
        pos_above_rotate = np.array(([1, 0, 0, 0   ],
                                    [0,-1, 0, -0.7],
                                    [0, 0,-1, 0.4 ],
                                    [0, 0, 0, 1   ]))
        
        pos_above_drop_stacked = np.array(([1, 0, 0, 0.52 ],
                                    [0,-1, 0,-0.2  ], 
                                    [0, 0,-1, 0.65  ],
                                    [0, 0, 0, 1    ]))
    
    q_above_rotate = calculate_q_via_ik(pos_above_rotate, q_current, verify=True)
    if (q_above_rotate == q_current).all():
        q_above_rotate = calculate_q_via_ik(pos_above_rotate, q_current)
        
    if team == "red":
        if q_above_rotate[-1] - pi < 2.897 and q_above_rotate[-1] - pi > -2.897:
            q_above_rotate[-1] = q_above_rotate[-1] - pi
        else:
            q_above_rotate[-1] = q_above_rotate[-1] + pi

    q_above_drop_stacked = calculate_q_via_ik(pos_above_drop_stacked, q_current)
    if (q_above_drop_stacked == q_current).all():
        q_above_drop_stacked = calculate_q_via_ik(q_above_drop_stacked, q_current)

    return q_above_rotate, q_above_drop_stacked

##################################################--MAIN--##################################################

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

    if team == 'red':
        emergency_qs = {
            'q_above_pickup': np.array([-1.83376634e-01, 7.90806995e-03, -1.84837988e-01, -1.56523269e+00, 1.45339161e-03,  1.57300605e+00, 4.17185979e-01]),
            'q_above_drop': np.array([0.17320456, -0.01933834, 0.19194961, -1.77723386, 0.00375479, 1.75824931, 1.14981758]),
            'q_above_rotate': np.array([-0.82870465, -0.93406695, 1.72730474, -1.14483025, 0.92926407, 1.43886509, -1.18035082]),
            'q_above_drop_stacked': np.array([0.32585352, 0.25697452, 0.05841297, -0.86094054, -0.01650469, 1.1175494, 1.160523]),
            
            'q_dynamic_pickup': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866]),

            'q_place_1': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866]),
            'q_place_2': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866]),
            'q_place_3': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866]),
            'q_place_4': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866]),
            'q_place_5': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866]),
            'q_place_6': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866]),
            'q_place_7': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866]),
            'q_place_8': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866])
        }
    else:
        emergency_qs = {
            'q_above_pickup': np.array([ 1.87884314e-01, 7.90148892e-03, 1.80304215e-01, -1.56523270e+00, -1.41695423e-03, 1.57300610e+00, 1.15358432e+00]),
            'q_above_drop': np.array([-0.18474463, -0.01929637, -0.18053245, -1.77723404, -0.00352629, 1.75825005, 0.42081113]),
            'q_above_rotate': np.array([ 0.84098714, -0.87723034, -1.81075025, -1.1518686, -0.84836095, 1.47756159, -0.41517303]),
            'q_above_drop_stacked': np.array([-0.24049283, 0.25966636, -0.17955265, -0.86067122, 0.05104353, 1.11685786, 0.3936485]),
            
            'q_dynamic_pickup': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866]),

            'q_place_1': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866]),
            'q_place_2': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866]),
            'q_place_3': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866]),
            'q_place_4': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866]),
            'q_place_5': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866]),
            'q_place_6': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866]),
            'q_place_7': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866]),
            'q_place_8': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866])
        }

    start_time = time_in_seconds()

    # Variables
    team = team
    ik = IK()
    fk = FK()

    red_blue_black_box_height = 0.2

    scaling_factor = 20

    ddrop_ee_dist = 0.09
    drop_ee_force = 10
    
    grab_ee_dist = 0.048
    grab_ee_force = 52

    # Static Pick and Place
    q_above_pickup, q_above_drop = set_static_view(start_position)

    if q_above_pickup is None:
        q_above_pickup = emergency_qs['q_above_pickup']
    
    if q_above_drop is None:
        q_above_drop = emergency_qs['q_above_drop']

    # Dynamic Pick and Place
    q_above_rotate, q_above_drop_stacked = set_dynamic_block_view(start_position)

    if q_above_rotate is None:
        q_above_rotate = emergency_qs['q_above_rotate']
    
    if q_above_drop_stacked is None:
        q_above_drop_stacked = emergency_qs['q_above_drop_stacked']
        
    ####################################################################################################
    static_start_time = time_in_seconds()
    pick_place_static(q_above_pickup, q_above_drop, stack_block_num=4)
    static_end_time = time_in_seconds()

    print("Time taken for static pick and place: ", static_end_time - static_start_time, " seconds")
    ####################################################################################################

    dynamic_strat_time = time_in_seconds()
    pick_place_dynamic(q_above_rotate, q_above_drop_stacked, stack_block_num=4)
    dynamic_end_time = time_in_seconds()

    print("Time taken for dynamic pick and place: ", dynamic_end_time - dynamic_strat_time, " seconds")
    ####################################################################################################

    end_time = time_in_seconds()
    print(f"Run Time: {end_time - start_time:.2f} seconds")

    # END STUDENT CODE