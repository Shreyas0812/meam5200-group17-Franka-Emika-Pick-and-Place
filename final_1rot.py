import sys
import numpy as np
from copy import deepcopy
from math import pi
import lib.IK_position_null 
from lib.IK_position_null import IK
from lib.rrt_t1 import rrt
from lib.calculateFK import FK
from scipy.spatial.transform import Rotation as R

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds
import time




if __name__ == "__main__":
    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")

    arm = ArmController()
    detector = ObjectDetector()


    def get_block_world(q_current):
  
        H_ee_camera = detector.get_H_ee_camera()
    
        H_camera_block = detector.get_detections()  
        
        # Validate H_camera_block
        if not H_camera_block or len(H_camera_block) == 0:
            rospy.logerr("H_camera_block is empty. No blocks detected.")
            raise ValueError("H_camera_block is empty.")

        if len(H_camera_block[0]) < 2:
            rospy.logerr("H_camera_block does not contain the expected data structure.")
            raise ValueError("H_camera_block has an invalid structure.")

        ee_block = H_ee_camera @ H_camera_block[0][1]
        _, T0e = fk.forward(q_current)  
        block_world = T0e @ ee_block
    
        return block_world

    def grab_block():
        print(arm.get_gripper_state())
        arm.exec_gripper_cmd(0.05, 46)
        print(arm.get_gripper_state())

    def drop_block():
        print(arm.get_gripper_state())
        arm.open_gripper()  
        print(arm.get_gripper_state())
        
        
    def normalize_rotation_matrix(rotation_mat):
        corrected_rotation_mat = np.zeros_like(rotation_mat)
        for i in range(3):
            corrected_rotation_mat[i, :] = rotation_mat[i, :] / np.linalg.norm(rotation_mat[i, :])
        return corrected_rotation_mat

    def validate_rotation_matrix(rotation_mat):
    # Check determinant
        det = np.linalg.det(rotation_mat)
        if not np.isclose(det, 1.0):
            print(f"Error: Determinant is not 1. Determinant = {det}")
            return False

    # Check orthogonality: R^T * R = I
        orthogonality = np.allclose(np.dot(rotation_mat.T, rotation_mat), np.eye(3))
        if not orthogonality:
            print("Error: Rotation matrix is not orthogonal. Attempting normalization...")
            # If orthogonality fails, attempt to normalize the matrix
            rotation_mat = normalize_rotation_matrix(rotation_mat)
            # After normalization, check orthogonality again
            if np.allclose(np.dot(rotation_mat.T, rotation_mat), np.eye(3)):
                print("Normalized rotation matrix is now orthogonal.")
            else:
                print("Error: Normalization failed. The matrix is still not orthogonal.")
                return False

        print("Rotation matrix is valid.")
        return True
     
        
    # Define the starting position for the robot
    start_position = np.array([-0.01779206, -0.76012354, 0.01978261, -2.34205014, 0.02984053, 1.54119353 + pi / 2, 0.75344866])
    arm.safe_move_to_position(start_position)
    rospy.loginfo("Moved to the starting position.")

    # Define the two target positions as transformation matrices
    pos_1 = np.array([
    	 [1,0,0,0.5],
    	 [0,-1,0,-0.2], 
    	 [0,0,-1,0.5],
    	 [0,0,0,1]
    	 ])
    
    
    #ee_goal = np.hstack((pos,block_pos))
    
    
    #pos_2 = np.array([
    #    [0, -1, 0, 0.562],
    #    [-1, 0, 0, -0.2],
    #    [0, 0, -1, 0.15],
    #    [0, 0, 0, 1]
    #])
    
    
    pos_3 = np.array([
        [0, -1, 0, 0.562],
        [-1, 0, 0, 0.2],
        [0, 0, -1, 0.25],
        [0, 0, 0, 1]
    ])

    # Initialize the inverse kinematics solver
    ik = IK()
    fk = FK()

    # Solve IK for the first position (Block Viewing Position "B")
    q1, _, success, _ = ik.inverse(pos_1, start_position, method="J_pseudo", alpha=0.5)
    if not success:
        rospy.logerr("Failed to compute IK for position 1. Exiting.")
        exit(1)

    rospy.loginfo("IK for position 1 succeeded. Moving to position 1.")
    print(q1)
    arm.safe_move_to_position(q1)
    
   
    print("Opening Arm Gripper...")
    drop_block() 
    
    
    ## Rotation
    
    block_world = get_block_world(q1)
    ##
    rotation_mat = block_world[:3, :3]
    
    if validate_rotation_matrix(rotation_mat):
        r = R.from_matrix(rotation_mat)
        euler_angles = r.as_euler('xyz', degrees=True)  # [roll, pitch, yaw]
        print("Rotation Matrix:\n", rotation_mat)
        print("Euler Angles (degrees):", euler_angles)

        # Extract the translation and construct pos_2
        block_pos = block_world[:3, 3]
        pos_2 = np.eye(4)
        pos_2[:3, :3] = rotation_mat
        pos_2[:3, :3] = normalize_rotation_matrix(pos_2[:3, :3])
        pos_2[:3, 3] = block_pos
        #pos_2[:3, :3] = np.eye(3)  # Replace rotation matrix with identity

        
        print("Desired Pose (Homogeneous Transformation Matrix):\n", pos_2)
    else:
        print("Invalid rotation matrix. Aborting further computations.")
    

    #pos = np.array(([1,0,0],
	#    			[0,-1,0], 
	 #   			[0,0,-1],
	  #  			[0,0,0]))
    #block_pos = block_world[:,3]
    #block_pos = block_pos.reshape(4,1)
    #pos_2 = np.hstack((pos,block_pos))
    #print("block world: ", block_world)
    #print("pos_2: ", pos_2)
    
    #arm.safe_move_to_position(pos_2)
    

    #pos_2 = block_world[:,3]
    #pos_2 = pos_2.reshape(4,1)
    

    # Solve IK for the second position
    q2, _, success, _ = ik.inverse(pos_2, q1, method="J_pseudo", alpha=0.5)
    if not success:
        rospy.logerr("Failed to compute IK for position 2. Exiting.")
        exit(1)

    rospy.loginfo("IK for position 2 succeeded. Moving to position 2.")
    
    #--------------------------

    
    #------------------------------
    arm.safe_move_to_position(q2)
    
    
    print("Grabbing the block")
    grab_block()
    
    # Copy pos_2 to a new matrix pos_a
    #pos_a = pos_2.copy()

    # Increment the z translation by 0.06
    #pos_a[2, 3] += 0.07
    #pos_a = np.array([
    #    [0, -1, 0, 0.48634463],
    #   [-1, 0, 0, -0.09353536],
    #   [0, 0, -1, 0.28538065],
    #    [0, 0, 0, 1]
    #])
     
    # Solve IK for the above position
    #qa, _, success, _ = ik.inverse(pos_a, q2, method="J_pseudo", alpha=0.5)
    #if not success:
    #    rospy.logerr("Failed to compute IK for position a. Exiting.")
    #    exit(1)

   # rospy.loginfo("IK for position a succeeded. Moving to position a.")
   # arm.safe_move_to_position(qa)
   
   
   # Bring back to the default position "B"
    arm.safe_move_to_position(q1)
        
        
    # Solve IK for the third position
    q3, _, success, _ = ik.inverse(pos_3, q1, method="J_pseudo", alpha=0.5)
    if not success:
        rospy.logerr("Failed to compute IK for position 3. Planning path to place block using RRT...")
        exit(1)
        
        
    # Plan path using RRT / Can input direct q3 with ik
    path = rrt(q1, q3)
    if len(path) == 0:
        rospy.logerr("RRT failed to find a path from position 1 to position 2. Exiting.")
        exit(1)

    rospy.loginfo("Path planning succeeded. Executing the planned path.")

    # Execute the planned path
    for i, config in enumerate(path):
        rospy.loginfo(f"Moving to configuration {i + 1}/{len(path)}.")
        arm.safe_move_to_position(config)

    rospy.loginfo("Successfully executed the path from position 1 to position 2.")
    
    print("Dropping Block...")
    drop_block() 
