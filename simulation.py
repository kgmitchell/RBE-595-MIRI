import pybullet as p
import pybullet_data
import scipy
from scipy.spatial.transform import Rotation as R
import numpy as np
import time

def create_room():
    # Create floor
    floorId = p.loadURDF("plane.urdf", basePosition=[0, 0, 0])

    # Create walls
    #wall_thickness = 0.1
    #wall_height = 2.5

    # Left wall
    #leftWallId = p.loadURDF("cube.urdf", basePosition=[-4, 0, wall_height / 2], baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi / 2]), globalScaling= wall_thickness)

    # Right wall
    #rightWallId = p.loadURDF("cube.urdf", basePosition=[4, 0, wall_height / 2], baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi / 2]), globalScaling= wall_thickness)

    # Back wall
    #backWallId = p.loadURDF("cube.urdf", basePosition=[0, -4, wall_height / 2], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), globalScaling= wall_thickness)
    # Front wall
    #frontWallId = p.loadURDF("cube.urdf", basePosition=[0, 4, wall_height / 2], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), globalScaling= wall_thickness)

    #return floorId, leftWallId, rightWallId, backWallId, frontWallId
    return floorId

def create_hospital_environment(tumor_pose):
    # Create hospital room
    #floorId, leftWallId, rightWallId, backWallId, frontWallId = create_room()
    floorId = create_room()

    # Create yellow tumor sphere
    sphere_radius = 0.1
    tumorId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphere_radius)
    tumorVisualId = p.createVisualShape(p.GEOM_SPHERE, radius=sphere_radius, rgbaColor=[1, 1, 0, 1])
    tumorObjId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=tumorId, baseVisualShapeIndex=tumorVisualId, basePosition=tumor_pose)

    # Create red screen rectangle
    screen_size = [0.4,0.3,0.01]
    screenId = p.createCollisionShape(p.GEOM_BOX, halfExtents=screen_size)
    screenVisualId = p.createVisualShape(p.GEOM_BOX, halfExtents=screen_size, rgbaColor=[1, 0, 0, 1])
    screenObjId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=screenId, baseVisualShapeIndex=screenVisualId, basePosition=[0, 0, 0])

    # Create blue head sphere
    headId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphere_radius)
    headVisualId = p.createVisualShape(p.GEOM_SPHERE, radius=sphere_radius, rgbaColor=[0, 0, 1, 1])
    headObjId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=headId, baseVisualShapeIndex=headVisualId, basePosition=[0, 0, 0])

    #return floorId, leftWallId, rightWallId, backWallId, frontWallId, tumorObjId, screenObjId, headObjId
    return floorId, tumorObjId, screenObjId, headObjId


def update_line(line_id, start_point, end_point):
    p.addUserDebugLine(start_point, end_point, [0, 1, 0], 2, replaceItemUniqueId=line_id)





######################################################################################################
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

tumor_pose = [0, 0, 0.7]

#floorId, leftWallId, rightWallId, backWallId, frontWallId, tumorObjId, screenObjId, headObjId = create_hospital_environment()
floorId, tumorObjId, screenObjId, headObjId = create_hospital_environment(tumor_pose)

amplitude = 0.5
period = 2
t = 0

# Set the distance between head and screen
distance_between_head_and_screen = 1

line_id = p.addUserDebugLine([0, 0, 0], [0, 0, 0], [0, 1, 0], 2)  # Initial line with zero length

try:
    while True:
        
        # Step the simulation using sinusoidal motion of the head
        t += 1 / 240
        time.sleep(1 / 240)
        p.stepSimulation()

        x = 4*amplitude * np.sin(2 * np.pi * t / period)
        y = amplitude * np.cos(2 * np.pi * t / period)  # Added height variation
        headPos = [-0.8 + x, -2, 3 + y]  # Update height dynamically
        p.resetBasePositionAndOrientation(headObjId, headPos, [0, 0, 0, 1])

        # Calculate the tumor to head vector values
        th_vec = np.array(headPos) - tumor_pose
        th_mag = np.linalg.norm(th_vec)
        th_unit_vec = th_vec / th_mag

        # Calculate the screen position
        screenPos = th_unit_vec * (th_mag - distance_between_head_and_screen) + tumor_pose

        # Calculate screen orientation
        zx_hat = th_unit_vec[0]
        zy_hat = th_unit_vec[1]
        zz_hat = th_unit_vec[2]

        Z_prime_hat = [zx_hat, zy_hat, zz_hat]
        #print(Z_prime_hat)

        X_prime = [-zy_hat, zx_hat, 0]
        X_prime_mag = np.linalg.norm(X_prime)
        X_prime_hat = X_prime / X_prime_mag
        #print(X_prime_hat)

        Y_prime_hat = np.cross(Z_prime_hat, X_prime_hat)
        #print(Y_prime_hat)

        screen_orientation_matrix = R.from_matrix([[X_prime_hat[0], Y_prime_hat[0], Z_prime_hat[0]], 
                                                   [X_prime_hat[1], Y_prime_hat[1], Z_prime_hat[1]],
                                                   [X_prime_hat[2], Y_prime_hat[2], Z_prime_hat[2]]])

        #print(screen_orientation_matrix)

        screen_orientation_quat = screen_orientation_matrix.as_quat()
        #print(screen_orientation_quat)
        p.resetBasePositionAndOrientation(screenObjId, screenPos.tolist(), screen_orientation_quat.tolist())


        #tumorPos = p.getBasePositionAndOrientation(tumorObjId)[0]

        # Update the line's endpoints
        update_line(line_id, tumor_pose, headPos)

except KeyboardInterrupt:
    pass

p.disconnect()