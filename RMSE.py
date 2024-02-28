import pybullet as p
import pybullet_data
from solver_agnes import Ik_solver_agnes
from scipy.spatial.transform import Rotation as R
import numpy as np
import time
import csv

# Existing functions and variables
def create_hospital_environment(tumor_pose):
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

    # # Create blue head sphere
    headId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphere_radius)
    headVisualId = p.createVisualShape(p.GEOM_SPHERE, radius=sphere_radius, rgbaColor=[0, 0, 1, 1])
    headObjId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=headId, baseVisualShapeIndex=headVisualId, basePosition=[0, 0, 0])

    # stick_figure_urdf_path = "stick.urdf.xml"
    # stick_figure_start_pos = [0, 0, 0]  # Adjust as needed
    # stick_figure_start_orientation = p.getQuaternionFromEuler([0, 0, 0])  # Adjust as needed
    # headObjId = p.loadURDF(stick_figure_urdf_path, stick_figure_start_pos, stick_figure_start_orientation)


    #fixed_base_position = [0, -0.35, 2.5]  # Adjust as needed
    #fake_base_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=-1,
    #                                basePosition=fixed_base_position, baseOrientation=[0, 0, 0, 1])

    # Load the robot URDF
    robot_urdf_path = "agnes.urdf.xml"  # Adjust as needed
    robotStartPos = [0, 0, 2.5]  # Adjust as needed
    #for step in range(0, 211, 30):
    #    print(step)
    robotStartOrn = p.getQuaternionFromEuler([0, (180*3.14159/180), (-90*3.14159/180)])  # Adjust as needed
    robotId = p.loadURDF(robot_urdf_path, robotStartPos, robotStartOrn, useFixedBase=1, globalScaling = 1)

    # Fix the robot base to the ground
    #p.createConstraint(parentBodyUniqueId=fake_base_id, parentLinkIndex=-1, childBodyUniqueId=robotId,
    #               childLinkIndex=-1, jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0],
    #               parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
    
    #screen_link_index = 5  # Adjust according to the URDF
    #p.createConstraint(parentBodyUniqueId=robotId, parentLinkIndex=screen_link_index, childBodyUniqueId=screenObjId,
    #                   childLinkIndex=-1, jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0],
    #                   parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
    
    return tumorObjId, screenObjId, headObjId, robotId

def update_line(line_id, start_point, end_point):
    p.addUserDebugLine(start_point, end_point, [0, 1, 0], 2, replaceItemUniqueId=line_id)

def euler_to_quaternion(euler):
    r = R.from_euler('xyz', euler, degrees=True)
    return r.as_quat()

# Simulation and RMSE calculation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

tumor_pose = [0, 0, 0.7]
tumorObjId, screenObjId, headObjId, robotId = create_hospital_environment(tumor_pose)

# Load head tracking data from CSV file
head_tracking_data = []
scaling_factor = 0.001  # Scaling factor for converting mm to meters
with open('head_tracking.csv', 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Skip header row
    for row in reader:
        rx, ry, rz, tx, ty, tz = map(float, row)
        head_tracking_data.append((tx * scaling_factor, ty * scaling_factor, tz * scaling_factor, rx, ry, rz))

with open('head_tracking.csv', 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Skip header row
    for row in reader:
        rx, ry, rz, tx, ty, tz = map(float, row)
        head_tracking_data.append((tx * scaling_factor, ty * scaling_factor, tz * scaling_factor, rx, ry, rz))

with open('head_tracking.csv', 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Skip header row
    for row in reader:
        rx, ry, rz, tx, ty, tz = map(float, row)
        head_tracking_data.append((tx * scaling_factor, ty * scaling_factor, tz * scaling_factor, rx, ry, rz))

distance_between_head_and_screen = 0.9
line_id = p.addUserDebugLine([0, 0, 0], [0, 0, 0], [0, 1, 0], 2)

# Set camera parameters
p.resetDebugVisualizerCamera(cameraDistance=4, cameraYaw=140, cameraPitch=-30, cameraTargetPosition=[0, -1, 0])

# Initialize simulated data list
simulated_data = []
temp = 0
try:
    for tx, ty, tz, rx, ry, rz in head_tracking_data:
        #while(True):
        #    temp = temp + 1
        headPos = [tx, ty, tz]
        p.resetBasePositionAndOrientation(headObjId, headPos, euler_to_quaternion([rx, ry, rz]))

        th_vec = np.array(headPos) - tumor_pose
        th_mag = np.linalg.norm(th_vec)
        th_unit_vec = th_vec / th_mag
        screenPos = th_unit_vec * (th_mag - distance_between_head_and_screen) + tumor_pose

        zx_hat, zy_hat, zz_hat = th_unit_vec
        Z_prime_hat = [zx_hat, zy_hat, zz_hat]

        X_prime = [-zy_hat, zx_hat, 0]
        X_prime_mag = np.linalg.norm(X_prime)
        X_prime_hat = X_prime / X_prime_mag

        Y_prime_hat = np.cross(Z_prime_hat, X_prime_hat)

        screen_orientation_matrix = R.from_matrix([
            [X_prime_hat[0], Y_prime_hat[0], Z_prime_hat[0]],
            [X_prime_hat[1], Y_prime_hat[1], Z_prime_hat[1]],
            [X_prime_hat[2], Y_prime_hat[2], Z_prime_hat[2]]
        ])

        screen_orientation_quat = screen_orientation_matrix.as_quat()
        p.resetBasePositionAndOrientation(screenObjId, screenPos.tolist(), screen_orientation_quat.tolist())

        update_line(line_id, tumor_pose, headPos)

        o = screenPos
        rpy = screen_orientation_matrix.as_euler('xyz', degrees=True)
        my_solver = Ik_solver_agnes(0.485, 1.0, 0.74, 0.254374022)
        q = my_solver.solve(o, rpy, -1)
        p.setJointMotorControlArray(robotId, range(6), p.POSITION_CONTROL, targetPositions = q)
        q_aux, rpy_aux = my_solver._forward_kinematics(q)

        time.sleep(1 / 240)
        p.stepSimulation()



        # Append simulated data to list
        simulated_data.append([tx, ty, tz])  # Replace with your actual simulated data

except KeyboardInterrupt:
    pass

p.disconnect()

# Convert simulated data and head tracking data to numpy arrays
simulated_data = np.array(simulated_data)
head_tracking_data = np.array(head_tracking_data)

# Extract position data from head tracking data
head_tracking_position = head_tracking_data[:, :3]

# Check if the arrays are not empty before performing operations
if simulated_data.size != 0 and head_tracking_position.size != 0:
    # Calculate RMSE for each dimension (X, Y, Z)
    rmse_x = np.sqrt(np.mean((simulated_data[:, 0] - head_tracking_position[:, 0]) ** 2))
    rmse_y = np.sqrt(np.mean((simulated_data[:, 1] - head_tracking_position[:, 1]) ** 2))
    rmse_z = np.sqrt(np.mean((simulated_data[:, 2] - head_tracking_position[:, 2]) ** 2))

    # Calculate overall RMSE
    overall_rmse = np.sqrt(np.mean((simulated_data - head_tracking_position) ** 2))

    print("Root Mean Square Error (RMSE) for X dimension:", rmse_x)
    print("Root Mean Square Error (RMSE) for Y dimension:", rmse_y)
    print("Root Mean Square Error (RMSE) for Z dimension:", rmse_z)
    print("Overall Root Mean Square Error (RMSE):", overall_rmse)
else:
    print("Error: Simulated data or head tracking data is empty.")

