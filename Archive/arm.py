import pybullet as p

# Start the PyBullet simulation
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

# Load the URDF model
robotStartPos = [0, 0, 1]
robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("robot.urdf.xml", robotStartPos, robotStartOrientation)

# Step the simulation
p.setRealTimeSimulation(1)

# Keep the simulation running until the user closes the window
while p.isConnected():
    p.getCameraImage(320, 200)
