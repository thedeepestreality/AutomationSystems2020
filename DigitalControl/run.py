import pybullet as p
import time
import pybullet_data
urdf_path = "robot.urdf"

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF(urdf_path, useFixedBase=True)
while True:
    p.stepSimulation()
p.disconnect()