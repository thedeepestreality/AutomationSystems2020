import pybullet as p
import time
import pybullet_data
urdf_path = "robot.urdf"

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF(urdf_path, useFixedBase=True)

print("JOINT")
for _id in range(p.getNumJoints(robotId)):
    print(f'{_id} {p.getJointInfo(robotId, _id)[12]}')

jointIndices = [1, 2]
home_pos = [0.78, 0.78]

p.setJointMotorControlArray(robotId,
                            jointIndices=jointIndices,
                            controlMode=p.POSITION_CONTROL,
                            targetPositions=home_pos)

while True:
    p.stepSimulation()
p.disconnect()