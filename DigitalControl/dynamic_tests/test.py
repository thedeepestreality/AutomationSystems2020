import pybullet as p
import time
import pybullet_data
comau_config_path= "./test.urdf"
#comau_config_path="~/kr10.urdf"

dt=1/240
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF(comau_config_path, useFixedBase=True)
while True:
    p.stepSimulation()
    time.sleep(dt)
p.disconnect()
