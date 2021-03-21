import pybullet as p
import time
import pybullet_data
import numpy as np
comau_config_path= "./simple.urdf"

dt=1/240
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF(comau_config_path, useFixedBase=True)

# get rid of all the damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=0.1, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

qi = 0
t = 0
kp = -4e4
ki = -1e4
kd = 0*-4e1
qd = 1
e_prev = 0

q0 = p.getJointState(boxId, 1)[0]

log = open("log.csv", 'w')
log.write(f"{t}, {q0}\n")
while t <= 20:
    p.stepSimulation()
    q = p.getJointState(boxId, 1)[0]
    ep = q - qd
    e_diff = (ep - e_prev)/dt
    f = kp*ep + ki*qi + kd*e_diff
    qi += ep*dt
    e_prev = ep
    t+= dt
   # p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, controlMode=p.TORQUE_CONTROL, force=f)
    log.write(f"{t}, {q}, {f}\n")

  #  print(f"JOINT: {q}")
    #time.sleep(dt)

p.disconnect()

exec(open("./pendulum.py").read())