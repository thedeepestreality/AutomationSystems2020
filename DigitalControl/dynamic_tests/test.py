import pybullet as p
import time
import pybullet_data
import numpy as np
import math
comau_config_path= "./simple.urdf"

dt = 1/240
x0 = np.array([0, 0])
T = 6
RT = False
COMPARE = True

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF(comau_config_path, useFixedBase=True)

# get rid of all the damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=x0[1], controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

qi = 0
t = 0
kp = -10
ki = -8
kd = -4
qd = 1
e_prev = 0

q0 = p.getJointState(boxId, 1)[0]
m = 1
L = 0.8
k = 1
g = 10

gm = k/(m*L*L)
w = g/L

log = open("log.csv", 'w')
log.write(f"{t}, {q0}\n")
while t <= T:
    p.stepSimulation()
    q = p.getJointState(boxId, 1)[0]
    v = p.getJointState(boxId, 1)[1]
    ep = q - qd
    e_diff = (ep - e_prev)/dt
    f = kp*ep + ki*qi + kd*e_diff
    qi += ep*dt
    e_prev = ep
    t+= dt
    
    fd = w*math.sin(q)

    f = (fd + f)*m*L*L
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, controlMode=p.TORQUE_CONTROL, force=f)
    log.write(f"{t}, {q}, {v}, {f}\n")

    print(f"JOINT: {q}")
    if (RT):
        time.sleep(dt)

p.disconnect()
print(f"JOINT: {q}")
#exec(open("./pendulum.py").read())
if (COMPARE):
    import pendulum
    pendulum.compare(x0,T)