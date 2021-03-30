import pybullet as p
import time
import pybullet_data
import numpy as np
import math
import sys
from camera import Camera
from PIL import Image as im
comau_config_path= "./simple.urdf"

dt = 1/240

def traj(t, q_end):
    q_start = 0
    vel_start = 0
    vel_end = 0
    q_end = 1
    ts = 1

    b0 = q_start
    db0 = vel_start
    b1 = q_end
    db1 = vel_end

    a0 = b0
    a1 = db0
    a2 = (3*b1 - 3*b0 - 2*db0*ts - db1*ts)/(ts**2)
    a3 = (2*b0 + (db0 + db1)*ts - 2*b1)/(ts**3)

    q = a0 + a1*t + a2*t**2 + a3*t**3
    dq = a1 + 2*a2*t + 3*a3*t**2
    ddq = 2*a2 + 6*a3*t

    return (q, dq, ddq)
        # Or you can apend q to some list
        # and return list at the end


x0 = np.array([0, 0])
T = 100
RT = True
COMPARE = False

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF(comau_config_path, useFixedBase=True)

p.changeVisualShape(boxId, 3, rgbaColor=[0,0,0,1])
# get rid of all the damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

camera = Camera()
data = camera.get_frame()
img = im.fromarray(data, 'RGBA')            
img.save('my.png')

p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=x0[1], controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

qi = 0
t = 0
kp = -4000
kd = -100
ki = -40000
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
    if (t <= 1):
        qd, dqd, ddqd = traj(t,1)
    else:
        qd = 1
        dqd = 0
        ddqd = 0
    ep = q - qd
    # e_diff = (ep - e_prev)/dt
    e_diff = v - dqd
    f = kp*ep + ki*qi + kd*e_diff
    qi += ep*dt
    e_prev = ep
    t+= dt
    
    fd = w*math.sin(q)

    f = (fd + f + dqd)*m*L*L
    # print(f"NUM: {p.getNumJoints(boxId)}")
    # M = p.calculateMassMatrix(boxId,[0,1,0])
    # print(f"MASS: {M} {m*L*L}")
    # f= p.calculateInverseDynamics(boxId,[0,0,0],[0,0,0],[0,0.1,0])
 
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, controlMode=p.TORQUE_CONTROL, force=f)
    log.write(f"{t}, {q}, {v}, {f}, {qd}, {dqd}\n")

    # # print(f"JOINT: {q}")
    if (RT):
        time.sleep(dt)

p.disconnect()
print(f"JOINT: {q}")
#exec(open("./pendulum.py").read())
if (COMPARE):
    import pendulum
    pendulum.compare(x0,T)