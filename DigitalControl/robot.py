import pybullet as p
import pybullet_data
import asyncio
from logger import Logger
import modern_robotics as mr

URDF_PATH = "robot.urdf"
PRINT_JOINTS = True

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)

plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF(URDF_PATH, useFixedBase=True)

joint_indices = [1, 2]
home_pos = [0,0]
dt = 1/240
eef_link_idx = 3

# p.setJointMotorControlArray(robot_id, jointIndices=joint_indices, 
#                                     controlMode=p.VELOCITY_CONTROL,
#                                     forces=(0,0))

# log format: "ts, j1, j2"

if PRINT_JOINTS:
    print("Available joints:")
    for joint in range(p.getNumJoints(robot_id)):
        jointInfo = p.getJointInfo(robot_id, joint)
        print(f'{joint} name: {jointInfo[1]}')
        print(f'{joint} maxVel: {jointInfo[11]}')

def get_ik(pos,orient):
    ik_joints = p.calculateInverseKinematics(robot_id, endEffectorLinkIndex = eef_link_idx,
                                targetPosition = pos,
                                targetOrientation = orient)
    return {"joints":ik_joints}

def move_to_position(joint_indices, position) -> None:
    p.setJointMotorControlArray(
        robot_id,
        jointIndices=joint_indices,
        controlMode=p.POSITION_CONTROL,
        targetPositions=position
    )

move_to_position(joint_indices, home_pos)

def get_state():
    link_state = p.getLinkState(robot_id,linkIndex=eef_link_idx,computeForwardKinematics=1)
    link_pos = link_state[0]
    link_orient = link_state[1]
    joint_states = p.getJointStates(robot_id, joint_indices)
    state = {
        "joints": [state[0] for state in joint_states] ,
        "cart": {
            "pos": link_pos,
            "quat": link_orient
        }
    }
    return state

async def step_in_background():
    t = 0.0
    while True:
        joint_data = [state[0] for state in p.getJointStates(robot_id, joint_indices)]
        Logger.log(t, joint_data, dt)
        p.stepSimulation()
        await asyncio.sleep(dt)
        t += dt

if __name__ == "__main__":    
    joint_indices = [1, 2]
    home_pos = [0.78, 0.78]
    print(p.getJointStates(robot_id, joint_indices))
    move_to_position(joint_indices, home_pos)

    print("After movement:")
    for joint in range(p.getNumJoints(robot_id)):
        print(f'{joint} {p.getJointInfo(robot_id, joint)}')
    
    import time
    print(p.getJointStates(robot_id, joint_indices))
    for i in range(10):
        start = time.monotonic()
        p.stepSimulation()
        end = time.monotonic()
        print(f'TIME: {end-start}')

    while True:
        p.stepSimulation()

    p.disconnect()