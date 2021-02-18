import pybullet as p
import pybullet_data
import asyncio
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


if PRINT_JOINTS:
    print("Available joints:")
    for joint in range(p.getNumJoints(robot_id)):
        jointInfo = p.getJointInfo(robot_id, joint)
        print(f'{joint} name: {jointInfo[1]}')
        print(f'{joint} maxVel: {jointInfo[11]}')


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
    while True:
        p.stepSimulation()
        await asyncio.sleep(dt)

if __name__ == "__main__":    
    joint_indices = [1, 2]
    home_pos = [0.78, 0.78]

    move_to_position(joint_indices, home_pos)

    print("After movement:")
    for joint in range(p.getNumJoints(robot_id)):
        print(f'{joint} {p.getJointInfo(robot_id, joint)}')
    
    while True:
        p.stepSimulation()

    p.disconnect()