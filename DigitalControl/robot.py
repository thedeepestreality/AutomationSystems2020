import pybullet as p
import pybullet_data
import asyncio

URDF_PATH = "robot.urdf"
PRINT_JOINTS = True

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)

plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF(URDF_PATH, useFixedBase=True)

if PRINT_JOINTS:
    print("Available joints:")
    for joint in range(p.getNumJoints(robot_id)):
        print(f'{joint} {p.getJointInfo(robot_id, joint)[12]}')


def move_to_position(joint_indices: list[float], position: list[float]) -> None:
    p.setJointMotorControlArray(
        robot_id,
        jointIndices=joint_indices,
        controlMode=p.POSITION_CONTROL,
        targetPositions=position
    )

def get_joint_position():
    return (
        {'id': joint_id, 'name': p.getJointInfo(robot_id, joint_id)[12]}
        for joint_id in range(p.getNumJoints(robot_id))
    )

async def step_in_background():
    while True:
        p.stepSimulation()
        await asyncio.sleep(0.05)

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