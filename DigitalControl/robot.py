import pybullet as p
import pybullet_data
import asyncio
from logger import Logger
import modern_robotics as mr
import numpy as np
from collections.abc import Iterable

URDF_PATH = "robot.urdf"
PRINT_JOINTS = True

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)

plane_id = p.loadURDF("plane.urdf")
# robot_id = p.loadURDF(URDF_PATH, useFixedBase=True)

joint_indices = [1, 2]
home_pos = np.array([0,0])
home_vel = np.array([0,0])
dt = 1/240
eef_link_idx = 3
_glob_data={"traj":[]}
_flags = {"traj":False}

class Robot:
    dt = 1/240
    robot_id = p.loadURDF(URDF_PATH, useFixedBase=True)
    joints = [1, 2]

    def __init__(self):
        self.t = 0.0
        self.trajectory: Iterable = []

    def get_joints_velocity(self):
        return np.array([
            state[1] for state in p.getJointStates(self.robot_id, self.joints)
        ])

    def set_trajectory(self, trajectory: Iterable):
        self.trajectory = trajectory

    def set_position_control(self, position):
        p.setJointMotorControlArray(
            self.robot_id,
            jointIndices=joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=position
        )

    def process_control(self):
        if next_position := next(self.trajectory, None):
            self.set_position_control(next_position)

    async def step(self):
        while True:
            self.process_control()
            p.stepSimulation()
            self.t += dt
            await asyncio.sleep(self.dt)

robot_id = Robot.robot_id


def get_ik(pos,orient):
    ik_joints = p.calculateInverseKinematics(robot_id, endEffectorLinkIndex = eef_link_idx,
                                targetPosition = pos,
                                targetOrientation = orient)
    return {"joints":ik_joints}

def p2p_cubic(q_start, q_end, t_end, t_start = 0):
    t = t_start
    a2 = 3/(t_end**2)
    a3 = -2/(t_end**3)
    traj = []
    while(t<t_end):
        t += dt
        s = a2*t**2 + a3*t**3
        q = q_start + s*(q_end-q_start)
        traj.append(q)
    return traj

def move_to_position(joint_indices, q_end, t_end, scaling = "cubic") -> None:
    q_start = p.getJointStates(robot_id, joint_indices)
    q_start = np.array([state[0] for state in q_start])
    q_end = np.array(q_end)

    if (scaling == "cubic"):
        traj = p2p_cubic(q_start, q_end, t_end)
    else:
        print("[Error] unknown scaling type")
        raise ValueError("unknown scaling type")
    _glob_data["traj"] = traj
    _flags["traj"] = True

def get_state():
    link_state = p.getLinkState(robot_id,linkIndex=eef_link_idx,computeForwardKinematics=1)
    link_pos = link_state[0]
    link_orient = link_state[1]
    joint_states = p.getJointStates(robot_id, joint_indices)
    state = {
        "joints": [state[0] for state in joint_states],
        "vels": [state[1] for state in joint_states],
        "cart": {
            "pos": link_pos,
            "quat": link_orient
        }
    }
    return state

async def step_in_background():
    t = 0.0
    traj_idx = 0
    vel_prev = np.array([0,0])
    while True:
        # print(f"flag: {_flags['traj']}")
        if _flags["traj"]:
            print(f"len: {len(_glob_data['traj'])}")
            if (traj_idx < len(_glob_data["traj"])):
                q_curr = _glob_data["traj"][traj_idx]
                print(f"CURR: {q_curr}")
                p.setJointMotorControlArray(
                        robot_id,
                        jointIndices=joint_indices,
                        controlMode=p.POSITION_CONTROL,
                        targetPositions=q_curr
                    )
                traj_idx += 1
            else:
                _flags["traj"] = False
                traj_idx = 0
        joint_data = [state[0] for state in p.getJointStates(robot_id, joint_indices)]
        vel_data = [state[1] for state in p.getJointStates(robot_id, joint_indices)]
        vel_data = np.array(vel_data)
        acc_data = (vel_data - vel_prev)/dt
        Logger.log(t, [*joint_data, *vel_data, *acc_data], dt)
        vel_prev = vel_data
        p.stepSimulation()
        await asyncio.sleep(dt)
        t += dt

if __name__ == "__main__":    
    joint_indices = [1, 2]
    home_pos = [0.78, 0.78]
    move_to_position(joint_indices, home_pos, 1)

    print("After movement:")
    for joint in range(p.getNumJoints(robot_id)):
        print(f'{joint} {p.getJointInfo(robot_id, joint)}')

    while True:
        p.stepSimulation()

    p.disconnect()