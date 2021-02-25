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
        self.trajectory: Iterable = iter([])
        self.control_modes = {}

    def get_joints_velocity(self):
        return [
            state[1] for state in p.getJointStates(self.robot_id, self.joints)
        ]

    def get_joints_position(self):
        return [
            state[0] for state in p.getJointStates(self.robot_id, self.joints)
        ]

    def set_position_control(self, position):
        p.setJointMotorControlArray(
            self.robot_id,
            jointIndices=joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=position
        )

    def add_control_mode(self, name, function):
        self.control_modes[name] = function

    def set_control(self, control_mode, ts, target):
        function = self.control_modes[control_mode]
        trajectory = function(
            self,
            np.array(self.get_joints_position()), 
            np.array(target),
            ts
        )
        if type(trajectory) is list:
            self.trajectory = iter(trajectory)
        else:
            self.trajectory = trajectory

    def process_control(self):
        # Python 3.9+: if next_position := next(self.trajectory, None):
        next_position = next(self.trajectory, None)
        if next_position is not None:
            self.set_position_control(next_position)

    def step(self):
        Logger.log(self.t, [*self.get_joints_position(), *self.get_joints_velocity(), 0,0], self.dt)
        self.process_control()
        p.stepSimulation()
        self.t += dt

    async def step_in_background(self):
        while True:
            self.step()
            await asyncio.sleep(self.dt)


def p2p_cubic(robot, q_start, q_end, ts):
    t_start = 0
    t_end = t_start + ts
    t = t_start
    a2 = 3/(t_end**2)
    a3 = -2/(t_end**3)
    while(t<t_end):
        t += dt
        s = a2*t**2 + a3*t**3
        q = q_start + s*(q_end-q_start)
        yield q

robot = Robot()
robot.add_control_mode('cubic', p2p_cubic)

robot_id = Robot.robot_id


def get_ik(pos,orient):
    ik_joints = p.calculateInverseKinematics(robot_id, endEffectorLinkIndex = eef_link_idx,
                                targetPosition = pos,
                                targetOrientation = orient)
    return {"joints":ik_joints}


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

if __name__ == "__main__":    
    robot.set_control("cubic", 3, np.array([1., 1.]))
    while True:
        robot.step()
