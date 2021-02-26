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

class NoSuchControlType(KeyError):
    pass

class Robot:
    dt = 1/240
    robot_id = p.loadURDF(URDF_PATH, useFixedBase=True)
    joints = (1, 2)
    eef_link_idx = 3

    def __init__(self):
        self.t = 0.0
        self.trajectory: Iterable = iter([])
        self.control_modes = {}
        self.previous_step_velocity = self.get_joints_velocity()

    def get_joints_position(self, joint_indicies=joints):
        return np.array([
            state[0] for state in p.getJointStates(self.robot_id, joint_indicies)
        ])

    def get_joints_velocity(self, joint_indicies=joints):
        return np.array([
            state[1] for state in p.getJointStates(self.robot_id, joint_indicies)
        ])

    def get_joints_acceleration(self, joint_indicies=joints):
        return (self.get_joints_velocity() - self.previous_step_velocity) / self.dt

    def get_full_state(self):
        link_state = p.getLinkState(self.robot_id, linkIndex=self.eef_link_idx)
        return {
            "joints": self.get_joints_position(),
            "vels": self.get_joints_velocity(),
            "cart": {
                "pos": link_state[0],
                "quat": link_state[1]
            }
        }

    def get_inverse_kinematics(self, position, orientation):
        return p.calculateInverseKinematics(
            self.robot_id,
            endEffectorLinkIndex=self.eef_link_idx,
            targetPosition=position,
            targetOrientation=orientation
        )

    def set_position_control(self, position):
        p.setJointMotorControlArray(
            self.robot_id,
            jointIndices=self.joints,
            controlMode=p.POSITION_CONTROL,
            targetPositions=position
        )

    def add_control_mode(self, name, function):
        """
        Adds new control regime
        
        function should take arguments
        robot: Robot, target_position: np.array, ts: time in s
        and return list or generator with trajectory points
        """
        self.control_modes[name] = function

    def set_control(self, control_mode, ts, target):
        try:
            control = self.control_modes[control_mode]
        except KeyError:
            raise NoSuchControlType(control_mode)

        trajectory = control(self, np.array(target), ts)
        
        if type(trajectory) is list:
            self.trajectory = iter(trajectory)
        else:
            self.trajectory = trajectory

    def process_control(self):
        next_position = next(self.trajectory, None)
        if next_position is not None:
            self.set_position_control(next_position)

    def log_state(self):
        data = (
            *self.get_joints_position(),
            *self.get_joints_velocity(),
            *self.get_joints_acceleration()
        )
        Logger.log(self.t, data, self.dt)

    def step(self):
        self.log_state()
        self.previous_step_velocity = self.get_joints_velocity()
        self.process_control()
        p.stepSimulation()
        self.t += self.dt

    async def step_in_background(self):
        while True:
            self.step()
            await asyncio.sleep(self.dt)


def p2p_cubic(robot, q_end, ts):
    q_start = robot.get_joints_position()
    a2 = 3/(ts**2)
    a3 = -2/(ts**3)
    t = 0

    while t < ts:
        t += robot.dt
        s = a2*t**2 + a3*t**3
        q = q_start + s*(q_end-q_start)
        yield q

robot = Robot()
robot.add_control_mode('cubic', p2p_cubic)

if __name__ == "__main__":    
    robot.set_control("cubic", ts=3, target=np.array([1., 1.]))
    while True:
        robot.step()
