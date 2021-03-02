import pybullet as p
import pybullet_data
import asyncio
from logger import Logger
import modern_robotics as mr
import numpy as np
from collections.abc import Iterable
from dtos import *

URDF_PATH = "robot.urdf"
PRINT_JOINTS = True

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
plane_id = p.loadURDF("plane.urdf")

class NoSuchControlType(KeyError):
    pass

class Robot:
    # To be available as default argument made as a class variable
    joints = (1, 2)

    def __init__(self):
        self.robot_id = p.loadURDF(URDF_PATH, useFixedBase=True)
        self.dt = 1/240
        self.t = 0.0
        self.trajectory: Iterable = iter([])
        self.scaling_modes = {}
        self.interpolation_modes = {}
        self.previous_step_velocity = self.get_joints_velocity()
        self.eef_link_idx = 3

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

    def get_cart_pose(self):
        link_state = p.getLinkState(self.robot_id, linkIndex=self.eef_link_idx)
        return link_state[0]

    def get_full_state(self):
        link_state = p.getLinkState(self.robot_id, linkIndex=self.eef_link_idx)
        return {
            "joints": list(self.get_joints_position()),
            "vels": list(self.get_joints_velocity()),
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

    def add_scaling_mode(self, name, function):
        """
        Adds new control regime
        
        function should take arguments
        robot: Robot, target_position: np.array, ts: time in s
        and return list or generator with trajectory points
        """
        self.scaling_modes[name] = function

    def add_interpolation_mode(self, name, function):
        """
        Adds new control regime
        
        function should take arguments
        robot: Robot, target_position: np.array, ts: time in s
        and return list or generator with trajectory points
        """
        self.interpolation_modes[name] = function

    def set_control(self, scaling_mode, ts, target):
        try:
            control = self.scaling_modes[scaling_mode]
        except KeyError:
            raise NoSuchControlType(scaling_mode)

        trajectory = control(self, np.array(target), ts)
        
        if type(trajectory) is list:
            self.trajectory = iter(trajectory)
        else:
            self.trajectory = trajectory
    
    def set_traj_control_lin(self, scaling_mode, target_traj):
        try:
            control = self.scaling_modes[scaling_mode]
        except KeyError:
            raise NoSuchControlType(scaling_mode)

        def union_generator():
            for target in target_traj:
                yield from control(self, np.array(target.pos), target.ts)
        
        trajectory = union_generator()

        if type(trajectory) is list:
            self.trajectory = iter(trajectory)
        else:
            self.trajectory = trajectory

    def set_traj_control_interp(self, interpolation_mode, target_traj):
        print("JCONTROL")
        try:
            control = self.interpolation_modes[interpolation_mode]
        except KeyError:
            raise NoSuchControlType(interpolation_mode)

        def union_generator():
            sz = len(target_traj)
            prev_pos = self.get_joints_position()
            for idx in range(sz):
                target = target_traj[idx]
                curr_pos = np.array(target.pos)
                next_pos = prev_pos
                if (idx < sz-1):
                    next_pos = np.array(target_traj[idx+1].pos)
                curr_vel = next_pos-prev_pos
                prev_pos = curr_pos
                norm = np.linalg.norm(curr_vel)
                if (norm > 1e-4):
                    curr_vel /= norm
                else:
                    curr_vel = np.array([0,0])
                
                yield from control(self, curr_pos, curr_vel, target.ts)
        
        trajectory = union_generator()

        if type(trajectory) is list:
            self.trajectory = iter(trajectory)
        else:
            self.trajectory = trajectory

    def set_cart_traj(self, interpolation_mode, target_traj):
        joint_targets = []
        for cart in target_traj:
            joints = p.calculateInverseKinematics(
                self.robot_id,
                endEffectorLinkIndex=self.eef_link_idx,
                targetPosition=cart.pos,
                targetOrientation=cart.orient
            )
            joints = list(joints)
            curr_targ = JointTarget(pos= joints, ts=cart.ts)
            joint_targets.append(curr_targ)
        self.set_traj_control_interp(interpolation_mode, joint_targets)
    
    def process_control(self):
        next_position = next(self.trajectory, None)
        if next_position is not None:
            self.set_position_control(next_position)

    def log_state(self):
        data = (
            *self.get_joints_position(),
            *self.get_joints_velocity(),
            *self.get_joints_acceleration(),
            *self.get_cart_pose()
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
        # Or you can apend q to some list
        # and return list at the end

def traj_segment_cubic(robot, q_end, vel_end, ts):
    q_start = robot.get_joints_position()
    vel_start = robot.get_joints_velocity()

    b0 = q_start
    db0 = vel_start
    b1 = q_end
    db1 = vel_end

    a0 = b0
    a1 = db0
    a2 = (3*b1 - 3*b0 - 2*db0*ts - db1*ts)/(ts**2)
    a3 = (2*b0 + (db0 + db1)*ts - 2*b1)/(ts**3)
    t = 0

    while t < ts:
        t += robot.dt
        q = a0 + a1*t + a2*t**2 + a3*t**3
        yield q
        # Or you can apend q to some list
        # and return list at the end

robot = Robot()
robot.add_scaling_mode('cubic', p2p_cubic)
robot.add_interpolation_mode('cubic', traj_segment_cubic)

if __name__ == "__main__":    
    robot.set_control("cubic", ts=3, target=np.array([1., 1.]))
    while True:
        robot.step()
