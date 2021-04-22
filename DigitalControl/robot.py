import pybullet as p
import pybullet_data
import asyncio
from logger import Logger
import modern_robotics as mr
import numpy as np
from collections.abc import Iterable
from dtos import *
from dynamic_tests.camera import Camera
import cv2
import math

URDF_PATH = "robot.urdf"
PRINT_JOINTS = True

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
plane_id = p.loadURDF("plane.urdf")

fov = 126.8
sz = {
    'width': 1000,
    'height': 1000
}

class NoSuchControlType(KeyError):
    pass

def rot2eul(R):
    beta = -np.arcsin(R[2,0])
    alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    return np.array((alpha, beta, gamma))
    
class Robot:
    # To be available as default argument made as a class variable
    joints = (1, 2, 3, 4, 5, 6)

    def __init__(self):
        self.robot_id = p.loadURDF(URDF_PATH, useFixedBase=True)
        self.dt = 1/240
        self.t = 0.0
        self.trajectory: Iterable = iter([])
        self.scaling_modes = {}
        self.interpolation_modes = {}
        self.motion_types = {}
        self.previous_step_velocity = self.get_joints_velocity()
        self.eef_link_idx = 7

        self.camera = Camera(size=sz, fov=fov, cameraTargetPosition=[3,0,0.5], cameraEyePosition=[2.5,0,0.5])

        self.set_position_control([0,0,1.57,0,1.57,0])
        for i in range(100):
            p.stepSimulation()

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

    def pybullet_to_homogeneous(self, pos, quat):
        np_pos = np.array([pos]).T
        flat_rot = np.array(p.getMatrixFromQuaternion(quat))
        mat_rot = np.reshape(flat_rot, (3,3))
        X = np.concatenate((mat_rot, np_pos), axis = 1)
        X = np.concatenate((X, np.array([[0,0,0,1]])), axis = 0)
        return X

    def get_homogeneous(self):
        link_state = p.getLinkState(self.robot_id, linkIndex=self.eef_link_idx)
        return self.pybullet_to_homogeneous(link_state[0], link_state[1])

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

    def add_motion_type(self, name, function):
        """
        Adds new control regime
        
        function should take arguments
        robot: Robot, target_position: np.array, ts: time in s
        and return list or generator with trajectory points
        """
        self.motion_types[name] = function

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

    def set_cart_traj_p2p(self, motion_type, target_traj):
        try:
            control = self.motion_types[motion_type]
        except KeyError:
            raise NoSuchControlType(motion_type)

        def union_generator():
            for cart in target_traj:
                T = self.pybullet_to_homogeneous(cart.pos, cart.orient)
                yield from control(self, T, cart.ts)
        
        trajectory = union_generator()

        if type(trajectory) is list:
            self.trajectory = iter(trajectory)
        else:
            self.trajectory = trajectory
    
    def process_control(self):
        # marker_pos = p.getJointState(self.robot_id, jointIndex = 8)[0]
        # p.setJointMotorControl2(bodyIndex=self.robot_id, 
        #                         jointIndex=8, 
        #                         targetPosition=marker_pos+0.05, 
        #                         controlMode=p.POSITION_CONTROL)

        # marker_estim = -self.process_image()[1]
        # link_state = p.getLinkState(self.robot_id, linkIndex=self.eef_link_idx)
        # new_pos = (link_state[0][0], link_state[0][1], link_state[0][2]+marker_estim)
        # joints = self.get_inverse_kinematics(new_pos, link_state[1])
        # joints = joints[0:6]
        # self.set_position_control(joints)
        next_position = next(self.trajectory, None)
        if next_position is not None:
            self.set_position_control(next_position)
        

    def process_image(self):
        data = robot.camera.get_frame()
        data = np.array(data)
        data = np.reshape(data, (sz['height'], sz['width'], 4))
        # RGB -> BGR UMat for opencv
        img = np.asarray(data[:,:,[2,1,0]], dtype=np.uint8)
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()

        corners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(img, dictionary, parameters=parameters)

        img = cv2.UMat(img)
        cv2.aruco.drawDetectedMarkers(img, corners, markerIds)

        focal_length = 1/math.tan(math.radians(fov)/2)
        focal_length = sz['height']/4
        markerLength = 3*0.2/4
        distCoeffs = np.zeros([0,0,0,0])
        cameraMatrix = np.array([[focal_length, 0, sz['width']/2],
                                [0, focal_length, sz['height']/2],
                                [0, 0, 1]])
        rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(	corners, markerLength, cameraMatrix, distCoeffs)
        print(tvecs[0,0,:])
        return tvecs[0,0,:]

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
        # self.process_image()
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


def p2p_five(robot, q_end, ts):
    q_start = robot.get_joints_position()
    a0 = q_start
    a1 = q_start * 0
    a2 = q_start * 0
    a3 = (
        1/2/(ts**3) *
        (20 * (q_end - q_start))
    )
    a4 = (
        1/2/(ts**4) *
        (30 * (q_start - q_end))
    )
    a5 = (
        1/2/(ts**5) *
        (12 * (q_end - q_start))
    )

    t = 0

    while t < ts:
        t += robot.dt
        q = a0 + a3*t**3 + a4*t**4 + a5*t**5
        yield q

def p2p_trapezoid(robot, q_end, ts):
    q_start = robot.get_joints_position()
    # !!! ---------------- !!!
    # t_acc = 1/3 * ts
    #v = 0.4
    a = 1
    if a*ts**2 - 4 < 0:
        print("Error: can't reach target in time")
        return
    v = 1/2 * (a*ts - np.sqrt(a) * np.sqrt(a*ts**2 - 4))
    #assert v**2 / a <= 1

    t = 0

    va = v / a

    while t < va:
        t += robot.dt
        s = 1/2 * a * t**2
        q = q_start + s * (q_end-q_start)
        yield q

    while t < ts - va:
        t += robot.dt
        s = (v*t) - ( v**2 / (2*a) )
        q = q_start + s * (q_end-q_start)
        yield q

    while t < ts:
        t += robot.dt
        s = (2*a*v*ts - 2*(v**2) - (a**2) * (t - ts)**2) / (2*a)
        q = q_start + s * (q_end-q_start)
        yield q

def p2p_cart_cubic_screw(robot, T_end, ts):
    T_start = robot.get_homogeneous()

    a2 = 3/(ts**2)
    a3 = -2/(ts**3)
    t = 0

    while t < ts:
        t += robot.dt
        s = a2*t**2 + a3*t**3
        T = T_start @ mr.MatrixExp6(mr.MatrixLog6(np.linalg.inv(T_start)@T_end)*s)

        pos = T[0:3,3]
        mat_rot = T[0:3,0:3]
        eul = rot2eul(mat_rot)

        quat = p.getQuaternionFromEuler(eul)
        joints = p.calculateInverseKinematics(
                robot.robot_id,
                endEffectorLinkIndex=robot.eef_link_idx,
                targetPosition=pos,
                targetOrientation=quat
            )
        yield joints
        # Or you can apend q to some list
        # and return list at the end

def p2p_cart_cubic_decoupled(robot, T_end, ts):
    T_start = robot.get_homogeneous()
    pos_start = T_start[0:3,3]
    pos_end = T_end[0:3,3]
    rot_start = T_start[0:3,0:3]
    rot_end = T_end[0:3,0:3]
    a2 = 3/(ts**2)
    a3 = -2/(ts**3)
    t = 0

    while t < ts:
        t += robot.dt
        s = a2*t**2 + a3*t**3
        pos = pos_start + (pos_end - pos_start)*s
        mat_rot = rot_start @ mr.MatrixExp3(mr.MatrixLog3(rot_start.T @ rot_end)*s)
        eul = rot2eul(mat_rot)
        quat = p.getQuaternionFromEuler(eul)
        joints = p.calculateInverseKinematics(
                robot.robot_id,
                endEffectorLinkIndex=robot.eef_link_idx,
                targetPosition=pos,
                targetOrientation=quat
            )
        yield joints
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
robot.add_scaling_mode('five', p2p_five)
robot.add_scaling_mode('trapezoid', p2p_trapezoid)

robot.add_interpolation_mode('cubic', traj_segment_cubic)

robot.add_motion_type('screw', p2p_cart_cubic_screw)
robot.add_motion_type("decoupled", p2p_cart_cubic_decoupled)

if __name__ == "__main__":    
    link_state = p.getLinkState(robot.robot_id, linkIndex=robot.eef_link_idx)
    X = robot.pybullet_to_homogeneous(link_state[0], link_state[1])
    print(f"X: {X}")
    rot = X[0:3,0:3]
    eul = rot2eul(rot)
    print(f"EUL: {eul}")

    # njoints = p.getNumJoints(robot.robot_id)
    # for idx in range (njoints):
    #     print(f'{idx}: {p.getJointInfo(robot.robot_id,idx)[1]}')
    # robot.set_control("cubic", ts=3, target=np.array([1., 1.]))
    while True:
        robot.step()
