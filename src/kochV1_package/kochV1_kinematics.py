from __future__ import annotations

import numpy as np
import sympy as sp

from dataclasses import dataclass, field
from enum import Enum

from spatialmath import SE3

from typing import (
    Tuple,
    List,
    Callable
)

from utils import (
    Unit, 
    to_dxl_units,
    check_limits
)

class JointType(Enum):
    REVOLUTE = 1
    PRISMATIC = 2


# d will be ignored for prismatic joint and 
# theta will be ignored for revolute joint
@dataclass(frozen=True, slots=True)
class DHJoint:
    type: JointType                 = JointType.REVOLUTE
    name: str                       = ""
    a: float                        = 0.0                   # link length (m)
    alpha: float                    = 0.0                   # link twist (rad)
    d: float                        = 0.0                   # link offset (m)
    theta: float                    = 0.0                   # joint angle (rad)
    q_limits: Tuple[float, float]   = (-np.inf, np.inf)     # joint limits (rad or m)
    q_offset: float                 = 0.0                   # joint offset (rad or m)

    def get_transform(self, q: float = 0.0) -> SE3:
        # Select DH parameters depending on joint type
        # REVOLUTE
        if self.type is JointType.REVOLUTE:
            theta = self.q_offset + q
            d     = self.d
        
        # PRISMATIC
        else:
            theta = self.theta
            d     = self.q_offset + q

        # see https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
        cos_alpha, sin_alpha = np.cos(self.alpha),  np.sin(self.alpha)
        cos_theta, sin_theta = np.cos(theta),       np.sin(theta)

        T = np.array([
            [ cos_theta, -sin_theta*cos_alpha,  sin_theta*sin_alpha, self.a*cos_theta],
            [ sin_theta,  cos_theta*cos_alpha, -cos_theta*sin_alpha, self.a*sin_theta],
            [     0,           sin_alpha,            cos_alpha,             d        ],
            [     0,               0,                    0,                 1        ],
        ])

        return SE3(T)


@dataclass(frozen=True, slots=True)
class RobotConfig:
    dh_joints: Tuple[DHJoint]

    @staticmethod
    def from_DH_parameters(dh_parameters: List[DHJoint]) -> 'RobotConfig':
        return RobotConfig(dh_joints=tuple(dh_parameters))

    @staticmethod
    def from_URDF(urdf_path: str) -> 'RobotConfig':
        pass


class KochV1_KinematicsModel:
    def __init__(self):
        joints = [
            DHJoint(type=JointType.REVOLUTE, a=0,       alpha=np.pi/2, d=0.0563,   q_offset=0),
            DHJoint(type=JointType.REVOLUTE, a=0.10931, alpha=0,       d=0,        q_offset=-7.78 * (np.pi/180)),
            DHJoint(type=JointType.REVOLUTE, a=0.10051, alpha=0,       d=0,        q_offset= 9.32 * (np.pi/180)),
            DHJoint(type=JointType.REVOLUTE, a=7e-6,    alpha=np.pi/2, d=0.953e-3, q_offset=88.46 * (np.pi/180)),
            DHJoint(type=JointType.REVOLUTE, a=0,       alpha=0,       d=0.0681,   q_offset=0),
        ]

        self._robot_cfg = RobotConfig.from_DH_parameters(joints)

        self._compute_jacobian_func = self._init_compute_jacobian_func()

    @property
    def robot_cfg(self) -> RobotConfig:
        return self._robot_cfg

    def compute_forward_kinematics(self, joint_angles: List[float]) -> SE3:
        T_ee = SE3()

        for dh_joint, q in zip(self.robot_cfg.dh_joints, joint_angles):
            T_ee = T_ee * dh_joint.get_transform(q)

        return T_ee
    
    def compute_inverse_kinematics(self, T_ee: SE3, elbow_down: bool = True) -> List[float] | None:
        l_0 = self.robot_cfg.dh_joints[0].d
        l_1 = self.robot_cfg.dh_joints[1].a
        l_2 = self.robot_cfg.dh_joints[2].a
        l_3 = self.robot_cfg.dh_joints[4].d

        # Desired Position:
        (x_des, y_des, z_des) = T_ee.t
        psi = np.asin(T_ee.A[2, 2])       

        # Theta 1 ---------------------------------------------------------------------
        theta_1 = np.atan2(y_des, x_des) if x_des != 0 else 0
        if not check_limits(theta_1, self.robot_cfg.dh_joints[0].q_limits):
            return None
        
        # Transform to y'-O_1-x'
        x_5 = np.sqrt(x_des**2 + y_des**2)
        y_5 = z_des - l_0

        # Transform to 2D-Planar:
        x_3 = x_5 - (l_3*np.cos(psi)) 
        y_3 = y_5 - (l_3*np.sin(psi)) 

        d = np.sqrt(x_3**2 + y_3**2)

        # Theta 2, Theta 3 ------------------------------------------------------------
        alpha = np.acos((l_1**2 + l_2**2 - d**2) / (2*l_1*l_2))
        beta = np.acos((l_1**2 + d**2 - l_2**2) / (2*l_1*d))

        if elbow_down:
            theta_2 = np.atan2(y_3, x_3) - beta
            theta_3 = np.pi - alpha
        else:
            theta_2 = np.atan2(y_3, x_3) + beta
            theta_3 = -(np.pi - alpha)

        if (not check_limits(theta_2, self.robot_cfg.dh_joints[1].q_limits) 
                or not check_limits(theta_3, self.robot_cfg.dh_joints[2].q_limits)):
                return None

        # Theta 4 ---------------------------------------------------------------------
        theta_4 = psi - (theta_2 + theta_3)
        if not check_limits(theta_4, self.robot_cfg.dh_joints[3].q_limits):
            return None

        # Theta 5 ---------------------------------------------------------------------
        # theta_5 can be determined by analyzing the difference between two poses:
        #   1. The pose before rotation around the end-effector's z-axis.
        #   2. The goal end-effector pose: pose after rotation around the end-effector's z-axis.
        T_before = self.compute_forward_kinematics([theta_1, theta_2, theta_3, theta_4, 0])
        T_after = T_ee

        x_before = T_before.R[:, 0]
        x_after = T_after.R[:, 0]

        z_local = T_after.R[:, 2]  # end-effector's local Z-axis, same for both poses

        cos_theta_5 = np.dot(x_before, x_after)
        sin_theta_5 = np.dot(z_local, np.cross(x_before, x_after))   # vectors are normalized
        
        theta_5 = np.arctan2(sin_theta_5, cos_theta_5)
        if not check_limits(theta_5, self.robot_cfg.dh_joints[4].q_limits):
            return None

        return [theta_1, theta_2, theta_3, theta_4, theta_5]
    
    def compute_jacobian(self, joint_angles: List[float]) -> np.ndarray:
        """
        Compute the Jacobian matrix for the current joint angles.
        :param joint_angles: List of joint angles.
        :return: Jacobian matrix for current joint state.
        """
        return np.asarray(self._compute_jacobian_func(*joint_angles), dtype=float)
    
    def _init_compute_jacobian_func(self) -> Callable:
        J_sp, q_sp = self._build_sympy_jacobian()

        J_func = sp.lambdify(q_sp, J_sp, modules="numpy")

        return J_func
    
    def _build_sympy_jacobian(self):
        """Build the symbolic Jacobian matrix using SymPy."""

        # helper function to create a symbolic DH transformation matrix
        def A_sym(a, alpha, d, theta) -> sp.Matrix:
            ca, sa = sp.cos(alpha), sp.sin(alpha)
            ct, st = sp.cos(theta), sp.sin(theta)
            
            return sp.Matrix([
                [ ct, -st*ca,  st*sa, a*ct],
                [ st,  ct*ca, -ct*sa, a*st],
                [  0,     sa,     ca,   d ],
                [  0,      0,      0,   1 ]
            ])
        
        # build J(q)
        n = len(self.robot_cfg.dh_joints)
        q  = sp.symbols(f'q1:{n+1}')           # (q1, q2, …, qn)

        T   = sp.eye(4)
        o   = [sp.Matrix([0, 0, 0])]          # origin of frame 0
        z   = [sp.Matrix([0, 0, 1])]          # z-axis of frame 0

        # Forward kinematics: T0i, oi, zi
        for i, joint in enumerate(self.robot_cfg.dh_joints):
            if joint.type is JointType.REVOLUTE:
                theta = q[i] + joint.q_offset
                d     = joint.d
            else:  # PRISMATIC
                theta = joint.theta
                d     = q[i] + joint.q_offset

            T  = T * A_sym(joint.a, joint.alpha, d, theta)
            o.append(T[:3, 3])
            z.append(T[:3, 2])

        o_n = o[-1]                            # end-effector origin

        # Assemble Jacobian column by column
        Jv, Jw = [], []
        for i, joint in enumerate(self.robot_cfg.dh_joints):
            if joint.type is JointType.REVOLUTE:
                Jv.append(z[i].cross(o_n - o[i]))
                Jw.append(z[i])
            else:  # PRISMATIC
                Jv.append(z[i])
                Jw.append(sp.zeros(3, 1))

        J = sp.Matrix.hstack(*Jv).col_join(sp.Matrix.hstack(*Jw))
        return J, q


if __name__ == "__main__":

    model = KochV1_KinematicsModel()
    print(model.robot_cfg.dh_joints[0].get_transform())

    # speed check forward kinematics
    print(model.compute_forward_kinematics([0, 0, 0, 0, 0]))
