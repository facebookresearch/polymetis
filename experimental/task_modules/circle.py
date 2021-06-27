from typing import Dict

import torch

import torchcontrol as toco

PI = 3.1415926
RADIUS = 0.15
PERIOD = 4


def timestamp2float(timestamp):
    return timestamp.seconds + 1e-9 * timestamp.nanos


class CircleDemoController(toco.PolicyModule):
    twopi: int

    def __init__(self, joint_positions, Kq, Kqd, robot_model, period):
        super().__init__()
        self.robot_model = robot_model
        self.period = period

        self.omega = 2 * PI / self.period
        self.feedback = toco.modules.JointSpacePD(kq, kqd)

        # Get initial pose
        self.x0 = self.robot_model.forward_kinematics(joint_positions)

    def forward(self, state_dict: Dict[str, torch.Tensor]):
        # Acquire timestamp
        t = timestamp2float(state_dict["timestamp"])
        if self.t0 < 0:
            self.t0 = t
        t = (t - self.t0) % self.period

        # Get joint state
        q_current = state_dict["joint_pos"]
        qd_current = state_dict["joint_vel"]

        # Compute next joint pos
        J = self.robot_model.compute_jacobian(q_current)
        x_current = self.robot_model.forward_kinematics(q_current)

        theta = self.omega * t
        x_desired = self.x0.copy()
        x_desired[0] = x_desired[0] + RADIUS * (torch.cos(theta) - 1)
        x_desired[1] = x_desired[1] + RADIUS * torch.sin(theta)

        q_desired = q_current + torch.pinverse(J)[:, 0:3] @ (x_desired - x_current)

        # Feedback control
        torque_out = self.feedback(
            q_current, qd_current, q_desired, torch.zeros_like(qd_current)
        )

        return {"torque_desired": torque_out}


class NNPeriodicPositionController(toco.PolicyModule):
    def __init__(self, net, Kq, Kqd, period):
        super().__init__()
        self.net = net
        self.period = period
        self.t0 = -1
        self.feedback = toco.modules.JointSpacePD(kq, kqd)

    def forward(self, state_dict: Dict[str, torch.Tensor]):
        # Acquire timestamp
        t = timestamp2float(state_dict["timestamp"])
        if self.t0 < 0:
            self.t0 = t
        t = (t - self.t0) % self.period

        # Get joint state
        q_current = state_dict["joint_pos"]
        qd_current = state_dict["joint_vel"]

        # Feedback control
        q_desired = self.net(torch.Tensor([t]))
        torque_out = self.feedback(
            q_current, qd_current, q_desired, torch.zeros_like(qd_current)
        )

        return {"torque_desired": torque_out}


class PdTask:
    @staticmethod
    def get_demonstration_policy(robot):
        joint_pos_current = robot.get_joint_angles()
        policy = CircleDemoController(
            joint_positions=joint_pos_current,
            Kp=robot.metadata.default_Kq,
            Kd=robot.metadata.default_Kqd,
            robot_model=robot.robot_model,
            period=PERIOD,
        )

        return policy

    @staticmethod
    def process_log(log):
        timestamp_ls = [timestamp2float(e.timestamp) for e in log]
        inputs = [torch.Tensor([t]) for t in timestamp_ls]
        outputs = [torch.Tensor(e.joint_positions) for e in log]

        return inputs, outputs

    @staticmethod
    def get_controller(robot, net):
        return NNPeriodicPositionController(
            net=net,
            Kp=robot.metadata.default_Kq,
            Kd=robot.metadata.default_Kqd,
            period=PERIOD,
        )
