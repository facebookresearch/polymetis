from typing import Dict

import google
import torch

import torchcontrol as toco

PI = 3.1415926
RADIUS = 0.15
PERIOD = 4


def timestamp_diff(ts1, ts2):
    return (ts1[0] - ts2[0]).to(torch.float32) + 1e-9 * (ts1[1] - ts2[1]).to(
        torch.float32
    )


class CircleDemoController(toco.PolicyModule):
    omega: float
    period: float
    radius: float

    def __init__(self, joint_positions, Kp, Kd, robot_model, period):
        super().__init__()
        self.robot_model = robot_model

        self.omega = 2 * PI / PERIOD
        self.radius = RADIUS
        self.feedback = toco.modules.JointSpacePD(Kp, Kd)

        # Initialize
        self.x0, _ = self.robot_model.forward_kinematics(joint_positions)
        self.ts0 = -torch.ones(2, dtype=torch.int32)

    def forward(self, state_dict: Dict[str, torch.Tensor]):
        # Acquire timestamp
        ts = state_dict["timestamp"]
        if self.ts0[0] < 0:
            self.ts0 = ts.clone()
        t = timestamp_diff(ts, self.ts0)

        # Get joint state
        q_current = state_dict["joint_pos"]
        qd_current = state_dict["joint_vel"]

        # Compute next joint pos
        J = self.robot_model.compute_jacobian(q_current)
        x_current, _ = self.robot_model.forward_kinematics(q_current)

        theta = self.omega * t
        x_desired = self.x0.clone()
        x_desired[0] = x_desired[0] + self.radius * (torch.cos(theta) - 1)
        x_desired[1] = x_desired[1] + self.radius * torch.sin(theta)

        q_desired = q_current + torch.pinverse(J)[:, 0:3] @ (x_desired - x_current)

        # Feedback control
        torque_out = self.feedback(
            q_current, qd_current, q_desired, torch.zeros_like(qd_current)
        )

        return {"torque_desired": torque_out}


class NNPeriodicPositionController(toco.PolicyModule):
    period: float

    def __init__(self, net, Kp, Kd, period):
        super().__init__()
        self.net = net
        self.period = period
        self.ts0 = -torch.ones(2, dtype=torch.int32)

        self.feedback = toco.modules.JointSpacePD(Kp, Kd)

    def forward(self, state_dict: Dict[str, torch.Tensor]):
        # Acquire timestamp
        ts = state_dict["timestamp"]
        if self.ts0[0] < 0:
            self.ts0 = ts
        t = torch.fmod(timestamp_diff(ts, self.ts0), self.period)

        # Get joint state
        q_current = state_dict["joint_pos"]
        qd_current = state_dict["joint_vel"]

        # Feedback control
        q_desired = self.net(t.unsqueeze(0))
        torque_out = self.feedback(
            q_current, qd_current, q_desired, torch.zeros_like(qd_current)
        )

        return {"torque_desired": torque_out}


class CircleTask:
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
        timestamp_ls = [
            torch.Tensor([e.timestamp.seconds, e.timestamp.nanos]) for e in log
        ]
        t0 = timestamp_ls[0]
        inputs = [timestamp_diff(t, t0).unsqueeze(0) for t in timestamp_ls]
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
