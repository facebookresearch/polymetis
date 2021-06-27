from typing import Dict

import torch

import torchcontrol as toco


class NNController(toco.PolicyModule):
    def __init__(self, net):
        super().__init__()
        self.net = net

    def forward(self, state_dict: Dict[str, torch.Tensor]):
        joint_pos = state_dict["joint_pos"]
        joint_vel = state_dict["joint_vel"]

        state = torch.cat([joint_pos, joint_vel])

        return {"torque_desired": self.net(state)}


class PdTask:
    @staticmethod
    def get_demonstration_policy(robot):
        joint_pos_current = robot.get_joint_angles()
        policy = toco.policies.JointImpedanceControl(
            joint_pos_current=joint_pos_current,
            Kp=robot.metadata.default_Kq,
            Kd=robot.metadata.default_Kqd,
            robot_model=robot.robot_model,
        )
        return policy

    @staticmethod
    def process_log(log):
        joint_pos_ls = [e.joint_positions for e in log]
        joint_vel_ls = [e.joint_velocities for e in log]
        torque_ls = [e.joint_torques_computed for e in log]

        states = [
            torch.cat([torch.Tensor(joint_pos), torch.Tensor(joint_vel)])
            for joint_pos, joint_vel in zip(joint_pos_ls, joint_vel_ls)
        ]
        actions = [torch.Tensor(torque) for torque in torque_ls]

        return states, actions

    @staticmethod
    def get_controller(robot, net):
        return NNController(net)
