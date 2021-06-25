import os

import torch

import hydra
from polymetis import RobotInterface
import torchcontrol as toco

from models.mlp import MlpTrainer

class NNController(toco.PolicyModule):
    def __init__(self, net):
        self.net = net

    def forward(state_dict):
        joint_pos = state_dict["joint_pos"]
        joint_vel = state_dict["joint_vel"]

        state = torch.cat([joint_pos, joint_vel])

        return {"torque_desired": self.net(state)}

@hydra.main(config_path="conf/experiment.yml")
def main(cfg):
    # Initialize robot
    robot = RobotInterface(ip_address="localhost")

    # Move robot to desired pos
    print("Moving robot to desired position...")
    robot.set_ee_pose(
        position=torch.Tensor(cfg.task.desired_pos), orientation=None, time_to_go=3.0
    )

    # Load model & create controller
    print("Loading model...")
    assert model_dir
    model_path = os.path.join(hydra.utils.get_original_cwd(), model_dir, cfg.model.filename)
    mlp = MlpTrainer(cfg.model)
    mlp.load(model_path)
    print(f"Model loaded from {model_path}")

    net = mlp.get_model()
    policy = NNController(net)

    # Execute model
    print("Executing NN policy...")
    robot.send_torch_policy(policy)


if __name__ == '__main__':
    main()