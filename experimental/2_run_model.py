import os
from typing import Dict
import time

import torch

import hydra
from polymetis import RobotInterface
import torchcontrol as toco

from models.mlp import MlpTrainer


class NNController(toco.PolicyModule):
    def __init__(self, net):
        super().__init__()
        self.net = net

    def forward(self, state_dict: Dict[str, torch.Tensor]):
        joint_pos = state_dict["joint_pos"]
        joint_vel = state_dict["joint_vel"]

        state = torch.cat([joint_pos, joint_vel])

        return {"torque_desired": self.net(state)}


@hydra.main(config_path="conf/experiment.yml")
def main(cfg):
    # Load model & create controller
    print("Loading model...")
    assert (
        cfg.model_dir
    ), "Missing model dir. Add hydra override 'model_dir=<hydra output dir for previous run of 1_model_training.py>'"
    model_path = os.path.join(
        hydra.utils.get_original_cwd(), cfg.model_dir, cfg.model.filename
    )
    mlp = MlpTrainer(cfg.model)
    mlp.load(model_path)
    print(f"Model loaded from {model_path}")

    net = mlp.get_model()
    nn_policy = NNController(net)

    # Initialize robot
    robot = RobotInterface(ip_address="localhost")

    # Move robot to desired pos
    print("Moving robot to desired position...")
    robot.set_ee_pose(
        position=torch.Tensor(cfg.task.desired_pos), orientation=None, time_to_go=3.0
    )

    # Execute model
    print("Executing NN policy...")
    robot.send_torch_policy(nn_policy, blocking=False)
    try:
        time.sleep(20)
    except KeyboardInterrupt:
        print("Interrupted by user.")
    log = robot.terminate_current_policy()


if __name__ == "__main__":
    main()
