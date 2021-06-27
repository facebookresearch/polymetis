import os
import time

import torch

import hydra
from polymetis import RobotInterface
import torchcontrol as toco

from models.mlp import MlpTrainer


@hydra.main(config_path="conf/experiment.yml")
def main(cfg):
    # Initialize
    robot = RobotInterface(ip_address="localhost")
    task_module = hydra.utils.instantiate(cfg.task.module)

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
    nn_policy = task_module.get_controller(robot, net)

    # Move robot to desired pos
    print("Moving robot to initial position...")
    robot.set_joint_positions(
        desired_positions=torch.Tensor(cfg.task.joint_pos_init), time_to_go=3.0
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
