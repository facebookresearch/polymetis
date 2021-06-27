import os

import time

import torch

import hydra
from polymetis import RobotInterface
import torchcontrol as toco


@hydra.main(config_path="conf/experiment.yml")
def main(cfg):
    # Initialize
    robot = RobotInterface(ip_address="localhost")
    task_module = hydra.utils.instantiate(cfg.task.module)

    # Move robot to desired pos
    print("Moving robot to desired position...")
    robot.set_joint_positions(
        desired_positions=torch.Tensor(cfg.task.joint_pos_init), time_to_go=3.0
    )

    # Execute Cartesian PD
    print("Executing demonstration policy...")
    demo_policy = task_module.get_demonstration_policy(robot)
    robot.send_torch_policy(demo_policy, blocking=False)
    try:
        time.sleep(cfg.data.collection_time)
    except KeyboardInterrupt:
        print("Interrupted by user.")
    log = robot.terminate_current_policy()

    # Save data
    print("Saving collected data...")
    inputs, outputs = task_module.process_log(log)
    data = {
        "inputs": inputs,
        "outputs": outputs,
    }

    torch.save(data, cfg.data.filename)
    print(f"Data saved to: {os.path.join(os.getcwd(), cfg.data.filename)}")


if __name__ == "__main__":
    main()
