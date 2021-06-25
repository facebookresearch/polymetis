import os

import time

import torch

import hydra
from polymetis import RobotInterface
import torchcontrol as toco


@hydra.main(config_path="conf/experiment.yml")
def main(cfg):
    # Initialize robot
    robot = RobotInterface(ip_address="localhost")

    # Move robot to desired pos
    print("Moving robot to desired position...")
    robot.set_ee_pose(
        position=torch.Tensor(cfg.task.desired_pos), orientation=None, time_to_go=3.0
    )

    # Execute Cartesian PD
    print("Executing demonstration policy...")
    joint_pos_current = robot.get_joint_angles()
    pd_policy = toco.policies.JointImpedanceControl(
        joint_pos_current=joint_pos_current,
        Kp=robot.metadata.default_Kq,
        Kd=robot.metadata.default_Kqd,
        robot_model=robot.robot_model,
    )

    robot.send_torch_policy(pd_policy, blocking=False)
    try:
        time.sleep(60)
    except KeyboardInterrupt:
        print("Interrupted by user.")
    log = robot.terminate_current_policy()

    # Save data
    print("Saving collected data...")

    joint_pos_ls = [e.joint_positions for e in log]
    joint_vel_ls = [e.joint_velocities for e in log]
    torque_ls = [e.joint_torques_computed for e in log]

    states = [
        torch.cat([torch.Tensor(joint_pos), torch.Tensor(joint_vel)])
        for joint_pos, joint_vel in zip(joint_pos_ls, joint_vel_ls)
    ]
    actions = [torch.Tensor(torque) for torque in torque_ls]

    data = {
        "inputs": states,
        "outputs": actions,
    }
    torch.save(data, cfg.data.filename)
    print(f"Data saved to: {os.path.join(os.getcwd(), cfg.data.filename)}")


if __name__ == "__main__":
    main()
