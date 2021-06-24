import torch

import hydra
from polymetis import RobotInterface

@hydra.main(config="conf/experiment.yml")
def main(cfg):
    # Initialize robot

    # Move robot to desired pos

    # Load model & create controller

    # Execute model


if __name__ == '__main__':
    main()