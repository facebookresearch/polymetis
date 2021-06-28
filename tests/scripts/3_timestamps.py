# Copyright (c) Facebook, Inc. and its affiliates.
#
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
import time
from typing import Dict

import torch

from polymetis import RobotInterface
import torchcontrol as toco


class TimestampCheckController(toco.PolicyModule):
    def __init__(self, hz):
        super().__init__()

        self.dt = torch.tensor(1.0 / hz)
        self.ts_prev = -torch.ones(2).to(torch.int32)

    def forward(self, state_dict: Dict[str, torch.Tensor]):
        ts = state_dict["timestamp"]

        if self.ts_prev[0] > 0:
            t_diff = (ts[0] - self.ts_prev[0]).to(torch.float32) + 1e-9 * (
                ts[1] - self.ts_prev[1]
            ).to(torch.float32)
            assert torch.allclose(t_diff, self.dt)

        self.ts_prev = ts

        return {"torque_desired": torch.zeros(7)}


if __name__ == "__main__":
    # Initialize robot interface
    robot = RobotInterface(
        ip_address="localhost",
    )

    # Run policy
    policy = TimestampCheckController(robot.metadata.hz)
    robot.send_torch_policy(policy, blocking=False)
    time.sleep(10 / robot.metadata.hz)
    robot.terminate_current_policy()
