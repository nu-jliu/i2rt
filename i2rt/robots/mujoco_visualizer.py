"""Real-time MuJoCo visualizer for i2rt robots.

Usage:
    robot = get_yam_robot(...)
    viz = MujocoVisualizer.from_robot(robot)
    viz.run()
"""

import time

import mujoco
import mujoco.viewer
import numpy as np

from i2rt.robots.robot import Robot
from i2rt.robots.utils import ArmType, GripperType
from i2rt.robots.motor_chain_robot import MotorChainRobot


class MujocoVisualizer:
    """MuJoCo viewer that mirrors an i2rt robot's joint positions."""

    def __init__(self, model: mujoco.MjModel, robot: Robot):
        self._model = model
        self._data = mujoco.MjData(model)
        self._robot = robot

    # ---- factory constructors ------------------------------------------------

    @classmethod
    def from_xml(cls, xml_path: str, robot: Robot) -> "MujocoVisualizer":
        model = mujoco.MjModel.from_xml_path(xml_path)
        return cls(model, robot)

    @classmethod
    def from_robot(cls, robot: "MotorChainRobot") -> "MujocoVisualizer":  # noqa: F821
        return cls.from_xml(robot.xml_path, robot)

    # ---- public API ----------------------------------------------------------

    def update(self, joint_pos: np.ndarray) -> None:
        """Set joint positions and forward-compute kinematics."""
        n = min(len(joint_pos), self._model.nq)
        self._data.qpos[:n] = joint_pos[:n]
        mujoco.mj_forward(self._model, self._data)

    def run(self, dt: float = 0.02) -> None:
        """Open the viewer and mirror the robot's joint positions until closed.

        Joint positions are polled from ``robot.get_joint_pos()`` each
        iteration.
        """
        nq = self._model.nq
        print("Mirror mode – reading joint positions from robot")
        with mujoco.viewer.launch_passive(self._model, self._data) as viewer:
            try:
                while viewer.is_running():
                    qpos = self._robot.get_joint_pos()
                    n = min(len(qpos), nq)
                    self._data.qpos[:n] = qpos[:n]
                    mujoco.mj_forward(self._model, self._data)
                    viewer.sync()
                    time.sleep(dt)
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    import argparse

    arm_choices = [a.value for a in ArmType]
    gripper_choices = [g.value for g in GripperType]

    parser = argparse.ArgumentParser(description="Visualize an i2rt robot model in MuJoCo")
    parser.add_argument("--arm", type=str, default="yam", choices=arm_choices, help="Arm type")
    parser.add_argument(
        "--gripper",
        type=str,
        default="crank_4310",
        choices=gripper_choices,
        help="Gripper type",
    )
    parser.add_argument("--sim", action="store_true", help="Use a SimRobot instead of real hardware")
    args = parser.parse_args()

    from i2rt.robots.get_robot import get_yam_robot

    arm = ArmType.from_string_name(args.arm)
    gripper = GripperType.from_string_name(args.gripper)
    robot = get_yam_robot(arm_type=arm, gripper_type=gripper, sim=args.sim)
    viz = MujocoVisualizer.from_robot(robot)
    viz.run()
