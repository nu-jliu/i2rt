"""Real-time MuJoCo visualizer for i2rt robots.

Usage:
    robot = get_yam_robot(...)

    # Interactive mode (default) – move joints via GUI/keyboard:
    viz = MujocoVisualizer.from_robot(robot, mode="interactive")
    viz.run()

    # Mirror mode – passively display robot joint positions:
    viz = MujocoVisualizer.from_robot(robot, mode="mirror")
    viz.run()
"""

import time
from typing import Literal, Optional

import mujoco
import mujoco.viewer
import numpy as np

from i2rt.robots.robot import Robot
from i2rt.robots.utils import ArmType, GripperType

Mode = Literal["interactive", "mirror"]


class MujocoVisualizer:
    """MuJoCo viewer that can mirror or interactively control an i2rt robot."""

    def __init__(self, model: mujoco.MjModel, robot: Robot, mode: Mode = "interactive"):
        self._model = model
        self._data = mujoco.MjData(model)
        self._robot = robot
        self._mode = mode

    # ---- factory constructors ------------------------------------------------

    @classmethod
    def from_xml(cls, xml_path: str, robot: Robot, mode: Mode = "interactive") -> "MujocoVisualizer":
        model = mujoco.MjModel.from_xml_path(xml_path)
        return cls(model, robot, mode)

    @classmethod
    def from_robot(cls, robot: "MotorChainRobot", mode: Mode = "interactive") -> "MujocoVisualizer":  # noqa: F821
        return cls.from_xml(robot.xml_path, robot, mode)

    # ---- public API ----------------------------------------------------------

    def update(self, joint_pos: np.ndarray) -> None:
        """Set joint positions and forward-compute kinematics."""
        n = min(len(joint_pos), self._model.nq)
        self._data.qpos[:n] = joint_pos[:n]
        mujoco.mj_forward(self._model, self._data)

    def _joint_name(self, qpos_idx: int) -> str:
        """Return the joint name for a given qpos index."""
        for i in range(self._model.njnt):
            if self._model.jnt_qposadr[i] == qpos_idx:
                name = self._model.joint(i).name
                return name if name else f"joint_{i}"
        return f"qpos[{qpos_idx}]"

    def run(self, dt: float = 0.02, step: float = 0.05) -> None:
        """Open the viewer and loop until the window is closed.

        If a robot was provided, joint positions are polled from
        ``robot.get_joint_pos()`` each iteration (passive mirror mode).
        Otherwise the viewer runs in interactive physics mode where joints
        can be moved via mouse perturbation or keyboard.

        Keyboard controls (always active):
            Tab / Right arrow : select next joint
            Left arrow        : select previous joint
            Up / Down arrows  : increase / decrease selected joint
            [ / ]             : halve / double the step size

        Mouse controls (interactive mode only):
            Double-click      : select a body
            Ctrl + right-drag : apply force to move the selected body
        """
        nq = self._model.nq
        selected = [0]
        step_size = [step]

        def _print_status():
            idx = selected[0]
            name = self._joint_name(idx)
            val = self._data.qpos[idx]
            print(
                f"\rJoint {idx}/{nq - 1}: {name} = {val:+.4f}  "
                f"(step={step_size[0]:.4f})",
                end="",
                flush=True,
            )

        def key_callback(keycode):
            if keycode in (258, 262):  # Tab or Right
                selected[0] = (selected[0] + 1) % nq
            elif keycode == 263:  # Left
                selected[0] = (selected[0] - 1) % nq
            elif keycode == 265:  # Up
                self._data.qpos[selected[0]] += step_size[0]
            elif keycode == 264:  # Down
                self._data.qpos[selected[0]] -= step_size[0]
            elif keycode == 93:  # ]
                step_size[0] *= 2
            elif keycode == 91:  # [
                step_size[0] /= 2
            else:
                return
            _print_status()

        if self._mode == "interactive":
            print("Interactive mode – move joints via the Joint panel or keyboard:")
            print("  Joint panel: use sliders in the right-side panel")
            print("  Keys:  Tab/Right=next  Left=prev  Up/Down=move  [/]=step size")
            _print_status()
            with mujoco.viewer.launch_passive(
                self._model, self._data, key_callback=key_callback
            ) as viewer:
                try:
                    while viewer.is_running():
                        mujoco.mj_forward(self._model, self._data)
                        viewer.sync()
                        time.sleep(dt)
                except KeyboardInterrupt:
                    pass
                finally:
                    print()  # newline after \r status line
        else:
            print("Mirror mode – reading joint positions from robot")
            print("  Keys: Tab/Right=next  Left=prev  Up/Down=move  [/]=step size")
            with mujoco.viewer.launch_passive(
                self._model, self._data, key_callback=key_callback
            ) as viewer:
                _print_status()
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
                finally:
                    print()  # newline after \r status line


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
    parser.add_argument(
        "--mode",
        type=str,
        default="interactive",
        choices=["interactive", "mirror"],
        help="interactive: move joints via GUI/keyboard; mirror: read joint positions from a robot",
    )
    parser.add_argument("--sim", action="store_true", help="Use a SimRobot instead of real hardware")
    args = parser.parse_args()

    from i2rt.robots.get_robot import get_yam_robot

    arm = ArmType.from_string_name(args.arm)
    gripper = GripperType.from_string_name(args.gripper)
    robot = get_yam_robot(arm_type=arm, gripper_type=gripper, sim=args.sim)
    viz = MujocoVisualizer.from_robot(robot, mode=args.mode)
    viz.run()
