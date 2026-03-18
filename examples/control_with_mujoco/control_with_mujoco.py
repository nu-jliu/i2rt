"""MuJoCo control interface for i2rt robots.

Starts in gravity-comp (vis-only) mode, mirroring the robot's joint state.
Press SPACE in the viewer to toggle mocap control mode, then double-click
the target marker, ctrl+right-drag to translate, ctrl+left-drag to rotate.

With --viz, launches interactive slider-based joint visualization with
self-collision checking instead of mocap IK control.

Usage:
    python examples/control_with_mujoco/control_with_mujoco.py --sim
    python examples/control_with_mujoco/control_with_mujoco.py --arm big_yam --gripper linear_4310 --sim
    python examples/control_with_mujoco/control_with_mujoco.py --channel can0
    python examples/control_with_mujoco/control_with_mujoco.py --sim --viz
    python examples/control_with_mujoco/control_with_mujoco.py --arm yam --gripper crank_4310 --sim --viz
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import os
import tempfile
import time
import xml.etree.ElementTree as ET

import mujoco
import mujoco.viewer
import numpy as np

from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.utils import ArmType, GripperType
from i2rt.utils.mujoco_control_interface import MujocoControlInterface


def _inject_actuators(xml_path: str, n_dofs: int, gripper_index: int | None = None) -> tuple[str, mujoco.MjModel]:
    """Add position actuators for controllable joints so the viewer shows Control panel sliders."""
    model = mujoco.MjModel.from_xml_path(xml_path)
    tree = ET.parse(xml_path)
    root = tree.getroot()

    actuator_el = root.find("actuator")
    if actuator_el is None:
        actuator_el = ET.SubElement(root, "actuator")

    for j in range(min(n_dofs, model.njnt)):
        jnt_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if jnt_name is None:
            continue
        act = ET.SubElement(actuator_el, "position")
        act.set("name", "gripper" if j == gripper_index else jnt_name)
        act.set("joint", jnt_name)
        act.set("kp", "100")
        if j == gripper_index:
            act.set("ctrlrange", "0 1")
            act.set("ctrllimited", "true")
        elif model.jnt_limited[j]:
            lo, hi = model.jnt_range[j]
            act.set("ctrlrange", f"{lo} {hi}")
            act.set("ctrllimited", "true")

    fd, tmp_path = tempfile.mkstemp(suffix=".xml", prefix="i2rt_viz_", dir="/tmp")
    os.close(fd)
    tree.write(tmp_path, xml_declaration=True)
    viz_model = mujoco.MjModel.from_xml_path(tmp_path)
    return tmp_path, viz_model


def _denormalize_slide_joints(model: mujoco.MjModel, data: mujoco.MjData, n_set: int) -> None:
    """Scale normalized [0,1] slide joint values to physical range."""
    for j in range(model.njnt):
        adr = model.jnt_qposadr[j]
        if adr >= n_set:
            continue
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_SLIDE:
            lo, hi = model.jnt_range[j]
            data.qpos[adr] = lo + data.qpos[adr] * (hi - lo)


def _has_self_collision(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    target_ctrl: np.ndarray,
    n: int,
    gripper_index: int | None = None,
    gripper_limits: np.ndarray | None = None,
) -> bool:
    """Return True if target joint positions would cause self-collision."""
    data.qpos[:n] = target_ctrl[:n]
    # Convert normalized 0-1 gripper value to raw MuJoCo joint angle
    if gripper_index is not None and gripper_limits is not None and gripper_index < n:
        adr = model.jnt_qposadr[gripper_index]
        lo, hi = float(gripper_limits[0]), float(gripper_limits[1])
        data.qpos[adr] = lo + data.qpos[adr] * (hi - lo)
    _denormalize_slide_joints(model, data, n)
    _enforce_eq_constraints(model, data)
    mujoco.mj_forward(model, data)
    for i in range(data.ncon):
        c = data.contact[i]
        if c.dist >= -1e-3:
            continue
        # Ignore contacts involving plane geoms (ground floor)
        if (
            model.geom_type[c.geom1] == mujoco.mjtGeom.mjGEOM_PLANE
            or model.geom_type[c.geom2] == mujoco.mjtGeom.mjGEOM_PLANE
        ):
            continue
        # Ignore contacts between adjacent bodies (parent-child links)
        b1 = model.geom_bodyid[c.geom1]
        b2 = model.geom_bodyid[c.geom2]
        if model.body_parentid[b1] == b2 or model.body_parentid[b2] == b1:
            continue
        return True
    return False


def _enforce_eq_constraints(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Project qpos to satisfy joint equality constraints (coupled fingers)."""
    for i in range(model.neq):
        if model.eq_type[i] != mujoco.mjtEq.mjEQ_JOINT:
            continue
        adr1 = model.jnt_qposadr[model.eq_obj1id[i]]
        adr2 = model.jnt_qposadr[model.eq_obj2id[i]]
        coef = model.eq_data[i, :5]
        q1 = data.qpos[adr1]
        data.qpos[adr2] = np.polyval(coef[::-1], q1)


def run_viz(robot: object, sim: bool, arm_name: str, gripper_name: str) -> None:
    """Run interactive slider-based joint visualization with self-collision checking."""
    robot_info = robot.get_robot_info()
    gripper_index: int | None = robot_info.get("gripper_index")
    gripper_limits = robot_info.get("gripper_limits")
    is_sim: bool = robot_info.get("sim", False)

    n_dofs = robot.num_dofs()
    _, viz_model = _inject_actuators(robot.xml_path, n_dofs=n_dofs, gripper_index=gripper_index)
    viz_data = mujoco.MjData(viz_model)
    check_data = mujoco.MjData(viz_model)

    print(f"Loaded {arm_name} + {gripper_name} ({n_dofs} DOFs, sim={sim})")
    print("Use the Control panel sliders (right side) to move joints.")

    in_collision = False
    with mujoco.viewer.launch_passive(viz_model, viz_data, show_left_ui=False) as viewer:
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
        while viewer.is_running():
            n_ctrl = min(n_dofs, viz_model.nu)
            ctrl = np.zeros(n_dofs)
            ctrl[:n_ctrl] = viz_data.ctrl[:n_ctrl]

            if _has_self_collision(viz_model, check_data, ctrl, n_ctrl, gripper_index, gripper_limits):
                if not in_collision:
                    print("Collision detected - command blocked")
                    in_collision = True
            else:
                cmd = ctrl.copy()
                if is_sim and gripper_index is not None and gripper_limits is not None:
                    # SimRobot expects raw joint values; convert 0-1 → raw
                    lo, hi = float(gripper_limits[0]), float(gripper_limits[1])
                    cmd[gripper_index] = lo + ctrl[gripper_index] * (hi - lo)
                # MotorChainRobot expects 0-1 for gripper (JointMapper handles it)
                robot.command_joint_pos(cmd)
                if in_collision:
                    print("Collision cleared - commands resumed")
                    in_collision = False

            qpos = robot.get_joint_pos()
            n = min(len(qpos), viz_model.nq)
            viz_data.qpos[:n] = qpos[:n]

            # MotorChainRobot returns 0-1 for gripper; convert to raw for MuJoCo rendering
            if not is_sim and gripper_index is not None and gripper_limits is not None:
                lo, hi = float(gripper_limits[0]), float(gripper_limits[1])
                viz_data.qpos[gripper_index] = lo + qpos[gripper_index] * (hi - lo)

            _denormalize_slide_joints(viz_model, viz_data, n)
            _enforce_eq_constraints(viz_model, viz_data)
            mujoco.mj_forward(viz_model, viz_data)

            viewer.sync()
            time.sleep(0.01)

    robot.close()


if __name__ == "__main__":
    arm_choices = [a.value for a in ArmType]
    gripper_choices = [g.value for g in GripperType]

    parser = argparse.ArgumentParser(
        description="MuJoCo control interface for i2rt robots",
    )
    parser.add_argument("--arm", type=str, default="yam", choices=arm_choices)
    parser.add_argument("--gripper", type=str, default="linear_4310", choices=gripper_choices)
    parser.add_argument("--channel", type=str, default="can0", help="CAN channel")
    parser.add_argument("--sim", action="store_true", help="Use SimRobot")
    parser.add_argument("--dt", type=float, default=0.02, help="Loop timestep (s)")
    parser.add_argument("--site", type=str, default=None, help="EE site name (auto-detected if omitted)")
    parser.add_argument("--viz", action="store_true", help="Slider visualization mode instead of mocap IK control")
    args = parser.parse_args()

    arm = ArmType.from_string_name(args.arm)
    gripper = GripperType.from_string_name(args.gripper)

    robot = get_yam_robot(
        channel=args.channel,
        arm_type=arm,
        gripper_type=gripper,
        sim=args.sim,
    )

    if args.viz:
        run_viz(robot, sim=args.sim, arm_name=args.arm, gripper_name=args.gripper)
    else:
        # Teaching handle only has tcp_site; all grippers with fingers have grasp_site.
        if args.site is not None:
            site = args.site
        elif gripper == GripperType.YAM_TEACHING_HANDLE:
            site = "tcp_site"
        else:
            site = "grasp_site"

        iface = MujocoControlInterface.from_robot(robot, ee_site=site, dt=args.dt)
        iface.run()
