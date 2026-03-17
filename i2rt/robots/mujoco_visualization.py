"""Interactive MuJoCo joint visualization with sliders.

Usage:
    uv run python -m i2rt.robots.mujoco_visualization --arm yam --gripper crank_4310 --sim
    uv run python -m i2rt.robots.mujoco_visualization --arm big_yam --sim
    uv run python -m i2rt.robots.mujoco_visualization --arm yam --gripper linear_4310 --channel can0
"""

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


def _inject_actuators(xml_path: str) -> tuple[str, mujoco.MjModel]:
    """Add position actuators for every joint so the viewer shows Control panel sliders."""
    model = mujoco.MjModel.from_xml_path(xml_path)
    tree = ET.parse(xml_path)
    root = tree.getroot()

    actuator_el = root.find("actuator")
    if actuator_el is None:
        actuator_el = ET.SubElement(root, "actuator")

    for j in range(model.njnt):
        jnt_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if jnt_name is None:
            continue
        act = ET.SubElement(actuator_el, "position")
        act.set("name", f"slider_{jnt_name}")
        act.set("joint", jnt_name)
        act.set("kp", "100")
        if model.jnt_limited[j]:
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


def _has_self_collision(model: mujoco.MjModel, data: mujoco.MjData, target_ctrl: np.ndarray, n: int) -> bool:
    """Return True if target joint positions would cause self-collision."""
    data.qpos[:n] = target_ctrl[:n]
    _denormalize_slide_joints(model, data, n)
    _enforce_eq_constraints(model, data)
    mujoco.mj_forward(model, data)
    for i in range(data.ncon):
        c = data.contact[i]
        if c.dist >= -1e-3:
            continue
        # Ignore contacts involving plane geoms (ground floor)
        if model.geom_type[c.geom1] == mujoco.mjtGeom.mjGEOM_PLANE or model.geom_type[c.geom2] == mujoco.mjtGeom.mjGEOM_PLANE:
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


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive MuJoCo joint visualization")
    parser.add_argument("--arm", default="yam", choices=[a.value for a in ArmType])
    parser.add_argument("--gripper", default="no_gripper", choices=[g.value for g in GripperType])
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--channel", default="can0")
    args = parser.parse_args()

    arm_type = ArmType.from_string_name(args.arm)
    gripper_type = GripperType.from_string_name(args.gripper)

    robot = get_yam_robot(
        channel=args.channel,
        arm_type=arm_type,
        gripper_type=gripper_type,
        sim=args.sim,
    )

    _, viz_model = _inject_actuators(robot.xml_path)
    viz_data = mujoco.MjData(viz_model)
    check_data = mujoco.MjData(viz_model)
    n_dofs = robot.num_dofs()

    print(f"Loaded {args.arm} + {args.gripper} ({n_dofs} DOFs, sim={args.sim})")
    print("Use the Control panel sliders (right side) to move joints.")

    in_collision = False
    with mujoco.viewer.launch_passive(viz_model, viz_data) as viewer:
        while viewer.is_running():
            n_ctrl = min(n_dofs, viz_model.nu)
            ctrl = np.zeros(n_dofs)
            ctrl[:n_ctrl] = viz_data.ctrl[:n_ctrl]

            if _has_self_collision(viz_model, check_data, ctrl, n_ctrl):
                if not in_collision:
                    print("Collision detected - command blocked")
                    in_collision = True
            else:
                robot.command_joint_pos(ctrl)
                if in_collision:
                    print("Collision cleared - commands resumed")
                    in_collision = False

            qpos = robot.get_joint_pos()
            n = min(len(qpos), viz_model.nq)
            viz_data.qpos[:n] = qpos[:n]

            _denormalize_slide_joints(viz_model, viz_data, n)
            _enforce_eq_constraints(viz_model, viz_data)
            mujoco.mj_forward(viz_model, viz_data)

            viewer.sync()
            time.sleep(0.01)

    robot.close()


if __name__ == "__main__":
    main()
