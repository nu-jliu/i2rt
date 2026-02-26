"""Tests for robot assembly: arm + gripper XML combination and sim-robot creation."""

import xml.etree.ElementTree as ET

import numpy as np
import pytest

from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.sim_robot import SimRobot
from i2rt.robots.utils import ArmType, GripperType, combine_arm_and_gripper_xml

# ---------------------------------------------------------------------------
# Helpers / parametrize IDs
# ---------------------------------------------------------------------------

ALL_ARM_GRIPPER_COMBOS = [
    (arm, gripper) for arm in ArmType for gripper in GripperType
]

# YAM combos that work end-to-end with get_yam_robot(sim=True)
VALID_YAM_SIM_COMBOS = [
    (GripperType.CRANK_4310, 7),
    (GripperType.LINEAR_3507, 7),
    (GripperType.LINEAR_4310, 7),
    (GripperType.YAM_TEACHING_HANDLE, 6),
    (GripperType.NO_GRIPPER, 6),
]


def _combo_id(val):
    """Readable pytest ID for (ArmType, GripperType) tuples."""
    if isinstance(val, (ArmType, GripperType)):
        return val.value
    return str(val)


# ---------------------------------------------------------------------------
# 2a) XML combination — all arm x gripper combos
# ---------------------------------------------------------------------------


@pytest.mark.sim
@pytest.mark.parametrize("arm,gripper", ALL_ARM_GRIPPER_COMBOS, ids=_combo_id)
def test_combine_xml_all_combos(arm: ArmType, gripper: GripperType):
    """combine_arm_and_gripper_xml should produce valid XML for every combo."""
    out_path = combine_arm_and_gripper_xml(arm.get_xml_path(), gripper.get_xml_path())

    # Returns a real file path
    assert isinstance(out_path, str)
    assert out_path.endswith(".xml")

    # File is parseable XML
    tree = ET.parse(out_path)
    root = tree.getroot()

    # Contains a link_6 or link6 body (the end-effector attachment point)
    link6 = root.find(".//*[@name='link_6']")
    if link6 is None:
        link6 = root.find(".//*[@name='link6']")
    assert link6 is not None, "Combined XML must contain a body named 'link_6' or 'link6'"


# ---------------------------------------------------------------------------
# 2b) get_yam_robot(sim=True) — valid YAM combos
# ---------------------------------------------------------------------------


@pytest.mark.sim
@pytest.mark.parametrize(
    "gripper_type,expected_dofs",
    VALID_YAM_SIM_COMBOS,
    ids=lambda v: v.value if isinstance(v, GripperType) else str(v),
)
def test_get_yam_robot_sim(gripper_type: GripperType, expected_dofs: int):
    """get_yam_robot(sim=True) should return a working SimRobot for valid combos."""
    robot = get_yam_robot(sim=True, gripper_type=gripper_type)

    assert isinstance(robot, SimRobot)
    assert robot.num_dofs() == expected_dofs

    # Observations contain the expected keys
    obs = robot.get_observations()
    assert "joint_pos" in obs
    assert "joint_vel" in obs
    assert "joint_eff" in obs

    if expected_dofs == 7:
        assert "gripper_pos" in obs
        assert obs["joint_pos"].shape == (6,)
        assert obs["gripper_pos"].shape == (1,)
    else:
        assert obs["joint_pos"].shape == (expected_dofs,)

    # get_joint_pos returns the full state vector
    qpos = robot.get_joint_pos()
    assert qpos.shape == (expected_dofs,)

    robot.close()


# ---------------------------------------------------------------------------
# 2c) ARX_DEFAULT should raise NotImplementedError
# ---------------------------------------------------------------------------


@pytest.mark.sim
def test_get_yam_robot_sim_arx_default_raises():
    """ARX_DEFAULT gripper raises NotImplementedError (motor type undefined)."""
    with pytest.raises(NotImplementedError):
        get_yam_robot(sim=True, gripper_type=GripperType.ARX_DEFAULT)


# ---------------------------------------------------------------------------
# 2d) Command and read back joint positions
# ---------------------------------------------------------------------------


@pytest.mark.sim
@pytest.mark.parametrize(
    "gripper_type,expected_dofs",
    VALID_YAM_SIM_COMBOS,
    ids=lambda v: v.value if isinstance(v, GripperType) else str(v),
)
def test_sim_robot_command_and_read(gripper_type: GripperType, expected_dofs: int):
    """Commanding a joint position should be readable back (within joint limits)."""
    robot = get_yam_robot(sim=True, gripper_type=gripper_type)

    # Build a target inside joint limits (small positive values for arm joints)
    target = np.zeros(expected_dofs)
    target[:6] = 0.3  # safe value within all arm joint limits

    robot.command_joint_pos(target)
    readback = robot.get_joint_pos()

    np.testing.assert_allclose(readback, target, atol=1e-6)
    robot.close()


# ---------------------------------------------------------------------------
# 3) Real-hardware tests (skipped unless -m real)
# ---------------------------------------------------------------------------


@pytest.mark.real
@pytest.mark.parametrize(
    "gripper_type",
    [g for g, _ in VALID_YAM_SIM_COMBOS],
    ids=lambda v: v.value,
)
def test_get_yam_robot_real(gripper_type: GripperType):
    """get_yam_robot(sim=False) should return a MotorChainRobot on real hardware."""
    from i2rt.robots.motor_chain_robot import MotorChainRobot

    robot = get_yam_robot(sim=False, gripper_type=gripper_type)
    assert isinstance(robot, MotorChainRobot)

    obs = robot.get_observations()
    assert "joint_pos" in obs
    robot.close()
