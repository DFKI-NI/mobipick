"""The servo cable guard (#200) with the real cable model of mobipick_sim_cable_entanglement (amenable_ws; skipped
without it) and its test URDF."""
import math
import os
import sys

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))
CABLE = os.environ.get('CABLE_PKG', os.path.expanduser('~/ros1_ws/amenable_ws/src/mobipick_sim_cable_entanglement'))
sys.path.insert(0, os.path.join(CABLE, 'src'))

from mobipick_servo import guard  # noqa: E402

J = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
TRANSPORT = [0.0, -120.3, 139.2, -109.1, -90.0, 180.0]


def q(deg):
    return {j: math.radians(v) for j, v in zip(J, deg)}


@pytest.fixture(scope='module')
def stretch():
    cable_model = pytest.importorskip('mobipick_sim_cable_entanglement.cable_model')
    cfg = cable_model.config_from_dict(yaml.safe_load(open(os.path.join(CABLE, 'config', 'cable_model.yaml'))))
    cfg.rope.enabled = False
    urdf = open(os.path.join(CABLE, 'test', 'mobipick_sim_robot_description.urdf')).read()
    model = cable_model.CableModel(urdf, cfg)
    return lambda joints: model.evaluate(joints).max_stretch


def test_a_small_move_in_a_slack_pose_is_forwarded(stretch):
    verdict = guard.check(q(TRANSPORT), q(TRANSPORT[:5] + [182.0]), stretch)
    assert verdict.forward and verdict.stretch_cmd <= -0.115


def test_turning_wrist_3_down_is_stopped_where_the_cable_tightens_and_the_way_back_stays_open(stretch):
    # wrist_3 below ~75 deg tightens the wrist span gradually (-0.16 at 90, -0.12 at 75, -0.02 at 30 deg)
    last_ok, refused = TRANSPORT, None
    for w3 in range(175, 29, -5):
        cmd = TRANSPORT[:5] + [float(w3)]
        if not guard.check(q(last_ok), q(cmd), stretch).forward:
            refused = cmd
            break
        last_ok = cmd
    assert refused is not None and 55.0 <= refused[5] <= 80.0, refused
    assert stretch(q(refused)) > -0.115 >= stretch(q(last_ok))
    assert guard.check(q(last_ok), q(TRANSPORT[:5] + [last_ok[5] + 5.0]), stretch).forward  # back up: allowed


def test_a_tight_pose_may_always_move_out(stretch):
    tight = TRANSPORT[:5] + [40.0]  # already beyond the margin (e.g. left there by another motion)
    assert stretch(q(tight)) > -0.115
    assert guard.check(q(tight), q(TRANSPORT[:5] + [45.0]), stretch).forward  # less tight: allowed
    assert not guard.check(q(tight), q(TRANSPORT[:5] + [35.0]), stretch).forward  # tighter: refused


def test_wrist_3_window_blocks_winding_further_but_not_back():
    def flat(joints):  # cable slack everywhere: only the window decides
        return -0.2

    window = (math.radians(30.0), math.radians(300.0))
    assert not guard.check(q(TRANSPORT[:5] + [300.0]), q(TRANSPORT[:5] + [305.0]), flat, wrist_3_window=window).forward
    assert guard.check(q(TRANSPORT[:5] + [310.0]), q(TRANSPORT[:5] + [305.0]), flat, wrist_3_window=window).forward
    assert guard.check(q(TRANSPORT), q(TRANSPORT[:5] + [185.0]), flat, wrist_3_window=window).forward
