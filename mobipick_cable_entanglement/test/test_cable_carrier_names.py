""": the arm's cable guide is the cable carrier; the names of before (tube, rope, disc chain) still load and warn
once each: config keys, the wrist_anchor value tube_tip and the Python API."""
import copy
import os
import sys
import warnings

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from mobipick_cable_entanglement import cable_model, path_check  # noqa: E402
from mobipick_cable_entanglement.cable_model import config_from_dict  # noqa: E402

CONFIG = yaml.safe_load(open(os.path.join(HERE, "..", "config", "cable_model.yaml")))


def old_keys(d):
    """the config as it was written before """
    d = copy.deepcopy(d)
    d["rope"] = d.pop("cable_carrier")
    d["rope"]["disc_link"] = d["rope"].pop("link_chain")
    d["tube_radius"] = d.pop("cable_carrier_radius")
    for s in d["spans"].values():
        for new, old in (("carrier_start", "rope_start"), ("carrier_end", "rope_end"),
                         ("carrier_free_length", "rope_free_length"), ("carrier_rest_side", "rope_rest_side")):
            if new in s:
                s[old] = s.pop(new)
    d["wrist_anchor"] = "tube_tip"
    return d


@pytest.fixture(autouse=True)
def fresh_warnings():
    cable_model._DEPRECATED_SEEN.clear()
    yield


def test_the_shipped_config_has_only_the_new_names():
    text = open(os.path.join(HERE, "..", "config", "cable_model.yaml")).read()
    for old in ("\nrope:", "rope_start", "rope_end", "rope_free_length", "rope_rest_side", "disc_link", "tube_radius",
                "tube_tip"):
        assert old not in text
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        config_from_dict(CONFIG, wrist_anchor="carrier_tip")


def test_an_old_key_config_loads_the_same_model_and_warns_once_per_name():
    new = config_from_dict(copy.deepcopy(CONFIG), wrist_anchor="support")
    with pytest.warns(FutureWarning) as caught:
        old = config_from_dict(old_keys(CONFIG), wrist_anchor="support")
        config_from_dict(old_keys(CONFIG), wrist_anchor="support")   # the second load warns no more
    names = [str(w.message).split("'")[1] for w in caught]
    assert len(names) == len(set(names))
    for old_name in ("rope", "tube_radius", "cable_carrier.disc_link", "spans.<span>.rope_free_length",
                     "spans.<span>.rope_start", "spans.<span>.rope_end", "spans.<span>.rope_rest_side",
                     "wrist_anchor tube_tip"):
        assert old_name in names
    assert repr(old) == repr(new)


def test_the_old_wrist_anchor_value_means_the_carrier_tip():
    with pytest.warns(FutureWarning):
        assert cable_model.resolve_wrist_anchor("tube_tip") == "carrier_tip"
    old = config_from_dict(old_keys(CONFIG), wrist_anchor="tube_tip")
    assert old.wrist_anchor == "carrier_tip"


def test_the_old_python_names_still_work():
    cfg = config_from_dict(copy.deepcopy(CONFIG))
    with pytest.warns(FutureWarning):
        assert cfg.rope is cfg.cable_carrier
    with pytest.warns(FutureWarning):
        assert cfg.tube_radius == cfg.cable_carrier_radius
    wrist = next(s for s in cfg.spans if s.name == "wrist")
    with pytest.warns(FutureWarning):
        assert wrist.rope_free_length == wrist.carrier_free_length
    wrist.rope_free_length = 0.5   # the same old name: warned once already
    assert wrist.carrier_free_length == 0.5
    with pytest.warns(FutureWarning):
        assert cfg.cable_carrier.disc_link is cfg.cable_carrier.link_chain
    assert cable_model.RopeConfig is cable_model.CableCarrierConfig
    assert cable_model.DiscLinkConfig is cable_model.LinkChainConfig
    assert cable_model.DiscChains is cable_model.CarrierLinkChains
    assert path_check.RopeVerdict is path_check.CarrierVerdict
