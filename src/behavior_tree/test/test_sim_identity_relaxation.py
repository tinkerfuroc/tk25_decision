"""Sim-mode identity relaxation for person_found() (GPSR_SIM_IDENTITY_RELAXED=1).

Sim persons carry no name identity -- the detector always labels them
"person" -- so a command like greetNameInRm's person_found(<Name>) gate
was rejecting a correct sim run (battery run family s2026-005/006/007,
2026-08-28) purely because the name wasn't in the detection labels. This
flag relaxes ONLY that specific labelled-mismatch path, and only for a
genuine person NAME argument (not a waving/gesture descriptor), and only
when at least one detection label is itself a person-class label. Every
other validator path -- object_seen, counted, and person_found's other
branches -- is unchanged; the flag is off by default so a real-robot
launch never sees this behaviour.

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 (ROS pytest plugins break
collection).
"""

from __future__ import annotations

from behavior_tree.GPSR.validators import Verdict, VerificationContext, check_all


def _check(text, *, evidence=None, env=None, monkeypatch):
    if env is not None:
        for key, value in env.items():
            monkeypatch.setenv(key, value)
    results, _facts = check_all([text], evidence or {}, VerificationContext(phase="postcondition"))
    assert len(results) == 1
    return results[0]


def test_flag_off_labelled_mismatch_is_still_invalid(monkeypatch):
    monkeypatch.delenv("GPSR_SIM_IDENTITY_RELAXED", raising=False)
    result = _check(
        "person_found(sarah)",
        evidence={"person_detection": {"objects": [{"label": "person"}]}},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.INVALID


def test_flag_on_person_label_is_valid_with_sim_reason(monkeypatch):
    result = _check(
        "person_found(sarah)",
        evidence={"person_detection": {"objects": [{"label": "person"}]}},
        env={"GPSR_SIM_IDENTITY_RELAXED": "1"},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.VALID
    assert result.confidence == 0.6
    assert result.evidence == "sim mode: person detected; name identity is not modelled in sim"


def test_flag_on_named_person_label_relaxes_via_class_token(monkeypatch):
    # A specialist label like "person_liam" carries a class token ("person")
    # plus a name token that isn't the requested one -- the relaxation must
    # key off ANY token of ANY label being in the person-class set, not
    # equality of the whole label with a bare "person"/"human"/... string.
    result = _check(
        "person_found(sarah)",
        evidence={"person_detection": {"objects": [{"label": "person_liam"}]}},
        env={"GPSR_SIM_IDENTITY_RELAXED": "1"},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.VALID
    assert result.confidence == 0.6
    assert result.evidence == "sim mode: person detected; name identity is not modelled in sim"


def test_flag_on_waving_persons_descriptor_label_relaxes_via_class_token(monkeypatch):
    # L4 (round-2 review): the token-based class check (any token of any
    # LABEL in _SIM_PERSON_CLASS_LABELS) widened the relaxation to also
    # fire for a "waving_persons" DETECTION LABEL -- distinct from the
    # person_found ARGUMENT naming a waving descriptor, which stays gated
    # by _is_person_name_arg (see test_flag_on_descriptor_argument_is_
    # unaffected below, requested="waving_person"). "waving_persons"
    # tokenises to "waving"/"persons", and "persons" is itself one of
    # _SIM_PERSON_CLASS_LABELS -- pinned here as ACCEPTED sim behaviour: a
    # waving-person label is still evidence of a person.
    result = _check(
        "person_found(sarah)",
        evidence={"person_detection": {"objects": [{"label": "waving_persons"}]}},
        env={"GPSR_SIM_IDENTITY_RELAXED": "1"},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.VALID
    assert result.confidence == 0.6
    assert result.evidence == "sim mode: person detected; name identity is not modelled in sim"


def test_flag_on_non_person_labels_stay_invalid(monkeypatch):
    result = _check(
        "person_found(sarah)",
        evidence={"person_detection": {"objects": [{"label": "chair"}]}},
        env={"GPSR_SIM_IDENTITY_RELAXED": "1"},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.INVALID


def test_flag_on_descriptor_argument_is_unaffected(monkeypatch):
    # "waving_person" is a specialist descriptor, not a name -- the
    # relaxation must not touch this path (it already has its own
    # provenance-gated VALID/UNKNOWN branch, tested in
    # test_gpsr_fact_validators.py).
    result = _check(
        "person_found(waving_person)",
        evidence={"person_detection": {"objects": [{"label": "person"}]}},
        env={"GPSR_SIM_IDENTITY_RELAXED": "1"},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.INVALID


def test_flag_on_does_not_leak_into_object_seen(monkeypatch):
    result = _check(
        "object_seen(mug)",
        evidence={"object_detection": {"objects": [{"label": "bowl"}]}},
        env={"GPSR_SIM_IDENTITY_RELAXED": "1"},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.INVALID
