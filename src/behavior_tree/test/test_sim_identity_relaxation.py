"""Sim-mode identity relaxation for person_found() (GPSR_SIM_IDENTITY_RELAXED=1).

Sim persons carry no name identity -- the detector always labels them
"person" -- so a command like greetNameInRm's person_found(<Name>) gate
was rejecting a correct sim run (battery run family s2026-005/006/007,
2026-08-28) purely because the name wasn't in the detection labels. This
flag relaxes ONLY that specific labelled-mismatch path, and only when at
least one detection label is itself a person-class label.

J13 (round-3 adversarial review, tier0 #4): the relaxation covers ANY
person_found() argument the sim cannot model identity for -- a NAME
("sarah") and a descriptor (gesture/pose/clothing, e.g. "person raising
their left arm") alike, since the sim's detector carries no more of one
than the other. Every other validator path -- object_seen, counted, and
person_found's other branches -- is unchanged; the flag is off by default
so a real-robot launch never sees this behaviour.

L2b (round-4 battery fix, runs 008/011): a VALID verdict here now ALSO
requires evidence that a pose was actually materialized
(``evidence["target_person_pose"]`` set, or ``person_provenance ==
"relaxed_generic"``) -- a leftover detection response with no pose used to
validate person_found() and then approach_person crashed with "Unsupported
type for gpsr/target_person_pose: NoneType" (runs 008/011). Every test below
that asserts VALID now supplies pose evidence for that reason.

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
        evidence={
            "person_detection": {"objects": [{"label": "person"}]},
            "target_person_pose": {"pose": "materialized"},
        },
        env={"GPSR_SIM_IDENTITY_RELAXED": "1"},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.VALID
    assert result.confidence == 0.6
    # J13 (round-3 adversarial review, tier0 #4): unified reason text for
    # any relaxed argument (name or descriptor) -- was "sim mode: person
    # detected; name identity is not modelled in sim".
    assert result.evidence == "sim mode: descriptor sarah not modelled"


def test_flag_on_named_person_label_relaxes_via_class_token(monkeypatch):
    # A specialist label like "person_liam" carries a class token ("person")
    # plus a name token that isn't the requested one -- the relaxation must
    # key off ANY token of ANY label being in the person-class set, not
    # equality of the whole label with a bare "person"/"human"/... string.
    result = _check(
        "person_found(sarah)",
        evidence={
            "person_detection": {"objects": [{"label": "person_liam"}]},
            "target_person_pose": {"pose": "materialized"},
        },
        env={"GPSR_SIM_IDENTITY_RELAXED": "1"},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.VALID
    assert result.confidence == 0.6
    assert result.evidence == "sim mode: descriptor sarah not modelled"


def test_flag_on_waving_persons_descriptor_label_relaxes_via_class_token(monkeypatch):
    # L4 (round-2 review): the token-based class check (any token of any
    # LABEL in _SIM_PERSON_CLASS_LABELS) widened the relaxation to also
    # fire for a "waving_persons" DETECTION LABEL -- distinct from the
    # person_found ARGUMENT naming a waving descriptor, which stays gated
    # by _is_person_name_arg (requested="waving_person",
    # test_flag_on_descriptor_argument_is_unaffected below). "waving_persons"
    # tokenises to "waving"/"persons", and "persons" is itself one of
    # _SIM_PERSON_CLASS_LABELS -- pinned here as ACCEPTED sim behaviour: a
    # waving-person label is still evidence of a person.
    result = _check(
        "person_found(sarah)",
        evidence={
            "person_detection": {"objects": [{"label": "waving_persons"}]},
            "target_person_pose": {"pose": "materialized"},
        },
        env={"GPSR_SIM_IDENTITY_RELAXED": "1"},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.VALID
    assert result.confidence == 0.6
    assert result.evidence == "sim mode: descriptor sarah not modelled"


def test_flag_on_non_person_labels_stay_invalid(monkeypatch):
    result = _check(
        "person_found(sarah)",
        evidence={"person_detection": {"objects": [{"label": "chair"}]}},
        env={"GPSR_SIM_IDENTITY_RELAXED": "1"},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.INVALID


def test_flag_on_descriptor_argument_is_unaffected(monkeypatch):
    # "waving_person" is the SPECIALIST descriptor, not a plain gesture/pose
    # one -- the generic relaxation must not touch this path (it already
    # has its own provenance-gated VALID/UNKNOWN branch, tested in
    # test_gpsr_fact_validators.py). This holds even WITH a materialized
    # pose (L2b) -- the pose requirement is a NECESSARY, not sufficient,
    # condition; _is_person_name_arg still excludes this descriptor.
    result = _check(
        "person_found(waving_person)",
        evidence={
            "person_detection": {"objects": [{"label": "person"}]},
            "target_person_pose": {"pose": "materialized"},
        },
        env={"GPSR_SIM_IDENTITY_RELAXED": "1"},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.INVALID


def test_flag_off_waving_descriptor_argument_with_pose_stays_invalid(monkeypatch):
    # L3: the flag-off path is completely untouched -- a materialized pose
    # alone is not enough without the flag; strict "waving_specialist
    # provenance required" behaviour still applies.
    monkeypatch.delenv("GPSR_SIM_IDENTITY_RELAXED", raising=False)
    result = _check(
        "person_found(waving_person)",
        evidence={
            "person_detection": {"objects": [{"label": "person"}]},
            "target_person_pose": {"pose": "materialized"},
        },
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.INVALID


def test_flag_on_gesture_descriptor_argument_is_now_relaxed(monkeypatch):
    # J13 (round-3 adversarial review, tier0 #4): "person raising their
    # left arm" is a gesture descriptor the sim does not model at all --
    # before this fix _is_person_name_arg rejected it purely for
    # containing "_"/"person", so this could NEVER be VALID under the
    # relaxation flag even though the sim's detector is exactly as blind
    # to gestures as it is to names.
    result = _check(
        "person_found(person raising their left arm)",
        evidence={
            "person_detection": {"objects": [{"label": "person"}]},
            "target_person_pose": {"pose": "materialized"},
        },
        env={"GPSR_SIM_IDENTITY_RELAXED": "1"},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.VALID
    assert result.confidence == 0.6
    assert result.evidence == (
        "sim mode: descriptor person_raising_their_left_arm not modelled"
    )


def test_flag_on_missing_pose_stays_invalid_the_008_regression(monkeypatch):
    # L2b (round-4 battery fix, runs 008/011): the exact 008/011 shape -- a
    # generic person detection is on the blackboard (a leftover
    # BtNode_ScanForGeneralist response) but NOTHING materialized a pose or
    # relaxed_generic provenance. Regression test for the NoneType cascade:
    # this must be INVALID so approach_person is never reached with no pose.
    result = _check(
        "person_found(sarah)",
        evidence={"person_detection": {"objects": [{"label": "person"}]}},
        env={"GPSR_SIM_IDENTITY_RELAXED": "1"},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.INVALID


def test_flag_on_provenance_alone_is_sufficient_without_pose_key(monkeypatch):
    # L2b: person_provenance == "relaxed_generic" is an alternative to a
    # target_person_pose evidence key (BtNode_ExtractDetection's relaxed
    # mode writes both to the SAME blackboard write, but the evidence dict
    # only needs one of the two signals).
    result = _check(
        "person_found(sarah)",
        evidence={
            "person_detection": {"objects": [{"label": "person"}]},
            "person_provenance": "relaxed_generic",
        },
        env={"GPSR_SIM_IDENTITY_RELAXED": "1"},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.VALID
    assert result.confidence == 0.6


def test_waving_specialist_success_path_valid_under_both_flags(monkeypatch):
    # The dedicated waving_specialist provenance branch (validators._verify,
    # ahead of/independent from the generic relaxed-degrade branch under
    # test in this file) is untouched by L2/L3 either way.
    waving = {"waving_persons": [{"x": 1}], "person_provenance": "waving_specialist"}
    monkeypatch.delenv("GPSR_SIM_IDENTITY_RELAXED", raising=False)
    off_result = _check("person_found(waving_person)", evidence=waving, monkeypatch=monkeypatch)
    assert off_result.verdict is Verdict.VALID
    on_result = _check(
        "person_found(waving_person)", evidence=waving,
        env={"GPSR_SIM_IDENTITY_RELAXED": "1"}, monkeypatch=monkeypatch,
    )
    assert on_result.verdict is Verdict.VALID


def test_flag_off_gesture_descriptor_argument_stays_invalid(monkeypatch):
    monkeypatch.delenv("GPSR_SIM_IDENTITY_RELAXED", raising=False)
    result = _check(
        "person_found(person raising their left arm)",
        evidence={"person_detection": {"objects": [{"label": "person"}]}},
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
