"""Physical-truth verdict for tier-2 bench runs (bench/truth.py + tier2 wiring).

The executor's postcondition gates score BELIEFS (grasp reported success, place opened the
gripper, a count of 0 was "counted"); these tests build synthetic run dirs with a
``run_truth.json`` written by the sim probe and assert the verdict only PASSes when physics
confirms it.

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 (ROS pytest plugins break collection).
"""
from __future__ import annotations

import json
import sys
import textwrap
from pathlib import Path

import pytest

from behavior_tree.GPSR.bench import tier2, truth
from behavior_tree.GPSR.bench.corpus import CorpusEntry
from behavior_tree.GPSR.bench.report import BenchResult, write_report
from behavior_tree.GPSR.bench.tier2 import run_tier2

HERE = Path(__file__).resolve().parent
CONSTANTS = HERE.parent / "behavior_tree" / "GPSR" / "constants.rcw2026.json"
EXAMPLE_RUNS = HERE.parent / "behavior_tree" / "GPSR" / "gpsr_runs" / "bench"

SIDE_TABLE = {"surface_xyz": [0.0, -5.0, 0.6], "half_extent_xy": [0.4, 0.3]}
SIDE_TABLE_02 = {"surface_xyz": [0.5, 1.75, 0.6], "half_extent_xy": [0.4, 0.3]}
COMMAND_POINT = json.loads(CONSTANTS.read_text())["possible_poses"]["command_point"]["point"]


# --------------------------------------------------------------------------- builders

def _events(run_dir: Path, targets: list[dict], gates: list[tuple[int, str, str]] = ()) -> None:
    d = run_dir / "debug" / "traj-1"
    d.mkdir(parents=True, exist_ok=True)
    lines = [{"event_type": "split.accepted", "task_id": "traj-1/task-1",
              "payload": {"slot": 0, "targets": targets}}]
    for index, fact, verdict in gates:
        lines.append({"event_type": "gate.verified", "task_id": "traj-1/task-1",
                      "payload": {"slot": 0, "target_index": index, "phase": "postcondition",
                                  "fact": fact, "verdict": verdict, "reason": "x"}})
    lines.append({"event_type": "task.finished", "task_id": "traj-1/task-1",
                  "payload": {"status": "succeeded", "reason": ""}})
    (d / "events.jsonl").write_text("\n".join(json.dumps(e) for e in lines) + "\n")


def _spawned(run_dir: Path, entities: list[tuple[str, str]], skipped: list[tuple[str, str]] = (),
             poses: dict[str, str] | None = None) -> None:
    data = {
        "skipped": [{"id": i, "reason": "already present", "where": w} for i, w in skipped],
        "not_attempted": [],
        "entities": [{"id": i, "asset_key": "x", "where": w, "entity_name": f"/World/Scenario/{i}",
                      "ok": True, "xyz": [0, 0, 0], "cleared": True} for i, w in entities],
    }
    (run_dir / "spawned.json").write_text(json.dumps(data))
    items = []
    for i, w in list(entities) + list(skipped):
        kind = "person" if "person" in i else "object"
        item = {"id": i, "kind": kind, "name": truth.object_key_of(i),
                "room": w if kind == "person" else "", "spot": "" if kind == "person" else w}
        if poses and i in poses:
            item["pose"] = poses[i]
        items.append(item)
    (run_dir / "scene-plan.json").write_text(json.dumps({"schema_version": 1, "items": items}))


def _obj(first, last, retained=False, lift=0.0):
    # Bench-spawned entities ALWAYS publish ever_retained=False (the probe only evaluates
    # retained_by_gripper for the first scenario-declared object): grasp truth is motion-based.
    return {"first_xyz": list(first), "last_xyz": list(last), "ever_retained": retained,
            "retained_at_end": False, "max_lift_m": lift,
            "max_xy_disp_m": ((last[0] - first[0]) ** 2 + (last[1] - first[1]) ** 2) ** 0.5,
            "samples": 10}


def _truth(run_dir: Path, objects: dict, robot=None, surfaces=None, persons=None, any_retained="auto") -> None:
    if any_retained == "auto":
        any_retained = any(o.get("ever_retained") for o in objects.values()) if objects else None
    data = {"schema_version": 1, "samples": 10, "any_object_retained": any_retained,
            "objects": objects,
            "robot": {"last_xyz": list(robot), "last_yaw_deg": 0.0, "samples": 10} if robot else None,
            "surfaces": surfaces or {}, "persons": persons or {}}
    (run_dir / "run_truth.json").write_text(json.dumps(data))


def _target(idx, post, obj="", loc=""):
    return {"id": f"t{idx}", "desc": f"target {idx}", "object": obj, "location": loc,
            "depends_on": [], "preconditions": [], "postconditions": post}


def _checks(run_dir, targets):
    return {c["fact"]: c for c in truth.physical_checks(run_dir, targets, constants=CONSTANTS)}


# --------------------------------------------------------------------------- held()

def test_held_fails_when_the_object_never_moved(tmp_path):
    _truth(tmp_path, {"cmd_soup_0": _obj((1, 2, 0.7), (1, 2, 0.7))})
    c = _checks(tmp_path, [_target(0, ["held(soup)"], obj="soup", loc="shelf")])["held(soup)"]
    assert c["verdict"] == "FAIL"
    assert "never retained by gripper" in c["reason"] and "never moved" in c["reason"]


def test_held_passes_on_lift_alone_for_a_category_member_by_prim_path(tmp_path):
    # ever_retained is False for BOTH (bench-spawned) -- the 0.3 m lift decides.
    _truth(tmp_path, {"/World/Scenario/cmd_sugar_box_0": _obj((1, 2, 0.7), (1, 2, 1.0), lift=0.3),
                      "/World/Scenario/cmd_banana_0": _obj((1, 2, 0.7), (1, 2, 0.7))})
    c = _checks(tmp_path, [_target(0, ["held(food)"], obj="food")])["held(food)"]
    assert c["verdict"] == "PASS"
    assert "cmd_sugar_box_0" in c["reason"] and "lifted/carried" in c["reason"]


def test_held_passes_when_carried_sideways_with_a_small_lift(tmp_path):
    _truth(tmp_path, {"cmd_soup_0": _obj((1, 2, 0.7), (1.4, 2, 0.7), lift=0.03)})
    assert _checks(tmp_path, [_target(0, ["held(soup)"], obj="soup")])["held(soup)"]["verdict"] == "PASS"


def test_held_fails_when_nudged_but_never_lifted(tmp_path):
    _truth(tmp_path, {"cmd_soup_0": _obj((1, 2, 0.7), (1.08, 2, 0.7), lift=0.0)})
    c = _checks(tmp_path, [_target(0, ["held(soup)"], obj="soup")])["held(soup)"]
    assert c["verdict"] == "FAIL"
    assert "nudged but never lifted" in c["reason"]


def test_held_uses_ever_retained_true_as_an_extra_pass_signal(tmp_path):
    rec = {"first_xyz": [1, 2, 0.7], "last_xyz": [1, 2, 0.7], "ever_retained": True, "samples": 3}
    _truth(tmp_path, {"cmd_soup_0": rec})
    c = _checks(tmp_path, [_target(0, ["held(soup)"], obj="soup")])["held(soup)"]
    assert c["verdict"] == "PASS" and "retained by the gripper" in c["reason"]


def test_held_unknown_without_motion_data_and_ever_retained_false(tmp_path):
    rec = {"first_xyz": [1, 2, 0.7], "last_xyz": [1, 2, 0.7], "ever_retained": False, "samples": 3}
    _truth(tmp_path, {"cmd_soup_0": rec})
    c = _checks(tmp_path, [_target(0, ["held(soup)"], obj="soup")])["held(soup)"]
    assert c["verdict"] == "UNKNOWN" and "no motion data" in c["reason"]


def test_held_unknown_when_no_truth_object_matches(tmp_path):
    _truth(tmp_path, {"cmd_banana_0": _obj((1, 2, 0.7), (1, 2, 0.7), False)}, any_retained=None)
    c = _checks(tmp_path, [_target(0, ["held(mug)"], obj="mug")])["held(mug)"]
    assert c["verdict"] == "UNKNOWN"


def test_held_fails_on_legacy_grasp_truth_closed_on_air(tmp_path):
    (tmp_path / "grasp_truth.json").write_text(json.dumps({"any_object_retained": False, "max_xy_disp_m": 0.004}))
    c = _checks(tmp_path, [_target(0, ["held(soup)"], obj="soup")])["held(soup)"]
    assert c["verdict"] == "FAIL"
    assert "4mm" in c["reason"]


# --------------------------------------------------------------------------- placed()

def test_placed_passes_when_the_carried_object_rests_on_the_surface(tmp_path):
    _truth(tmp_path, {"cmd_soup_0": _obj((-3.6, 0.3, 1.07), (0.1, -5.1, 0.63), lift=0.2)},
           surfaces={"side_table": SIDE_TABLE})
    c = _checks(tmp_path, [_target(0, ["placed(soup,side_table)"], obj="soup", loc="side_table")])
    assert c["placed(soup,side_table)"]["verdict"] == "PASS"


def test_placed_fails_when_the_object_was_dropped_on_the_floor(tmp_path):
    _truth(tmp_path, {"cmd_soup_0": _obj((-3.6, 0.3, 1.07), (0.2, -5.0, 0.03), lift=0.2)},
           surfaces={"side_table": SIDE_TABLE})
    c = _checks(tmp_path, [_target(0, ["placed(soup,side_table)"], obj="soup", loc="side_table")])
    assert c["placed(soup,side_table)"]["verdict"] == "FAIL"
    assert "dz=-0.57" in c["placed(soup,side_table)"]["reason"]
    assert "below the surface" in c["placed(soup,side_table)"]["reason"]


def test_placed_ignores_an_untouched_object_already_on_the_destination(tmp_path):
    # findObjInRoom fix3: the food was spawned ON side_table_02 and never moved; "placed" must
    # not PASS just because a never-retained candidate happens to rest on the destination.
    _truth(tmp_path, {"cmd_soup_0": _obj((0.79, 1.75, 0.61), (0.79, 1.75, 0.61), False),
                      "cmd_sugar_box_0": _obj((0.61, 1.75, 0.61), (0.61, 1.75, 0.61), False)},
           surfaces={"side_table_02": SIDE_TABLE_02})
    c = _checks(tmp_path, [_target(0, ["placed(food,side_table_02)"], obj="food", loc="side_table_02")])
    assert c["placed(food,side_table_02)"]["verdict"] == "FAIL"
    assert "never retained" in c["placed(food,side_table_02)"]["reason"]


def test_placed_unknown_when_the_surface_is_not_in_truth(tmp_path):
    _truth(tmp_path, {"cmd_soup_0": _obj((0, 0, 0.7), (0, 0, 0.7), True)}, surfaces={})
    c = _checks(tmp_path, [_target(0, ["placed(soup,sofa)"], obj="soup", loc="sofa")])
    assert c["placed(soup,sofa)"]["verdict"] == "UNKNOWN"


# --------------------------------------------------------------------------- delivered()

def test_delivered_passes_when_retained_and_robot_ended_at_the_operator(tmp_path):
    # Handed over: released at the operator, so it ends low (sim has no hand).
    end = (COMMAND_POINT["x"] + 0.5, COMMAND_POINT["y"] - 0.3, 0.05)
    _truth(tmp_path, {"cmd_soup_0": _obj((-3.6, 0.3, 1.07), end, True, lift=0.2)},
           robot=(COMMAND_POINT["x"] + 0.4, COMMAND_POINT["y"] - 0.3, 0.0))
    c = _checks(tmp_path, [_target(0, ["delivered(soup,operator)"], obj="soup", loc="start_position")])
    assert c["delivered(soup,operator)"]["verdict"] == "PASS"


def test_delivered_fails_when_robot_ended_far_from_the_operator(tmp_path):
    _truth(tmp_path, {"cmd_soup_0": _obj((-3.6, 0.3, 1.07), (0, 0, 0.8), True, lift=0.2)},
           robot=(COMMAND_POINT["x"] + 3.0, COMMAND_POINT["y"], 0.0))
    c = _checks(tmp_path, [_target(0, ["delivered(soup,me)"], obj="soup")])
    assert c["delivered(soup,me)"]["verdict"] == "FAIL"
    assert "3.00 m" in c["delivered(soup,me)"]["reason"]


def test_delivered_fails_when_nothing_was_ever_retained(tmp_path):
    _truth(tmp_path, {"cmd_soup_0": _obj((0, 0, 0.7), (0, 0, 0.7), False)},
           robot=(COMMAND_POINT["x"], COMMAND_POINT["y"], 0.0))
    c = _checks(tmp_path, [_target(0, ["delivered(soup,operator)"], obj="soup")])
    assert c["delivered(soup,operator)"]["verdict"] == "FAIL"
    assert "nothing to deliver" in c["delivered(soup,operator)"]["reason"]


def test_delivered_to_a_person_in_a_room_uses_truth_persons(tmp_path):
    _truth(tmp_path, {"cmd_soup_0": _obj((0, 0, 0.7), (0.4, 1.6, 0.05), True)},
           robot=(0.6, 1.5, 0.0), persons={"bedroom": {"xyz": [0.28, 1.755, 0.0]}})
    c = _checks(tmp_path, [_target(0, ["delivered(soup,bedroom)"], obj="soup", loc="bedroom")])
    assert c["delivered(soup,bedroom)"]["verdict"] == "PASS"
    assert "person in bedroom" in c["delivered(soup,bedroom)"]["reason"]


def test_delivered_fails_when_the_object_was_dropped_on_the_way(tmp_path):
    # fix5 run 004: banana carried 0.45 m up, then fell at the kitchen table;
    # the robot drove back to the operator empty-handed.
    _truth(tmp_path, {"cmd_banana_0": _obj((2.39, -3.11, 0.72), (3.05, -1.87, 0.05), False, lift=0.45)},
           robot=(COMMAND_POINT["x"] + 0.1, COMMAND_POINT["y"] + 0.1, 0.0))
    c = _checks(tmp_path, [_target(0, ["delivered(food,operator)"], obj="food")])
    check = c["delivered(food,operator)"]
    assert check["verdict"] == "FAIL"
    assert "never reached the recipient" in check["reason"]


def test_delivered_unknown_without_robot_pose(tmp_path):
    _truth(tmp_path, {"cmd_soup_0": _obj((0, 0, 0.7), (0, 0, 0.7), True)}, robot=None)
    c = _checks(tmp_path, [_target(0, ["delivered(soup,operator)"], obj="soup")])
    assert c["delivered(soup,operator)"]["verdict"] == "UNKNOWN"


# --------------------------------------------------------------------------- counted()

def _count_run(tmp_path, log_lines: list[str] | None, announcements: list[str] | None):
    if log_lines is not None:
        (tmp_path / "orchestrator.log").write_text("\n".join(log_lines) + "\n")
    if announcements is not None:
        (tmp_path / "announcements.txt").write_text("\n".join(announcements) + "\n")


def test_counted_fails_on_mismatch_and_names_both_numbers(tmp_path):
    _spawned(tmp_path, [("cmd_bleach_0", "side_table"), ("cmd_mustard_0", "side_table"),
                        ("cmd_bowl_0", "side_table"), ("cmd_banana_0", "side_table")])
    _count_run(tmp_path, ["--> count detections [✕] -- counted 0 objects — deferring to fallback",
                          "--> parse vlm count [✓] -- parsed count=0 from VLM answer"], None)
    c = _checks(tmp_path, [_target(0, ["counted(kitchen items)"], obj="kitchen items", loc="side_table")])
    check = c["counted(kitchen items)"]
    assert check["verdict"] == "FAIL"
    assert check["announced"] == 0 and check["expected"] == 3
    assert "announced 0, 3 x kitchen items" in check["reason"]
    assert "parsed count=0" in check["reason"]


def test_counted_passes_on_match_from_log_feedback(tmp_path):
    _spawned(tmp_path, [("cmd_bleach_0", "side_table"), ("cmd_mustard_0", "side_table"),
                        ("cmd_bowl_0", "side_table")])
    _count_run(tmp_path, ["--> count detections [✓] -- counted 3 objects"], ["I counted 3"])
    c = _checks(tmp_path, [_target(0, ["counted(kitchen items)"], obj="kitchen items", loc="side_table")])
    assert c["counted(kitchen items)"]["verdict"] == "PASS"
    assert c["counted(kitchen items)"]["announced"] == 3


def test_counted_last_log_match_wins_over_the_deferred_detector_zero(tmp_path):
    _spawned(tmp_path, [("cmd_mug_0", "kitchen_table"), ("cmd_soup_0", "kitchen_table")])
    _count_run(tmp_path, ["counted 0 objects — deferring to fallback",
                          "parsed count=2 from VLM answer"], None)
    c = _checks(tmp_path, [_target(0, ["counted(drinks)"], obj="drinks", loc="kitchen_table")])
    assert c["counted(drinks)"]["verdict"] == "PASS"


def test_counted_falls_back_to_announcements_text(tmp_path):
    _spawned(tmp_path, [("cmd_person_bedroom", "bedroom")])
    _count_run(tmp_path, None, ["I am counting now", "There are **0 persons** visible in the image"])
    c = _checks(tmp_path, [_target(0, ["counted(persons_pointing_right)"],
                                   obj="persons pointing to the right", loc="bedroom")])
    assert c["counted(persons_pointing_right)"]["verdict"] == "PASS"
    assert c["counted(persons_pointing_right)"]["expected"] == 0
    assert "announced 0, 0 pointing_right person(s) spawned in bedroom" in c["counted(persons_pointing_right)"]["reason"]
    assert "announcements.txt" in c["counted(persons_pointing_right)"]["reason"]


def test_counted_prefers_the_spoken_count_over_later_per_view_lines(tmp_path):
    # fix4b run 002: the sweep keeps the max over views; a 0 view logged AFTER
    # the spoken "I counted 2" must not become the announced value.
    _spawned(tmp_path, [("cmd_person_bedroom", "bedroom")])
    _count_run(tmp_path, ["--> count view detections [✓] -- counted 2 objects",
                          "--> buffer count [✓] -- buffered: I counted 2",
                          "--> announce count [✓] -- Finished announcing I counted 2",
                          "--> count view detections [✓] -- counted 0 objects"], None)
    c = _checks(tmp_path, [_target(0, ["counted(persons_pointing_right)"],
                                   obj="persons pointing to the right", loc="bedroom")])
    assert c["counted(persons_pointing_right)"]["announced"] == 2
    assert c["counted(persons_pointing_right)"]["expected"] == 0
    assert c["counted(persons_pointing_right)"]["verdict"] == "FAIL"
    assert "I counted 2" in c["counted(persons_pointing_right)"]["reason"]


def test_counted_ignores_plan_narration_in_announcements(tmp_path):
    # fix4 run 001: the orchestrator crashed before any count was spoken;
    # the only count-shaped line was the plan narration whose "2 tasks" was
    # taken as the announced count. Narration is not a result -> UNKNOWN.
    _spawned(tmp_path, [("cmd_bleach_0", "side_table"), ("cmd_bowl_0", "side_table")])
    _count_run(tmp_path, None, ["Here is my plan. I will complete 2 tasks. First, count how many kitchen items "
                                "are on the side_table. Then, report the result to the operator.",
                                "I am counting now."])
    c = _checks(tmp_path, [_target(0, ["counted(kitchen items)"], obj="kitchen items", loc="side_table")])
    assert c["counted(kitchen items)"]["verdict"] == "UNKNOWN"


def test_counted_parses_number_words_in_announcements(tmp_path):
    _spawned(tmp_path, [("cmd_bowl_0", "side_table")])
    _count_run(tmp_path, None, ["Zero red bowl or red ceramic mug are visible in this image"])
    c = _checks(tmp_path, [_target(0, ["counted(dishes)"], obj="dishes", loc="side_table")])
    assert c["counted(dishes)"]["verdict"] == "FAIL"
    assert c["counted(dishes)"]["announced"] == 0


def test_counted_counts_already_present_skips_and_room_spots(tmp_path):
    # "already present" = the base scenario had it there: physically in the scene.
    _spawned(tmp_path, [("cmd_sugar_box_0", "side_table_02")], skipped=[("cmd_banana_0", "side_table_02")])
    _count_run(tmp_path, ["parsed count=2 from VLM answer"], None)
    c = _checks(tmp_path, [_target(0, ["counted(food)"], obj="food", loc="bedroom")])
    assert c["counted(food)"]["verdict"] == "PASS"
    assert c["counted(food)"]["expected"] == 2


def test_counted_lying_people_filters_out_a_standing_person(tmp_path):
    _spawned(tmp_path, [], skipped=[("cmd_person_standing", "kitchen")],
             poses={"cmd_person_standing": "standing"})
    _count_run(tmp_path, ["--> announce count [✓] -- Finished announcing I counted 1"], None)
    check = _checks(tmp_path, [_target(0, ["counted(lying_persons)"],
                                       obj="lying persons", loc="kitchen")])["counted(lying_persons)"]
    assert check["expected"] == 0
    assert check["verdict"] == "FAIL"
    assert "0 lying person(s) spawned in kitchen" in check["reason"]


def test_counted_lying_people_counts_only_the_matching_pose(tmp_path):
    _spawned(tmp_path, [], skipped=[("cmd_person_standing", "kitchen"),
                                    ("cmd_person_lying", "kitchen")],
             poses={"cmd_person_standing": "standing", "cmd_person_lying": "lying"})
    _count_run(tmp_path, ["--> announce count [✓] -- Finished announcing I counted 1"], None)
    lying = _checks(tmp_path, [_target(0, ["counted(lying_persons)"],
                                       obj="lying persons", loc="kitchen")])["counted(lying_persons)"]
    people = _checks(tmp_path, [_target(0, ["counted(persons)"],
                                        obj="persons", loc="kitchen")])["counted(persons)"]
    assert lying["expected"] == 1 and lying["verdict"] == "PASS"
    assert people["expected"] == 2


def test_counted_pointing_people_counts_only_the_matching_pose(tmp_path):
    _spawned(tmp_path, [], skipped=[("cmd_person_standing", "kitchen"),
                                    ("cmd_person_pointing", "kitchen")],
             poses={"cmd_person_standing": "standing", "cmd_person_pointing": "pointing_right"})
    _count_run(tmp_path, ["--> announce count [✓] -- Finished announcing I counted 1"], None)
    check = _checks(tmp_path, [_target(0, ["counted(persons_pointing_right)"],
                                       obj="persons pointing to the right", loc="kitchen")])["counted(persons_pointing_right)"]
    assert check["expected"] == 1
    assert check["verdict"] == "PASS"


def test_counted_does_not_count_a_failed_lying_person_spawn(tmp_path):
    (tmp_path / "scene-plan.json").write_text(json.dumps({"items": [
        {"id": "cmd_person_lying", "kind": "person", "room": "kitchen", "pose": "lying"}
    ]}))
    (tmp_path / "spawned.json").write_text(json.dumps({"entities": [
        {"id": "cmd_person_lying", "where": "kitchen", "ok": False}
    ], "skipped": []}))
    _count_run(tmp_path, ["--> announce count [✓] -- Finished announcing I counted 0"], None)
    check = _checks(tmp_path, [_target(0, ["counted(lying_persons)"],
                                       obj="lying persons", loc="kitchen")])["counted(lying_persons)"]
    assert check["expected"] == 0
    assert check["verdict"] == "PASS"


def test_counted_does_not_double_count_present_entity_reported_as_skipped(tmp_path):
    # The scene applier may include the same pre-existing entity in both arrays.
    _spawned(tmp_path, [("cmd_bowl_0", "side_table")],
             skipped=[("cmd_bowl_0", "side_table")])
    _count_run(tmp_path, ["parsed count=1 from VLM answer"], None)
    c = _checks(tmp_path, [_target(0, ["counted(kitchen items)"], obj="kitchen items", loc="side_table")])
    assert c["counted(kitchen items)"]["verdict"] == "PASS"
    assert c["counted(kitchen items)"]["expected"] == 1


def test_counted_unknown_without_spawned_manifest_or_announced_value(tmp_path):
    c = _checks(tmp_path, [_target(0, ["counted(food)"], obj="food", loc="shelf")])
    assert c["counted(food)"]["verdict"] == "UNKNOWN"
    _spawned(tmp_path, [("cmd_soup_0", "shelf")])
    c = _checks(tmp_path, [_target(0, ["counted(food)"], obj="food", loc="shelf")])
    assert c["counted(food)"]["verdict"] == "UNKNOWN"
    assert "announced count not found" in c["counted(food)"]["reason"]


# --------------------------------------------------------------------------- at_robot / others

def test_at_robot_checked_only_for_the_last_target(tmp_path):
    kt = json.loads(CONSTANTS.read_text())["possible_poses"]["kitchen_table"]["point"]
    _truth(tmp_path, {}, robot=(kt["x"] + 0.2, kt["y"], 0.0))
    targets = [_target(0, ["at_robot(start_position)"], loc="start_position"),
               _target(1, ["at_robot(kitchen_table)"], loc="kitchen_table")]
    c = _checks(tmp_path, targets)
    assert c["at_robot(start_position)"]["verdict"] == "PLANNER"
    assert c["at_robot(kitchen_table)"]["verdict"] == "PASS"
    _truth(tmp_path, {}, robot=(kt["x"] + 2.0, kt["y"], 0.0))
    assert _checks(tmp_path, targets)["at_robot(kitchen_table)"]["verdict"] == "FAIL"


def test_other_facts_keep_the_planner_gate_verdict(tmp_path):
    _truth(tmp_path, {})
    targets = [_target(0, ["object_seen(food)", "answered(x)"], obj="food")]
    checks = truth.physical_checks(tmp_path, targets, constants=CONSTANTS,
                                   gate_verdicts={(0, "object_seen(food)"): "VALID"})
    assert [c["verdict"] for c in checks] == ["PLANNER", "PLANNER"]
    assert checks[0]["gate"] == "VALID"


# --------------------------------------------------------------------------- combination

def test_combine_fail_wins_then_referee_then_pass_then_unknown():
    p = {"fact": "held(x)", "verdict": "PASS", "reason": "ok"}
    f = {"fact": "placed(x,y)", "verdict": "FAIL", "reason": "floor"}
    u = {"fact": "held(x)", "verdict": "UNKNOWN", "reason": truth.PHYSICAL_TRUTH_UNAVAILABLE}
    assert truth.combine_verdicts("PASS", [p, f], False)[:2] == ("FAIL", "FAIL")
    assert truth.combine_verdicts("PASS", [p, f], True)[:2] == ("FAIL", "FAIL")
    assert truth.combine_verdicts("PASS", [p], True)[:2] == ("REFEREE", "REFEREE")
    assert truth.combine_verdicts("PASS", [u], True)[:2] == ("REFEREE", "REFEREE")
    assert truth.combine_verdicts("PASS", [p], False)[:2] == ("PASS", "PASS")
    physical, final, detail = truth.combine_verdicts("PASS", [u], False)
    assert (physical, final) == ("UNKNOWN", "PASS")
    assert detail.startswith(truth.PHYSICAL_TRUTH_UNAVAILABLE)
    assert truth.combine_verdicts("PASS", [], False)[:2] == ("UNKNOWN", "PASS")
    # A non-PASS planner verdict always stands, whatever physics says.
    assert truth.combine_verdicts("FAIL", [p], False)[:2] == ("PASS", "FAIL")
    assert truth.combine_verdicts("TIMEOUT", [], True)[:2] == ("REFEREE", "TIMEOUT")


# --------------------------------------------------------------------------- tier2 wiring

@pytest.fixture(autouse=True)
def _fake_llm_preflight_ok(monkeypatch):
    monkeypatch.setattr(tier2, "llm_preflight", lambda env: (True, ""))


def _entry(text="bring me a soup from the shelf"):
    return CorpusEntry(id="c0", seed=7, template="bringMeObjFromPlcmt", followups=(), category="objects",
                       text=text, feasibility="B")


def _fake_orchestrator(tmp_path: Path, targets: list[dict], method: str | None = None) -> list[str]:
    """Fake launcher: emits split.accepted + a grasp step (optionally referee_fallback) + task.finished."""
    script = tmp_path / "fake_orch_truth.py"
    script.write_text(textwrap.dedent(f"""
        import json, os, time
        d = os.path.join(os.environ["BT_GPSR_PLAN_DIR"], "debug", "traj-1"); os.makedirs(d, exist_ok=True)
        f = open(os.path.join(d, "events.jsonl"), "a", buffering=1)
        def ev(t, payload):
            f.write(json.dumps({{"event_type": t, "task_id": "traj-1/task-1", "payload": payload, "occurred_at": "x"}}) + "\\n")
        ev("split.accepted", {{"slot": 0, "targets": {targets!r}}})
        step = {{"action": "grasp", "outcome": "succeeded", "params": {{"object": "soup"}}}}
        if {method!r}:
            step["method"] = {method!r}
        ev("step.finished", step)
        ev("task.finished", {{"status": "succeeded", "reason": "r"}})
        while True:
            time.sleep(1)
    """))
    return [sys.executable, str(script)]


def _run(tmp_path, targets, *, method=None, pre_run=None):
    out = tmp_path / "out"
    run_dir = out / "runs" / "c0"
    run_dir.mkdir(parents=True)
    if pre_run:
        pre_run(run_dir)
    results = run_tier2([_entry()], mock_config=tmp_path / "m.json", constants=CONSTANTS,
                        out_dir=out, timeout_s=20, launcher=_fake_orchestrator(tmp_path, targets, method),
                        reset_cmd=["true"], settle_s=0)
    run_json = json.loads((run_dir / "run.json").read_text())
    return results[0], run_json


HELD_TARGETS = [_target(0, ["held(soup)"], obj="soup", loc="shelf")]


def test_tier2_pass_becomes_fail_when_truth_shows_no_grasp(tmp_path):
    result, run_json = _run(tmp_path, HELD_TARGETS, pre_run=lambda rd: _truth(
        rd, {"cmd_soup_0": _obj((-3.6, 0.3, 1.07), (-3.6, 0.3, 1.07), False)}))
    assert (result.planner_verdict, result.physical_verdict, result.verdict) == ("PASS", "FAIL", "FAIL")
    assert "never retained by gripper" in result.detail
    assert run_json["verdict"] == "FAIL" and run_json["planner_verdict"] == "PASS"
    assert run_json["physical_verdict"] == "FAIL"
    assert run_json["physical_checks"][0]["fact"] == "held(soup)"
    assert run_json["physical_checks"][0]["verdict"] == "FAIL"


def test_tier2_pass_confirmed_by_truth_stays_pass(tmp_path):
    result, run_json = _run(tmp_path, HELD_TARGETS, pre_run=lambda rd: _truth(
        rd, {"cmd_soup_0": _obj((-3.6, 0.3, 1.07), (-3.6, 0.3, 1.3), True, lift=0.23)}))
    assert (result.planner_verdict, result.physical_verdict, result.verdict) == ("PASS", "PASS", "PASS")
    assert "retained by the gripper" in result.physical_detail
    assert run_json["physical_verdict"] == "PASS"


def test_tier2_referee_assisted_run_scores_referee_unless_physics_fails(tmp_path):
    result, run_json = _run(tmp_path, HELD_TARGETS, method="referee_fallback")
    assert result.referee_assisted is True
    assert (result.planner_verdict, result.physical_verdict, result.verdict) == ("PASS", "REFEREE", "REFEREE")
    assert run_json["verdict"] == "REFEREE"

    result, _ = _run(tmp_path / "b", HELD_TARGETS, method="referee_fallback", pre_run=lambda rd: _truth(
        rd, {"cmd_soup_0": _obj((0, 0, 1), (0, 0, 1), False)}))
    assert (result.physical_verdict, result.verdict) == ("FAIL", "FAIL")


def test_tier2_without_truth_file_is_planner_pass_with_truth_unavailable(tmp_path):
    result, run_json = _run(tmp_path, HELD_TARGETS)
    assert (result.planner_verdict, result.physical_verdict, result.verdict) == ("PASS", "UNKNOWN", "PASS")
    assert result.physical_detail.startswith("physical truth unavailable")
    assert result.detail == ""  # planner detail untouched (existing tier2 tests rely on it)
    assert run_json["physical_detail"].startswith("physical truth unavailable")


def test_tier2_legacy_grasp_truth_gate_still_downgrades(tmp_path):
    def pre(rd):
        (rd / "grasp_truth.json").write_text(json.dumps({"any_object_retained": False, "max_xy_disp_m": 0.002}))
    result, _ = _run(tmp_path, HELD_TARGETS, pre_run=pre)
    assert (result.physical_verdict, result.verdict) == ("FAIL", "FAIL")
    assert "never retained" in result.detail or "closed on air" in result.detail


def test_tier2_count_mismatch_fails_the_run(tmp_path):
    targets = [_target(0, ["counted(kitchen items)"], obj="kitchen items", loc="side_table")]

    def pre(rd):
        _spawned(rd, [("cmd_bleach_0", "side_table"), ("cmd_mustard_0", "side_table"),
                      ("cmd_bowl_0", "side_table")])
        (rd / "orchestrator.log").write_text("--> parse vlm count [✓] -- parsed count=0 from VLM answer\n")
    result, run_json = _run(tmp_path, targets, pre_run=pre)
    assert result.verdict == "FAIL"
    assert "announced 0, 3 x kitchen items" in result.detail
    assert run_json["physical_checks"][0]["expected"] == 3


def test_tier2_physical_verdict_helper_never_raises(tmp_path, monkeypatch):
    def boom(*a, **k):
        raise RuntimeError("probe exploded")
    monkeypatch.setattr(truth, "read_split_targets", boom)
    scored = tier2._physical_verdict(tmp_path, "PASS", "", False, CONSTANTS)
    assert scored["verdict"] == "PASS" and scored["physical_verdict"] == "UNKNOWN"
    assert "probe exploded" in scored["physical_detail"]


# --------------------------------------------------------------------------- report / old runs

def test_report_renders_planner_and_physical_columns_and_referee_totals(tmp_path):
    results = [
        BenchResult("a", "goToLoc", "A", 2, "PASS", planner_verdict="PASS", physical_verdict="PASS",
                    physical_detail="held(x): ok"),
        BenchResult("b", "goToLoc", "A", 2, "REFEREE", referee_assisted=True, planner_verdict="PASS",
                    physical_verdict="REFEREE", physical_detail="referee-assisted run"),
        BenchResult("c", "goToLoc", "A", 2, "FAIL", "held(x): never retained", planner_verdict="PASS",
                    physical_verdict="FAIL", physical_detail="held(x): never retained"),
    ]
    text = write_report(results, tmp_path).read_text()
    assert "| run | verdict | planner | physical | physical detail |" in text
    assert "| b | **REFEREE** | PASS | REFEREE | referee-assisted run |" in text
    assert "- T2: PASS 1, FAIL 1, REFEREE 1, TIMEOUT 0, ERROR 0" in text
    assert "- T2 physical: PASS 1, FAIL 1, REFEREE 1, UNKNOWN 0" in text
    assert "- b **REFEREE** _(referee-assisted)_ (planner PASS / physical REFEREE)" in text
    data = json.loads((tmp_path / "report.json").read_text())
    assert data["totals"]["2"] == {"PASS": 1, "REFEREE": 1, "FAIL": 1}
    assert data["results"][2]["physical_checks"] == []


def test_old_report_json_without_physical_fields_still_loads_and_renders(tmp_path):
    old = {"entry_id": "x", "template": "goToLoc", "feasibility": "A", "tier": 2, "verdict": "PASS",
           "detail": "", "seconds": 1.0, "plan": ["goto"], "referee_assisted": False}
    r = BenchResult(**old)
    assert r.planner_verdict is None and r.physical_verdict is None and r.physical_checks == []
    text = write_report([r], tmp_path).read_text()
    assert "## Physical truth" not in text
    assert "- x **PASS**\n" in text


@pytest.mark.skipif(not EXAMPLE_RUNS.is_dir(), reason="example bench runs not checked out")
def test_existing_example_run_dirs_score_without_error():
    roots = [p for p in (EXAMPLE_RUNS / "t2-rand-70718-fix2" / "runs",
                         EXAMPLE_RUNS / "t2-rand-70718-fix3" / "runs") if p.is_dir()]
    if not roots:
        pytest.skip("example run dirs absent")
    seen = 0
    for root in roots:
        for rd in sorted(root.iterdir()):
            run = json.loads((rd / "run.json").read_text())
            scored = tier2._physical_verdict(rd, run["verdict"], run["detail"], run["referee_assisted"], CONSTANTS)
            assert scored["physical_verdict"] in ("PASS", "FAIL", "REFEREE", "UNKNOWN")
            assert scored["verdict"] in ("PASS", "FAIL", "REFEREE", "TIMEOUT", "ERROR")
            if run["verdict"] != "PASS":
                assert scored["verdict"] == run["verdict"]
            for c in scored["physical_checks"]:
                assert {"fact", "verdict", "reason", "target"} <= set(c)
            seen += 1
    assert seen > 0


# --------------------------------------------------------------------------- partial spawns

def _manifest(tmp_path, entities, not_attempted=(), skipped=()):
    p = tmp_path / "spawned.json"
    p.write_text(json.dumps({"entities": list(entities), "not_attempted": list(not_attempted),
                             "skipped": list(skipped)}))
    return p


def test_partial_spawn_with_a_room_refusal_is_usable(tmp_path):
    # fix6 run 003: sugar box spawned, soup can refused for lack of room -> run it.
    p = _manifest(tmp_path, [{"id": "cmd_sugar_box_0", "ok": True}],
                  [{"id": "cmd_soup_0", "ok": False,
                    "error": "not attempted: no room on 'side_table_02' usable 0.34 x 0.34 m"}])
    assert tier2._partial_spawn_is_usable(p) is True


def test_spawn_with_a_sim_error_or_nothing_placed_is_not_usable(tmp_path):
    assert tier2._partial_spawn_is_usable(_manifest(tmp_path, [{"id": "a", "ok": False}])) is False
    p = _manifest(tmp_path, [], [{"id": "b", "error": "not attempted: no room on 'x'"}])
    assert tier2._partial_spawn_is_usable(p) is False
    p = _manifest(tmp_path, [{"id": "a", "ok": True}], [{"id": "b", "error": "service unavailable"}])
    assert tier2._partial_spawn_is_usable(p) is False
    assert tier2._partial_spawn_is_usable(tmp_path / "missing.json") is False


def test_delivered_fails_when_the_object_is_still_in_the_robots_hand(tmp_path):
    # fix8 run s70718-000: approach to the person aborted, soup still in the
    # gripper at z=1.16, 0.28 m from the robot, near the operator start pose.
    _truth(tmp_path, {"cmd_soup_0": _obj((-3.687, 0.309, 1.06), (-1.66, -1.66, 1.16), False, lift=0.1)},
           robot=(-1.93, -1.74, 0.0))
    c = _checks(tmp_path, [_target(0, ["delivered(soup,operator)"], obj="soup")])
    check = c["delivered(soup,operator)"]
    assert check["verdict"] == "FAIL"
    assert "never handed over" in check["reason"]


def test_delivered_to_operator_uses_the_spawned_operator_body(tmp_path):
    op = {"xyz": [-1.0, -2.0, 0.0]}
    _truth(tmp_path, {"cmd_soup_0": _obj((-3.6, 0.3, 1.07), (-1.3, -2.0, 0.05), True, lift=0.2)},
           robot=(-1.6, -2.0, 0.0), persons={"operator": op})
    c = _checks(tmp_path, [_target(0, ["delivered(soup,me)"], obj="soup")])
    check = c["delivered(soup,me)"]
    assert check["verdict"] == "PASS"
    assert "from operator" in check["reason"]
