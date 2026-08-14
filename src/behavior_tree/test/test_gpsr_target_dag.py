"""Tests for validation of the canonical ordered GPSR target DAG."""
from __future__ import annotations

from types import MappingProxyType

import pytest

from behavior_tree.GPSR.planner_validators import validate_dag


def assert_invalid(targets, *reason_substrings):
    ok, reason = validate_dag(targets)
    assert not ok
    assert reason is not None
    for substring in reason_substrings:
        assert substring in reason


def test_validate_dag_accepts_empty_list():
    assert validate_dag([]) == (True, None)


def test_validate_dag_accepts_single_target():
    assert validate_dag([{"id": "grasp", "depends_on": []}]) == (True, None)


def test_validate_dag_accepts_multiple_dependencies_in_order():
    targets = [
        {"id": "grasp", "depends_on": []},
        {"id": "navigate", "depends_on": ["grasp"]},
        {"id": "place", "depends_on": ["grasp", "navigate"]},
    ]
    assert validate_dag(targets) == (True, None)


def test_validate_dag_accepts_mapping_targets():
    target = MappingProxyType({"id": "grasp", "depends_on": []})
    assert validate_dag([target]) == (True, None)


def test_validate_dag_rejects_non_list_targets():
    assert_invalid({"id": "grasp", "depends_on": []}, "targets", "list")


def test_validate_dag_rejects_non_mapping_target():
    assert_invalid(["grasp"], "target 0", "mapping")


@pytest.mark.parametrize("target", [
    {"id": "", "depends_on": []},
    {"id": "   ", "depends_on": []},
    {"id": 123, "depends_on": []},
    {"depends_on": []},
])
def test_validate_dag_rejects_empty_or_non_string_id(target):
    assert_invalid([target], "target 0", "id")


def test_validate_dag_rejects_duplicate_ids():
    assert_invalid([
        {"id": "grasp", "depends_on": []},
        {"id": "grasp", "depends_on": []},
    ], "target 1", "duplicate", "grasp")


def test_validate_dag_rejects_absent_depends_on():
    assert_invalid([{"id": "grasp"}], "target 0", "depends_on", "required")


def test_validate_dag_rejects_non_list_depends_on():
    assert_invalid([{"id": "grasp", "depends_on": None}], "target 0", "depends_on", "list")


@pytest.mark.parametrize("dependency", [123, "", "   ", None])
def test_validate_dag_rejects_non_string_or_empty_dependency(dependency):
    assert_invalid([
        {"id": "grasp", "depends_on": [dependency]},
    ], "target 0", "dependency", "depends_on")


def test_validate_dag_rejects_duplicate_dependency_ids():
    assert_invalid([
        {"id": "grasp", "depends_on": []},
        {"id": "place", "depends_on": ["grasp", "grasp"]},
    ], "target 1", "duplicate", "grasp")


def test_validate_dag_rejects_unknown_dependency_ids():
    assert_invalid([
        {"id": "place", "depends_on": ["missing"]},
    ], "target 0", "unknown", "missing")


def test_validate_dag_rejects_self_dependency():
    assert_invalid([
        {"id": "grasp", "depends_on": ["grasp"]},
    ], "target 0", "self", "grasp")


def test_validate_dag_rejects_forward_dependency():
    assert_invalid([
        {"id": "place", "depends_on": ["grasp"]},
        {"id": "grasp", "depends_on": []},
    ], "target 0", "earlier", "grasp")


def test_validate_dag_reports_first_error_deterministically():
    targets = [
        {"id": "first", "depends_on": ["missing-first"]},
        {"id": "second", "depends_on": ["missing-second"]},
    ]
    ok, reason = validate_dag(targets)
    assert not ok
    assert reason == "target 0 'first': unknown dependency id 'missing-first'"
