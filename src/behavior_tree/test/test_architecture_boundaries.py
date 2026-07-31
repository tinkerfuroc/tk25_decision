"""Static dependency contracts for the refactored package."""

import ast
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1] / "behavior_tree"
TASK_PACKAGES = {
    "DoingLaundry",
    "FollowPerson",
    "GPSR",
    "HelpMeCarry",
    "HRI",
    "Inspection",
    "PickAndPlace",
    "Receptionist",
    "Restaurant",
    "ServeBreakfast",
    "StoringGroceries",
}
SHARED_PACKAGES = {"core", "interfaces", "nodes", "components"}


def _absolute_imports(path):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            yield from (alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            yield node.module


def _behavior_tree_child(module):
    prefix = "behavior_tree."
    if not module.startswith(prefix):
        return None
    return module[len(prefix):].split(".", 1)[0]


def test_task_packages_do_not_import_other_task_packages():
    violations = []
    for task in sorted(TASK_PACKAGES):
        for path in (PACKAGE / task).rglob("*.py"):
            for module in _absolute_imports(path):
                child = _behavior_tree_child(module)
                if child in TASK_PACKAGES and child != task:
                    violations.append((path.relative_to(PACKAGE), module))
    assert violations == []


def test_shared_layers_do_not_reach_into_tasks():
    violations = []
    for shared in sorted(SHARED_PACKAGES):
        for path in (PACKAGE / shared).rglob("*.py"):
            for module in _absolute_imports(path):
                if _behavior_tree_child(module) in TASK_PACKAGES:
                    violations.append((path.relative_to(PACKAGE), module))
    assert violations == []


def test_legacy_shared_module_paths_are_gone():
    for legacy in (
        "Constants.py",
        "TemplateNodes",
        "config.py",
        "messages.py",
        "mock_messages.py",
        "runtime.py",
        "visualization.py",
    ):
        assert not (PACKAGE / legacy).exists()

