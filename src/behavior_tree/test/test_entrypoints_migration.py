"""Contract tests for the intentionally small installed command surface."""

import ast
from pathlib import Path


SETUP_PY = Path(__file__).resolve().parents[1] / "setup.py"
PKG_ROOT = Path(__file__).resolve().parents[1]

EXPECTED_CONSOLE_SCRIPTS = {
    "doing-laundry": "behavior_tree.DoingLaundry.cli:main",
    "follow-person": "behavior_tree.FollowPerson.cli:main",
    "gpsr": "behavior_tree.GPSR.cli:main",
    "help-me-carry": "behavior_tree.HelpMeCarry.cli:main",
    "hri": "behavior_tree.HRI.cli:main",
    "inspection": "behavior_tree.Inspection.cli:main",
    "pick-and-place": "behavior_tree.PickAndPlace.cli:main",
    "receptionist": "behavior_tree.Receptionist.cli:main",
    "restaurant": "behavior_tree.Restaurant.cli:main",
    "serve-breakfast": "behavior_tree.ServeBreakfast.cli:main",
    "store-groceries": "behavior_tree.StoringGroceries.cli:main",
    "draw": "behavior_tree.tools.draw:main",
    "fetch-points": "behavior_tree.tools.fetch_points:main",
    "verify-task-endpoints": "behavior_tree.tools.verify_task_endpoints:main",
}


def _console_scripts_from_setup():
    """Read the literal ``CONSOLE_SCRIPTS`` assignment without executing setup."""
    tree = ast.parse(SETUP_PY.read_text(encoding="utf-8"))
    for node in tree.body:
        if not isinstance(node, (ast.Assign, ast.AnnAssign)):
            continue
        value = node.value
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        if not any(
            isinstance(target, ast.Name) and target.id == "CONSOLE_SCRIPTS"
            for target in targets
        ):
            continue
        scripts = ast.literal_eval(value)
        assert isinstance(scripts, list)
        assert all(isinstance(script, str) for script in scripts)
        return scripts
    raise AssertionError("setup.py must define a literal CONSOLE_SCRIPTS list")


def _parse_entrypoint(entry):
    name, target = (part.strip() for part in entry.split("=", 1))
    module, function = (part.strip() for part in target.split(":", 1))
    return name, module, function


def _module_source_path(module_name):
    return PKG_ROOT / f"{Path(*module_name.split('.'))}.py"


def _top_level_function_names(path):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    return {
        node.name
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }


def test_console_script_surface_is_exactly_the_canonical_fourteen():
    parsed = {
        name: f"{module}:{function}"
        for name, module, function in map(
            _parse_entrypoint, _console_scripts_from_setup()
        )
    }

    assert parsed == EXPECTED_CONSOLE_SCRIPTS
    assert len(parsed) == 14


def test_every_canonical_dispatch_target_exists():
    for command, target in EXPECTED_CONSOLE_SCRIPTS.items():
        module, function = target.split(":", 1)
        source_path = _module_source_path(module)
        assert source_path.exists(), f"{command}: missing {source_path}"
        assert function in _top_level_function_names(source_path), (
            f"{command}: missing {function} in {source_path}"
        )


def test_no_versioned_or_development_commands_leak_into_operator_surface():
    command_names = {
        _parse_entrypoint(entry)[0] for entry in _console_scripts_from_setup()
    }
    forbidden_fragments = (
        "-2025",
        "-2026",
        "-new",
        "-old",
        "-test",
        "-demo",
        "-dev",
        "-dryrun",
        "-orchestrator",
        "-simplified",
        "-placing-only",
    )
    assert not {
        name
        for name in command_names
        if any(fragment in name for fragment in forbidden_fragments)
    }
