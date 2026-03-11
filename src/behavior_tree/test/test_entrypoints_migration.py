import ast
import importlib
import sys
import types
from pathlib import Path

import pytest


SETUP_PY = Path(__file__).resolve().parents[1] / "setup.py"
PKG_ROOT = Path(__file__).resolve().parents[1]


def _console_scripts_from_setup():
    tree = ast.parse(SETUP_PY.read_text(encoding="utf-8"))
    scripts = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "setup":
            continue
        for kw in node.keywords:
            if kw.arg != "entry_points" or not isinstance(kw.value, ast.Dict):
                continue
            for key, value in zip(kw.value.keys, kw.value.values):
                is_console_scripts = (
                    isinstance(key, ast.Constant)
                    and key.value == "console_scripts"
                )
                if not is_console_scripts:
                    continue
                if not isinstance(value, (ast.List, ast.Tuple)):
                    continue
                for item in value.elts:
                    is_string_literal = (
                        isinstance(item, ast.Constant)
                        and isinstance(item.value, str)
                    )
                    if is_string_literal:
                        scripts.append(item.value)
    return scripts


def _parse_entrypoint(entry):
    name, target = [part.strip() for part in entry.split("=", 1)]
    module, func = [part.strip() for part in target.split(":", 1)]
    return name, module, func


def _module_source_path(module_name):
    rel = Path(*module_name.split("."))
    return PKG_ROOT / f"{rel}.py"


def _function_defined_in_file(file_path, func_name):
    tree = ast.parse(file_path.read_text(encoding="utf-8"))
    return any(
        isinstance(node, ast.FunctionDef) and node.name == func_name
        for node in tree.body
    )


def _install_runtime_stub(monkeypatch):
    runtime = types.ModuleType("behavior_tree.runtime")
    calls = []

    def run_tree(factory, **kwargs):
        calls.append(("run_tree", factory, kwargs))

    def draw_tree(factory, **kwargs):
        calls.append(("draw_tree", factory, kwargs))

    runtime.run_tree = run_tree
    runtime.draw_tree = draw_tree
    monkeypatch.setitem(sys.modules, "behavior_tree.runtime", runtime)
    return calls


def _install_factory_stub(monkeypatch, module_name, factory_name):
    module = types.ModuleType(module_name)

    def factory():
        return "root"

    factory.__name__ = factory_name
    setattr(module, factory_name, factory)
    monkeypatch.setitem(sys.modules, module_name, module)
    return factory


def _install_parent_package_stubs(monkeypatch, dotted_name):
    parts = dotted_name.split(".")
    for idx in range(1, len(parts)):
        pkg_name = ".".join(parts[:idx])
        if pkg_name == "behavior_tree":
            continue
        pkg_module = types.ModuleType(pkg_name)
        pkg_path = PKG_ROOT / Path(*pkg_name.split("."))
        pkg_module.__path__ = [str(pkg_path)]
        monkeypatch.setitem(sys.modules, pkg_name, pkg_module)


MIGRATED_EXPECTED = {
    "draw": "behavior_tree.cli_draw:main",
    "follow": "behavior_tree.HelpMeCarry.cli:follow",
    "follow-audio": "behavior_tree.HelpMeCarry.cli:follow_audio",
    "follow-action": "behavior_tree.HelpMeCarry.cli:follow_action",
    "test-track": "behavior_tree.HelpMeCarry.cli:follow",
    "receptionist": "behavior_tree.Receptionist.cli:main",
    "grasp-intel": "behavior_tree.grasp_intel_demo.cli:grasp_intel",
    "grasp-audio": "behavior_tree.grasp_intel_demo.cli:grasp_audio",
    "serve-breakfast": "behavior_tree.ServeBreakfast.cli:main",
    "store-groceries": "behavior_tree.StoringGroceries.cli:main",
    "store-groceries-placing-only": (
        "behavior_tree.StoringGroceries.cli:placing_only"
    ),
    "help-me-carry": "behavior_tree.HelpMeCarry.cli:help_me_carry",
    "test-prompt-reached": "behavior_tree.HelpMeCarry.cli:test_prompt_reached",
    "inspection": "behavior_tree.Inspection.cli:main",
    "hri": "behavior_tree.HRI.cli:main",
    "restaurant": "behavior_tree.Restaurant.cli:main",
    "restaurant-simplified": "behavior_tree.Restaurant.cli:simplified",
}


CLI_DISPATCH_CASES = [
    (
        "behavior_tree.Receptionist.cli",
        "main",
        "behavior_tree.Receptionist.receptionist",
        "createReceptionist",
        "run_tree",
        {"period_ms": 250.0, "title": "Receptionist"},
    ),
    (
        "behavior_tree.HelpMeCarry.cli",
        "follow",
        "behavior_tree.HelpMeCarry.Track",
        "createFollowPerson",
        "run_tree",
        {"period_ms": 500.0, "title": "Follow"},
    ),
    (
        "behavior_tree.HelpMeCarry.cli",
        "follow_audio",
        "behavior_tree.HelpMeCarry.Track",
        "createFollowPerson",
        "run_tree",
        {"period_ms": 500.0, "title": "Follow"},
    ),
    (
        "behavior_tree.HelpMeCarry.cli",
        "follow_action",
        "behavior_tree.HelpMeCarry.Track",
        "createFollowPerson",
        "run_tree",
        {"period_ms": 500.0, "title": "Follow"},
    ),
    (
        "behavior_tree.HelpMeCarry.cli",
        "help_me_carry",
        "behavior_tree.HelpMeCarry.help_me_carry",
        "createHelpMeCarry",
        "run_tree",
        {"period_ms": 200.0, "title": "Help Me Carry"},
    ),
    (
        "behavior_tree.HelpMeCarry.cli",
        "test_prompt_reached",
        "behavior_tree.HelpMeCarry.prompt_reached",
        "testPromptReached",
        "run_tree",
        {"period_ms": 500.0, "title": "Prompt Reached"},
    ),
    (
        "behavior_tree.ServeBreakfast.cli",
        "main",
        "behavior_tree.ServeBreakfast.serve_breakfast",
        "createServeBreakfast",
        "run_tree",
        {"period_ms": 500.0, "title": "Serve Breakfast"},
    ),
    (
        "behavior_tree.StoringGroceries.cli",
        "main",
        "behavior_tree.StoringGroceries.storing_groceries",
        "createStoreGroceries",
        "run_tree",
        {"period_ms": 500.0, "title": "Store Groceries"},
    ),
    (
        "behavior_tree.StoringGroceries.cli",
        "placing_only",
        "behavior_tree.StoringGroceries.storing_groceries_place_only",
        "createStoreGroceriesPlaceOnly",
        "run_tree",
        {"period_ms": 500.0, "title": "Store Groceries Placing Only"},
    ),
    (
        "behavior_tree.Inspection.cli",
        "main",
        "behavior_tree.Inspection.inspection",
        "createInspection",
        "run_tree",
        {"period_ms": 500.0, "title": "Inspection"},
    ),
    (
        "behavior_tree.Restaurant.cli",
        "main",
        "behavior_tree.Restaurant.restaurants",
        "createRestaurantTask",
        "run_tree",
        {"period_ms": 500.0, "title": "Restaurant"},
    ),
    (
        "behavior_tree.Restaurant.cli",
        "simplified",
        "behavior_tree.Restaurant.restaurant_simplified",
        "createRestaurantSimplifiedTask",
        "run_tree",
        {"period_ms": 500.0, "title": "Restaurant Simplified"},
    ),
    (
        "behavior_tree.HRI.cli",
        "main",
        "behavior_tree.HRI.hri",
        "createHRITask",
        "run_tree",
        {"period_ms": 500.0, "title": "HRI"},
    ),
    (
        "behavior_tree.grasp_intel_demo.cli",
        "grasp_intel",
        "behavior_tree.grasp_intel_demo.grasp_intel",
        "create_demo",
        "run_tree",
        {"period_ms": 500.0, "title": "Grasp Intel"},
    ),
    (
        "behavior_tree.grasp_intel_demo.cli",
        "grasp_audio",
        "behavior_tree.grasp_intel_demo.grasp_audio",
        "createGraspAudio",
        "run_tree",
        {"period_ms": 500.0, "title": "Grasp Audio"},
    ),
    (
        "behavior_tree.cli_draw",
        "main",
        "behavior_tree.Receptionist.receptionist",
        "createReceptionist",
        "draw_tree",
        {},
    ),
]


def test_migrated_entrypoints_are_repointed():
    parsed = {
        name: f"{module}:{func}"
        for name, module, func in (
            _parse_entrypoint(entry) for entry in _console_scripts_from_setup()
        )
    }
    for name, expected in MIGRATED_EXPECTED.items():
        assert parsed.get(name) == expected


def test_no_console_script_points_to_behavior_tree_main():
    parsed = [
        _parse_entrypoint(entry) for entry in _console_scripts_from_setup()
    ]
    assert all(module != "behavior_tree.main" for _, module, _ in parsed)


@pytest.mark.parametrize("entry", _console_scripts_from_setup())
def test_entrypoint_target_function_exists_in_source(entry):
    _name, module, func = _parse_entrypoint(entry)
    source_path = _module_source_path(module)
    assert source_path.exists(), f"Missing module file: {source_path}"
    assert _function_defined_in_file(
        source_path, func
    ), f"Missing function {func} in {source_path}"


@pytest.mark.parametrize(
    (
        "cli_module,cli_func,factory_module,"
        "factory_name,runner_name,expected_kwargs"
    ),
    CLI_DISPATCH_CASES,
)
def test_cli_dispatches_to_runtime(
    monkeypatch,
    cli_module,
    cli_func,
    factory_module,
    factory_name,
    runner_name,
    expected_kwargs,
):
    calls = _install_runtime_stub(monkeypatch)
    _install_parent_package_stubs(monkeypatch, cli_module)
    _install_parent_package_stubs(monkeypatch, factory_module)
    expected_factory = _install_factory_stub(
        monkeypatch, factory_module, factory_name
    )

    sys.modules.pop(cli_module, None)
    module = importlib.import_module(cli_module)
    getattr(module, cli_func)()

    assert len(calls) == 1
    actual_runner, actual_factory, kwargs = calls[0]
    assert actual_runner == runner_name
    assert actual_factory is expected_factory
    assert kwargs == expected_kwargs
