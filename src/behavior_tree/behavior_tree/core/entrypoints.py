"""Canonical installed command registry."""

TASK_ENTRYPOINTS = {
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
}

TOOL_ENTRYPOINTS = {
    "draw": "behavior_tree.tools.draw:main",
    "fetch-points": "behavior_tree.tools.fetch_points:main",
    "verify-task-endpoints": "behavior_tree.tools.verify_task_endpoints:main",
}

ALL_ENTRYPOINTS = {**TASK_ENTRYPOINTS, **TOOL_ENTRYPOINTS}

