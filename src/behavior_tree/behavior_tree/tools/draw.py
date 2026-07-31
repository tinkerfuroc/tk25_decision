"""Render any canonical task tree to Graphviz files."""

import argparse

from behavior_tree.core.entrypoints import TASK_ENTRYPOINTS
from behavior_tree.core.runtime import draw_tree


def _factory(task):
    if task == "doing-laundry":
        from behavior_tree.DoingLaundry.laundry import createDoingLaundry

        return createDoingLaundry
    if task == "follow-person":
        from behavior_tree.components.following.follow_person import (
            create_follow_person_tree,
        )

        return create_follow_person_tree
    if task == "gpsr":
        from behavior_tree.GPSR.gpsr_orchestrator import createGPSROrchestrator

        return createGPSROrchestrator
    if task == "help-me-carry":
        from behavior_tree.HelpMeCarry.help_me_carry import createHelpMeCarry

        return createHelpMeCarry
    if task == "hri":
        from behavior_tree.HRI.hri_2026 import createHRITask2026

        return createHRITask2026
    if task == "inspection":
        from behavior_tree.Inspection.inspection import createInspection

        return createInspection
    if task == "pick-and-place":
        from behavior_tree.PickAndPlace.pick_and_place_rulebook import (
            pickAndPlaceRulebook,
        )

        return pickAndPlaceRulebook
    if task == "receptionist":
        from behavior_tree.Receptionist.receptionist import createReceptionist

        return createReceptionist
    if task == "restaurant":
        from behavior_tree.Restaurant.restaurant_v2 import (
            createRestaurantTask2026,
        )

        return createRestaurantTask2026
    if task == "serve-breakfast":
        from behavior_tree.ServeBreakfast.serve_breakfast import (
            createServeBreakfast,
        )

        return createServeBreakfast
    if task == "store-groceries":
        from behavior_tree.StoringGroceries.storing_groceries import (
            createStoreGroceries,
        )

        return createStoreGroceries
    raise ValueError(f"Unknown canonical task: {task}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("task", choices=sorted(TASK_ENTRYPOINTS))
    args = parser.parse_args(argv)
    draw_tree(_factory(args.task))
