from behavior_tree.runtime import run_tree


def main():
    from .phases.top_level import createPickAndPlaceTask

    run_tree(createPickAndPlaceTask, period_ms=500.0, title="Pick And Place")