from behavior_tree.runtime import run_tree


def main():
    from .hri import createHRITask

    run_tree(createHRITask, period_ms=500.0, title="HRI")
