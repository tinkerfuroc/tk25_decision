from behavior_tree.runtime import run_tree


def main():
    from .inspection import createInspection

    run_tree(createInspection, period_ms=500.0, title="Inspection")
