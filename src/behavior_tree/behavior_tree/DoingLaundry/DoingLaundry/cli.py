from behavior_tree.runtime import run_tree


def main():
    from .laundry import createDoingLaundry

    run_tree(createDoingLaundry, period_ms=500.0, title="Doing Laundry")
