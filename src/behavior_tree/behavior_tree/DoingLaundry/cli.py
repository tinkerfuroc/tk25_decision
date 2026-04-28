from behavior_tree.runtime import run_tree


def main():
    from .laundry import createDoingLaundryTask

    run_tree(createDoingLaundryTask, period_ms=500.0, title="Doing Laundry")
