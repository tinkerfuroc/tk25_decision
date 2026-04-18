from behavior_tree.runtime import run_tree


def main():
    from .restaurants import createRestaurantTask

    run_tree(createRestaurantTask, period_ms=500.0, title="Restaurant")


def simplified():
    from .restaurant_simplified import createRestaurantSimplifiedTask

    run_tree(
        createRestaurantSimplifiedTask,
        period_ms=500.0,
        title="Restaurant Simplified",
    )
