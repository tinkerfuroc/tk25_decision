from behavior_tree.runtime import run_tree


def main():
    from .storing_groceries import createStoreGroceries

    run_tree(createStoreGroceries, period_ms=500.0, title="Store Groceries")


def placing_only():
    from .storing_groceries_place_only import createStoreGroceriesPlaceOnly

    run_tree(
        createStoreGroceriesPlaceOnly,
        period_ms=500.0,
        title="Store Groceries Placing Only",
    )
