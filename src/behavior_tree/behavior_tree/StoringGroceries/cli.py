from behavior_tree.core.runtime import run_tree


def main():
    from .storing_groceries import createStoreGroceries

    run_tree(createStoreGroceries, period_ms=500.0, title="Store Groceries")
