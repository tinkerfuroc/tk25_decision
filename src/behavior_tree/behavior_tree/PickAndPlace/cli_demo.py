from behavior_tree.runtime import run_tree


def main():
    """Run the legacy narrow demo tree (pickAndPlaceShortened), preserved as-is."""
    from .pick_and_place import pickAndPlaceShortened

    run_tree(pickAndPlaceShortened, period_ms=500.0, title="Pick And Place (demo)")
