from behavior_tree.runtime import run_tree


def main():
    from .pick_and_place import pickAndPlaceShortened

    run_tree(pickAndPlaceShortened, period_ms=500.0, title="Pick And Place")
