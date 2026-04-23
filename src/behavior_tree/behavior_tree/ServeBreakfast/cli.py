from behavior_tree.runtime import run_tree


def main():
    from .serve_breakfast import createServeBreakfast

    run_tree(createServeBreakfast, period_ms=500.0, title="Serve Breakfast")
