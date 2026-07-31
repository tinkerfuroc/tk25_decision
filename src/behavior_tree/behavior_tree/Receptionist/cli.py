from behavior_tree.core.runtime import run_tree


def main():
    from .receptionist import createReceptionist

    run_tree(createReceptionist, period_ms=250.0, title="Receptionist")
