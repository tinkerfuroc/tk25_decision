def main():
    """Run the Help Me Carry mission."""
    from behavior_tree.core.runtime import run_tree
    from .help_me_carry import createHelpMeCarry

    run_tree(createHelpMeCarry, period_ms=200.0, title="Help Me Carry")
