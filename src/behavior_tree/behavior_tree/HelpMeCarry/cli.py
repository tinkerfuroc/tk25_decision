from behavior_tree.runtime import run_tree


def follow():
    from .Track import createFollowPerson

    run_tree(createFollowPerson, period_ms=500.0, title="Follow")


def follow_audio():
    # Legacy command kept for backward compatibility.
    follow()


def follow_action():
    # Legacy command kept for backward compatibility.
    follow()


def help_me_carry():
    from .help_me_carry import createHelpMeCarry

    run_tree(createHelpMeCarry, period_ms=200.0, title="Help Me Carry")


def test_prompt_reached():
    from .prompt_reached import testPromptReached

    run_tree(testPromptReached, period_ms=500.0, title="Prompt Reached")
