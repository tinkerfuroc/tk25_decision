from behavior_tree.runtime import run_tree


def grasp_intel():
    from .grasp_intel import create_demo

    run_tree(create_demo, period_ms=500.0, title="Grasp Intel")


def grasp_audio():
    from .grasp_audio import createGraspAudio

    run_tree(createGraspAudio, period_ms=500.0, title="Grasp Audio")
