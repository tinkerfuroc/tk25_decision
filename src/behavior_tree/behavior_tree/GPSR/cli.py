"""Canonical GPSR command."""


def main():
    from .gpsr_orchestrator import main as run_orchestrator

    return run_orchestrator()

