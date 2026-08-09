"""Pick-and-place public API without import-time ROS message side effects."""


def pickAndPlaceShortened(*args, **kwargs):
    """Build the legacy tree, importing its heavy ROS dependencies on demand."""
    from .pick_and_place import pickAndPlaceShortened as build_tree

    return build_tree(*args, **kwargs)

__all__ = ["pickAndPlaceShortened"]
