def createPickAndPlaceTask():
    from .pick_and_place import createPickAndPlaceTask as _create_pick_and_place_task

    return _create_pick_and_place_task()


__all__ = ["createPickAndPlaceTask"]
