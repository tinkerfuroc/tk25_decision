def createHRITask():
    from .hri import createHRITask as _create_hri_task

    return _create_hri_task()


__all__ = ["createHRITask"]
