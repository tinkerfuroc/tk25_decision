def createDoingLaundryTask():
    from .laundry import createDoingLaundryTask as _create

    return _create()


__all__ = ["createDoingLaundryTask"]
