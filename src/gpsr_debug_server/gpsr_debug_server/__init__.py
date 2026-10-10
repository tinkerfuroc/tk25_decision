"""Standalone developer GPSR mission debugger.

The package deliberately has no import-time dependency on ROS or FastAPI. The
storage/projection modules are useful in offline replay tests, while ``main``
loads the optional web and ROS adapters when the server is started.
"""

__all__ = ["__version__"]
__version__ = "0.1.0"
