"""Package-resource helpers that work from source and installed wheels."""

from importlib.resources import files
import json
from typing import Any


def read_json(package: str, filename: str = "constants.json") -> Any:
    """Read a JSON resource owned by ``package``."""
    resource = files(package).joinpath(filename)
    with resource.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def resource(package: str, filename: str):
    """Return an importlib ``Traversable`` for a packaged resource."""
    return files(package).joinpath(filename)

