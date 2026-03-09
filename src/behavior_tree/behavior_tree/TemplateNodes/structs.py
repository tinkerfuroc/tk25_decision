# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Data Structures Module
======================

This module defines data structures used across behavior tree nodes
for storing and passing information between nodes via the blackboard.

Classes
-------
Person
    Data container for person-related information in tasks like
    Receptionist, where the robot needs to track names, drinks,
    visual features, and interests of people it meets.
"""


class Person(object):
    """
    Data container for person information in social robot tasks.

    This class stores all relevant information about a person that the
    robot encounters during tasks like Receptionist. It is typically
    populated by vision and audio nodes and stored in the blackboard
    for use by other nodes.

    Attributes
    ----------
    name : str or None
        The person's name, extracted via speech recognition.
    fav_drink : str or None
        The person's favorite drink preference.
    features : list or None
        Visual feature vector for person recognition/identification.
    interests : str or None
        The person's stated interests for conversation matching.

    Example
    -------
    >>> person = Person()
    >>> person.name = "Alice"
    >>> person.fav_drink = "coffee"
    >>> person.interests = "robotics"
    """

    def __init__(self) -> None:
        """Initialize a Person with all attributes set to None."""
        self.name = None
        self.fav_drink = None
        self.features = None
        self.interests = None