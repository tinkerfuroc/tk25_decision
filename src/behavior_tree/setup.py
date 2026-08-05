from glob import glob

from setuptools import find_packages, setup


PACKAGE_NAME = "behavior_tree"

# Public operator surface. Development probes belong in pytest, not in the
# installed ROS command namespace.
CONSOLE_SCRIPTS = [
    "doing-laundry = behavior_tree.DoingLaundry.cli:main",
    "follow-person = behavior_tree.FollowPerson.cli:main",
    "gpsr = behavior_tree.GPSR.cli:main",
    "help-me-carry = behavior_tree.HelpMeCarry.cli:main",
    "hri = behavior_tree.HRI.cli:main",
    "inspection = behavior_tree.Inspection.cli:main",
    "pick-and-place = behavior_tree.PickAndPlace.cli:main",
    "receptionist = behavior_tree.Receptionist.cli:main",
    "restaurant = behavior_tree.Restaurant.cli:main",
    "serve-breakfast = behavior_tree.ServeBreakfast.cli:main",
    "store-groceries = behavior_tree.StoringGroceries.cli:main",
    "draw = behavior_tree.tools.draw:main",
    "fetch-points = behavior_tree.tools.fetch_points:main",
    "verify-task-endpoints = behavior_tree.tools.verify_task_endpoints:main",
]


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + PACKAGE_NAME],
        ),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        ("share/behavior_tree/launch", glob("launch/*.launch.py")),
        ("share/" + PACKAGE_NAME + "/config", glob("config/*.json")),
    ],
    package_data={
        "behavior_tree": ["mock_config.json"],
        "behavior_tree.DoingLaundry": ["constants.json"],
        "behavior_tree.GPSR": ["constants.json"],
        "behavior_tree.GPSR.supervision": ["fixtures/*"],
        "behavior_tree.HelpMeCarry": ["constants.json"],
        "behavior_tree.HRI": ["constants.json"],
        "behavior_tree.Inspection": ["constants.json"],
        "behavior_tree.PickAndPlace": ["constants.json"],
        "behavior_tree.Receptionist": ["constants.json"],
        "behavior_tree.Restaurant": ["constants.json"],
        "behavior_tree.StoringGroceries": ["constants.json"],
    },
    install_requires=["setuptools"],
    extras_require={
        "gpsr": ["openai>=1.0"],
        "gpsr-supervisor": ["openai>=1.0", "Pillow>=9.0"],
    },
    zip_safe=True,
    maintainer="Tinker Team",
    maintainer_email="cindy.w0135@gmail.com",
    description="RoboCup behaviour-tree mission orchestration",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": CONSOLE_SCRIPTS},
)
