from setuptools import find_packages, setup

package_name = 'behavior_tree'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={
        'behavior_tree': ['mock_config.json'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cindy',
    maintainer_email='cindy.w0135@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_move_arm = behavior_tree.test_move_arm_teleop:main',
            'restaurant-demo = behavior_tree.Restaurant.restaurants_fake:main',
            'gpsr-demo = behavior_tree.GPSR.gpsr_2ndcall:main',
            'receptionist_new = behavior_tree.Receptionist.receptionist_new:main',
            'receptionist_secondcall = behavior_tree.Receptionist.receptionist_2ndcall:main',
            'draw = behavior_tree.cli_draw:main',
            'follow = behavior_tree.HelpMeCarry.cli:follow',
            'follow-audio = behavior_tree.HelpMeCarry.cli:follow_audio',
            'follow-action = behavior_tree.HelpMeCarry.cli:follow_action',
            'test-track = behavior_tree.HelpMeCarry.cli:follow',
            'receptionist = behavior_tree.Receptionist.cli:main',
            "grasp-intel = behavior_tree.grasp_intel_demo.cli:grasp_intel",
            'grasp-audio = behavior_tree.grasp_intel_demo.cli:grasp_audio',
            'serve-breakfast = behavior_tree.ServeBreakfast.cli:main',
            'store-groceries = behavior_tree.StoringGroceries.cli:main',
            'store-groceries-placing-only = behavior_tree.StoringGroceries.cli:placing_only',
            'help-me-carry = behavior_tree.HelpMeCarry.cli:help_me_carry',
            'test_follow_head = behavior_tree.Receptionist.test_follow_head:main',
            'test-prompt-reached = behavior_tree.HelpMeCarry.cli:test_prompt_reached',
            'inspection = behavior_tree.Inspection.cli:main',
            'hri = behavior_tree.HRI.cli:main',
            'GPSR = behavior_tree.GPSR.gpsr_new:main',
            'EGPSR = behavior_tree.GPSR.egpsr:main',
            'restaurant = behavior_tree.Restaurant.cli:main',
            'restaurant-simplified = behavior_tree.Restaurant.cli:simplified',
            'grasp_once = behavior_tree.grasp_intel_demo.grasp:main',
            # Mock mode test scripts
            'test-mock-mode = behavior_tree.test_mock_mode:main',
            'test-mockable-wrapper = scripts.test_mockable_wrapper:main',
            # HelpMeCarry test scripts
            'hmc-mock-nav = behavior_tree.HelpMeCarry.test.mock_nav_server:main',
            'hmc-mock-track = behavior_tree.HelpMeCarry.test.mock_track_server:main',
            'hmc-test-follow = behavior_tree.HelpMeCarry.test.test_follow:main',
            'hmc-standalone = behavior_tree.HelpMeCarry.test.standalone_test:main',
        ],
    },
)
