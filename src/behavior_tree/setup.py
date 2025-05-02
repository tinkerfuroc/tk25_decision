from setuptools import find_packages, setup

package_name = 'behavior_tree'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cindy',
    maintainer_email='cindy.w0135@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'draw = behavior_tree.main:main',
            'follow = behavior_tree.main:test_follow',
            'follow-audio = behavior_tree.main:test_follow_audio',
            'follow-action = behavior_tree.main:test_follow_action',
            'test-track = behavior_tree.main:test_follow',
            'receptionist = behavior_tree.main:receptionist',
            "grasp-intel = behavior_tree.main:grasp_intel",
            'grasp-audio = behavior_tree.main:grasp_audio',
            'serve-breakfast = behavior_tree.main:serve_breakfast',
            'store-groceries = behavior_tree.main:store_groceries',
            'store-groceries-placing-only = behavior_tree.main:store_groceries_placing_only',
            'GPSR = behavior_tree.main:gpsr',
        ],
    },
)
