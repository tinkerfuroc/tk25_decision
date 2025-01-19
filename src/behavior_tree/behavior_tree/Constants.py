import math
# service names (matching the mocked services, please prioritize changing the service names in your own node INSTEAD of changing them here)
SRV_ANNOUNCE = "announce"
SRV_DROP = "start_drop"
SRV_GOTO = "go_to"
SRV_GOTO_GRASP = "go_to_grasp"
SRV_GRASP = "start_grasp"
# SRV_GRASP = "grasp"
SRV_OBJ_DETECTION = "object_detection"
# SRV_REL_TO_ABS = "rel_to_abs"
SRV_WAIT_FOR_START = "wait_for_start"
SRV_MOVE_ARM = "arm_joint_service"

PRINT_BLACKBOARD = False
PRINT_DEBUG = True

DROP_EXISTS = True
WITHOUT_NAV_CONSTANTS = True
RUNNING_NAV = True

# joint poses in degrees
SCAN_POSES_D = [
    [0.0, 0.0, 0.0, 0.0, 0.0, -45.0, 0.0],
    [0.0, -2.2, 34.0, 9.4, 0.2, -42.9, 0.0],
    [0.0, -2.2, -34.0, 9.4, 0.2, -42.9, 0.0]
    # [0.0, 50.7, 0.0, 12.6, 0.0, -87.8, 0.0],
    # [0.0, 65.6, 0.0, 38.4, 0.0, -87.9, 0.0],
    # [0., 0.6, 0., -0.3, 0., -44.4, 0.],
    # [0.0, 62.8, 0.4, 48.2, -4.4, -85.3, 0.0]
    # [1.2 , -15.8 , 0.0 , -1.5,1.0,-32.4,0.0]
    # [0., 0.6, 0, 20.2, 0., -24.3, 1.4],
    # [0., 13.3, 0, 44.2, 0., -11.2, 1.4],
    # [-0.8, -9.6, 3, 59.4, -4.7, 34.8, 4.5]
]
SCAN_POSES = [
    [x / 180 * math.pi for x in p] for p in SCAN_POSES_D
]

# TODO: change this to the actual pose
LOOK_POSE_D = [0.0, 28, 0.0, 3, 0.0, -84, 0.0]
LOOK_POSE = [x / 180 * math.pi for x in LOOK_POSE_D]

INITIAL_POSE2_D = [-87.0, -59.0, -2.0, -3.0, 12.0, -85.0, -8.0]
INITIAL_POSE2 = [x / 180 * math.pi for x in INITIAL_POSE2_D]
