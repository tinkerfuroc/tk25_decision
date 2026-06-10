import py_trees
import rclpy
import py_trees_ros

from behavior_tree.TemplateNodes.BaseBehaviors import ServiceHandler
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle, BtNode_Grasp
from behavior_tree.TemplateNodes.Audio import (
    BtNode_Announce
)
from tinker_vision_msgs_26.srv import ObjectMatch

import math

arm_pos_navigating = [x / 180.0 * math.pi for x in [
    -87.0,
    -58.8,
    -2.4,
    8.0,
    12.3,
    -68.5,
    -8.2
  ]
]
arm_pos_table = [x / 180.0 * math.pi for x in
[
    0.0,
    -43.0,
    4.0,
    73.0,
    0.0,
    71.0,
    5.0
  ]
]

KEY_ARM_POS_NAV = "NAV"
KEY_ARM_POS_SCAN = "SCAN"

KEY_PROMPT='PROMPT'
KEY_VIS_RESULT='VIS RESULT'

class BtNode_MatchingService(ServiceHandler):
    def __init__(self,
                 name:str,
                 bb_result_key:str,
                 bb_prompt_key:str,
                 service_name:str = "object_match",
                 camera:str = "realsense",
                 target_frame = ""
                 ):
        super().__init__(name, service_name, ObjectMatch)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="prompt",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_prompt_key),
        )
        self.blackboard.register_key(
            key="result",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_result_key),
        )
        self.camera = camera
        self.target_frame = target_frame
    
    def initialise(self):
        if not self.blackboard.exists("prompt"):
            self.feedback_message = "prompt does not exist on black board"
            return py_trees.common.Status.FAILURE

        category = self.blackboard.prompt

        request = ObjectMatch.Request()
        request.category = category
        request.camera = self.camera
        request.target_frame = self.target_frame

        self.response = self.call_service_async(request)
        self.feedback_message = f"Initialized ObjectMatch"
    
    def update(self):
        if self.response is None:
            self.feedback_message = "No response object"
            return py_trees.common.Status.FAILURE

        if self.response.done():
            result = self.response.result()
            if result.status == 0:
                self.blackboard.result= result
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.FAILURE
        self.feedback_message = "Still scanning (generalist)..."
        return py_trees.common.Status.RUNNING 

def writeToBlackBoard():
    root=py_trees.composites.Sequence(
        "writing to blackboard",
        True
    )

    root.add_child(
        BtNode_WriteToBlackboard(
        name="write arm pose navigating",
        bb_namespace="", 
        bb_source=None, 
        bb_key=KEY_ARM_POS_NAV, 
        object=arm_pos_navigating
        )
    )

    root.add_child(
        BtNode_WriteToBlackboard(
        name="write arm pose scan",
        bb_namespace="", 
        bb_source=None, 
        bb_key=KEY_ARM_POS_SCAN, 
        object=arm_pos_table
        )
    )

    root.add_child(
        BtNode_WriteToBlackboard(
        name="write arm pose scan",
        bb_namespace="", 
        bb_source=None, 
        bb_key=KEY_PROMPT, 
        object='biscuit'
        )
    )
    return root 

def graspOnceForTarget(target_key):
    root = py_trees.composites.Sequence(
        name="grasp once",
        memory=True
    )

    root.add_child(
        BtNode_MoveArmSingle(
            name="move arm to scan pose",
            arm_pose_bb_key=KEY_ARM_POS_SCAN
        )
    )

    # root.add_child(
    #     BtNode_Announce(
    #         "repeat order",
    #         bb_source= target_key,
    #         message="Grasping "
    #     )
    # )

    root.add_child(
        BtNode_MatchingService(
            name="match",
            bb_result_key=KEY_VIS_RESULT,
            bb_prompt_key=target_key
        )
    )

    root.add_child(
        BtNode_Grasp(
            name="grasp object on the table",
            bb_source=None,
            bb_key_vision_res=KEY_VIS_RESULT,
        )
    )

    root.add_child(
        BtNode_MoveArmSingle(
            name="move arm to nav pose",
            arm_pose_bb_key=KEY_ARM_POS_NAV
        )
    )

    return root

def testGrasp():
    root = py_trees.composites.Sequence(
        name="test grasp",
        memory=True
    )

    root.add_child(writeToBlackBoard())

    root.add_child(graspOnceForTarget(KEY_PROMPT))

    root.add_child(
        BtNode_MoveArmSingle(
            name="move arm to nav pose",
            arm_pose_bb_key=KEY_ARM_POS_NAV
        )
    )

    return root

def main():
    rclpy.init()
    root = testGrasp()
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name="test_scan", timeout=15)

    def _print(t):
        print(py_trees.display.unicode_tree(root=t.root, show_status=True))

    tree.tick_tock(period_ms=500.0, post_tick_handler=_print)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()



