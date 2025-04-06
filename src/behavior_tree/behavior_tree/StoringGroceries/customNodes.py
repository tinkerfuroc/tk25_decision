import py_trees
from behavior_tree.TemplateNodes.BaseBehaviors import ServiceHandler
from behavior_tree.messages import ObjectDetection

class BtNode_FindObjTable(ServiceHandler):
    """
    Find object on table
    """

    def __init__(self, name: str, 
                 bb_key_prompt: str, 
                 bb_key_image: str, 
                 bb_key_segment: str, 
                 bb_key_result: str,
                 target_frame: str = "base_link",
                 use_realsense: bool = True):
        super(BtNode_FindObjTable, self).__init__(name=name,
                                                  service_name="find_object",
                                                  service_type="behavior_tree/FindObject")
        self.bb_key_prompt = bb_key_prompt
        self.bb_key_image = bb_key_image
        self.bb_key_segment = bb_key_segment
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="prompt",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_prompt)
        )
        self.blackboard.register_key(
            key="image",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_image)
        )
        self.blackboard.register_key(
            key="segmentation",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_segment)
        )
        self.blackboard.register_key(
            key="result",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_result)
        )
        self.use_realsense = use_realsense

    def initialise(self):
        request = ObjectDetection.Request()
        request.prompt = self.blackboard.prompt
        request.flags = "find_for_grasp|request_image|request_segmentation"
        if self.use_realsense:
            request.camera = "realsense"
        else:
            request.camera = "orbecc"
        self.response = self.client.call_async(request)
        self.logger.debug(f"Initialized FindObjTable with prompt: {self.blackboard.prompt}")

    def update(self):
        self.logger.debug(f"Updating FindObjTable with prompt: {self.blackboard.prompt}")
        if self.response.done():
            if self.response.result.status == 0:
                response = self.response.result()
                self.blackboard.image = response.rgb_image
                self.blackboard.segmentation = response.segments[0]
                self.blackboard.result = response
                self.feedback_message = f"Found object: {response.objects[0].cls}"
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = f"Failed to find object with {self.response.result().status} and error message {self.response.result().error_msg}"
                return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = "Waiting for response from find object service"
            return py_trees.common.Status.RUNNING
