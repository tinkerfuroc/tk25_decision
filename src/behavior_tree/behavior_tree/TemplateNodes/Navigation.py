import py_trees
from .BaseBehaviors import ServiceHandler

from geometry_msgs.msg import PointStamped, PoseStamped

from behavior_tree.messages import Goto, GotoGrasp
from nav_msgs.msg import Odometry

class BtNode_Goto(ServiceHandler):
    def __init__(self, 
                name: str,
                bb_source: str,
                service_name : str = "goto",
                target : PoseStamped = None
                ):
        """
        executed when creating tree diagram, therefor very minimal
        Args:
            name: the name of the pytree node
            bb_source: path to the key in blackboard containing a geometry_msgs/PoseStamped object of the pos of trash can
            service_name: name of the service of type tinker_decision_msgs/Drop     
        """
        super(BtNode_Goto, self).__init__(name, service_name, Goto)
        self.bb_source = bb_source
        self.bb_read_client = None
        self.target = target
        if target is not None:
            assert isinstance(self.target, PoseStamped)
        

    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)

        if self.target is None:
            self.bb_read_client = self.attach_blackboard_client(name="Goto Read")
            self.bb_read_client.register_key(self.bb_source, access=py_trees.common.Access.READ)

            # debugger info (shown with DebugVisitor)
            self.logger.debug(f"Setup Goto, reading from {self.bb_source}")
        else:
            self.logger.debug(f"Setup Goto from fixed input {self.target}")

    def initialise(self) -> None:
        """
        Called when the node is visited
        """

        if self.target is None:
            try:
                self.target = self.bb_read_client.get(self.bb_source)
                assert isinstance(self.target, PoseStamped)
            except Exception as e:
                self.feedback_message = f"Goto reading target pose failed"
                raise e

        self.logger.debug(f"Initialized Goto for pose {self.target}")

        request = Goto.Request()
        request.target = self.target
        # setup things that needs to be cleared
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized Goto"

    def update(self):
        self.logger.debug(f"Update Goto")
        if self.response.done():
            if self.response.result().status == 0:
                self.feedback_message = f"Goto Successful"
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = f"Goto failed with status {self.response.result().status}: {self.response.result().error_msg}"
                return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = "Still navigating to target pose..."
            return py_trees.common.Status.RUNNING


class BtNode_GotoGrasp(ServiceHandler):
    def __init__(self, 
                name: str,
                bb_source: str,
                service_name : str = "goto_grasp",
                target : PointStamped = None
                ):
        """
        executed when creating tree diagram, therefor very minimal
        Args:
            name: the name of the pytree node
            bb_source: path to the key in blackboard containing a geometry_msgs/PoseStamped object of the pos of trash can
            service_name: name of the service of type tinker_decision_msgs/Drop     
        """
        super(BtNode_GotoGrasp, self).__init__(name, service_name, GotoGrasp)
        self.bb_source = bb_source
        self.bb_read_client = None
        self.target = target
        self.read = True
        if target is not None:
            assert isinstance(self.target, PointStamped)
            self.read = False


    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)

        if self.read:
            self.bb_read_client = self.attach_blackboard_client(name="GotoGrasp Read")
            self.bb_read_client.register_key(self.bb_source, access=py_trees.common.Access.READ)

            # debugger info (shown with DebugVisitor)
            self.logger.debug(f"Setup GotoGrasp, reading from {self.bb_source}")
        else:
            self.logger.debug(f"Setup GotoGrasp from fixed point {self.target}")

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        if self.read:
            try:
                self.target = self.bb_read_client.get(self.bb_source)
                assert isinstance(self.target, PointStamped)
                self.read = True
            except Exception as e:
                self.feedback_message = f"GotoGrasp reading target point failed"
                raise e

        self.logger.debug(f"Initialized GotoGrasp for target point {self.target}")

        request = GotoGrasp.Request()
        request.target = self.target
        # setup things that needs to be cleared
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized GotoGrasp, read set to {self.read}"

    def update(self):
        self.logger.debug(f"Update GotoGrasp")
        if self.response.done():
            self.feedback_message = f"GotoGrasp returned with status {self.response.result().status}"
            return py_trees.common.Status.SUCCESS
            # if self.response.result().status == 0:
            #     self.feedback_message = f"GotoGrasp Successful"
            #     return py_trees.common.Status.SUCCESS
            # else:
            #     self.feedback_message = f"GotoGrasp failed with status {self.response.result().status}: {self.response.result().error_msg}"
            #     return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = f"Still navigating to grasping pose {self.target} from {self.bb_source} (read set to {self.read})"
            return py_trees.common.Status.RUNNING


class BtNode_RelToAbs(ServiceHandler):
    def __init__(self, 
            name: str,
            bb_key_sourse: str,
            bb_key_result: str,
            is_point: bool,
            service_name : str = "rel_to_abs",
            target : PointStamped | PoseStamped = None
            ):
        """
        executed when creating tree diagram, therefor very minimal
        """
        super(BtNode_RelToAbs, self).__init__(name, service_name, RelToAbs)
        self.bb_key_source = bb_key_sourse
        self.bb_key_result = bb_key_result
        self.is_point = is_point
        self.bb_read_client = None
        self.bb_write_client = None
        self.target = target
        self.read = True
        if target is not None:
            assert (isinstance(target, PointStamped) or isinstance(target, PoseStamped))
            self.read = False


    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)

        self.bb_write_client = self.attach_blackboard_client(name="RelToAbs Write")
        self.bb_write_client.register_key(self.bb_key_result, access = py_trees.common.Access.WRITE)


        if self.read:
            self.bb_read_client = self.attach_blackboard_client(name="RelToAbs Read")
            self.bb_read_client.register_key(self.bb_key_source, access=py_trees.common.Access.READ)

            # debugger info (shown with DebugVisitor)
            self.logger.debug(f"Setup RelToAbs, reading from {self.bb_key_source} and writing to {self.bb_key_result}")
        else:
            self.logger.debug(f"Setup RelToAbs from fixed Point/Pose and writing to {self.bb_key_result}")

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        if self.read:
            try:
                self.target = self.bb_read_client.get(self.bb_key_source)
                assert (isinstance(self.target, PointStamped) or isinstance(self.target, PoseStamped))
            except Exception as e:
                self.feedback_message = f"RelToAbs reading target point/pose failed"
                raise e

        self.logger.debug(f"Initialized RelToAbs for target point/pose {self.target}")

        request = RelToAbs.Request()
        if self.is_point:
            request.flags = "point"
            request.point_rel = self.target
        else:
            request.flags = "pose"
            request.pose_rel = self.target
        # setup things that needs to be cleared
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized RelToAbs"

    def update(self):
        self.logger.debug(f"Update RelToAbs")
        if self.response.done():
            if self.response.result().status == 0:
                self.feedback_message = f"RelToAbs Successful"
                if self.is_point:
                    self.bb_write_client.set(self.bb_key_result, self.response.result().point_abs, overwrite=True)
                else:
                    self.bb_write_client.set(self.bb_key_result, self.response.result().pose_abs, overwrite=True)
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = f"RelToAbs failed with status {self.response.result().status}: {self.response.result().error_msg}"
                return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = "Still converting pose/point"
            return py_trees.common.Status.RUNNING


class BtNode_Turn(ServiceHandler):
    def __init__(self, name: str, service_name: str):
        super().__init__(name, service_name, Goto)
        self.current_pos = None
    
    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.odom_sub = self.create_subscription(Odometry, '/rtabmap/odom', self.odomCallback, 1)
        
