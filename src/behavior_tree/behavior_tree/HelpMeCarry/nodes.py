from typing import Any
import py_trees
from rclpy.node import Node
import math

from behavior_tree.messages import PointStamped

class BtNode_ProcessTrack(py_trees.behaviour.Behaviour):
    """
    Reads a PointStamped target of the centroid of the person from bb_key_source in bb_namespace
    Checks that it has been updated. returns RUNNING is there is no update, and FAILURE is the person has stopped
    """
    def __init__(self,
                 name: str,
                 bb_namespace: str,
                 bb_key_source: str,
                 threshold_m : float = 0.2,
                 threshold_frames :int = 3
                 ):
        super(BtNode_ProcessTrack, self).__init__(name=name)
        
        self.bb_namespace = bb_namespace
        self.bb_key_source = bb_key_source
        self.threshold_m = threshold_m
        self.threshold_frames = threshold_frames

        self.last_point : PointStamped = None
        self.point : PointStamped = None
        self.counter = 0
    
    def target_stopped(self):
        if self.last_point is None:
            return False
        
        x1, y1 = self.point.point.x, self.point.point.y
        x2, y2 = self.last_point.point.x, self.last_point.point.y

        # pytagorean theorem to get distance
        distance = math.sqrt(((x2-x1) ** 2) + ((y2-y1) ** 2))

        if distance < self.threshold_m:
            self.counter += 1
            if self.counter >= self.threshold_frames:
                return True
            return False
        else:
            self.counter = 0
            self.last_point = self.point
            return False


    def setup(self, **kwargs: Any) -> None:
        try:
            self.node : Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        self.bb_read_client = self.attach_blackboard_client(name=f"Read {self.name}", namespace=self.bb_namespace)
        self.bb_read_client.register_key(self.bb_key_source, access=py_trees.common.Access.READ)

        self.logger.debug(f'setup complete')
        
    def initialise(self) -> None:
        self.point = self.bb_read_client.get(self.bb_key_source)
        
        self.logger.debug("Finished initializing")
    
    def update(self):
        self.logger.debug(f"Updating")

        # first check if the point is the same as the previous one (aka vision was too slow)
        if self.point.header.stamp.sec == self.last_point.header.stamp.sec:
            self.point = self.bb_read_client.get(self.bb_key_source)
            self.feedback_message = "Point has not been updated, waiting..."
            return py_trees.common.Status.RUNNING

        # then check if the person has stopped
        if self.target_stopped():
            self.feedback_message = f"Person stopped (within {self.threshold_m} meters for {self.threshold_frames} updates)"
            return py_trees.common.Status.FAILURE
        
        # if the point has been updated and the person is still moving, return success and pass this on
        self.feedback_message = "Person location has been updated"
        return py_trees.common.Status.SUCCESS

        

        