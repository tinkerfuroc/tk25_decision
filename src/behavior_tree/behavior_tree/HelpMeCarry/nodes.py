from typing import Any
import py_trees
from rclpy.node import Node
import math

import time
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
                 threshold_m : float = 0.3,
                 threshold_t : float = 3.0
                 ):
        super(BtNode_ProcessTrack, self).__init__(name=name)
        
        self.bb_namespace = bb_namespace
        self.bb_key_source = bb_key_source
        self.threshold_m = threshold_m
        self.threshold_t = threshold_t

        self.last_point : PointStamped = None
        self.anchor_point : PointStamped = None
        self.anchor_time : float = time.time()
        self.point : PointStamped = None
        self.current_point : PointStamped = None
        self.moved = True
    
    def target_stopped(self):
        if self.anchor_point is None:
            self.anchor_point = self.current_point
            return False
        
        x1, y1 = self.current_point.point.x, self.current_point.point.y
        x2, y2 = self.anchor_point.point.x, self.anchor_point.point.y

        # pytagorean theorem to get distance
        distance = math.sqrt(((x2-x1) ** 2) + ((y2-y1) ** 2))

        if distance < self.threshold_m:
            self.moved = False
            if time.time() - self.anchor_time > self.threshold_t:
                return True
            return False
        else:
            self.moved  = True
            self.anchor_point = self.current_point
            self.anchor_time = time.time()
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
        self.logger.debug("Finished initializing")
    
    def update(self):
        self.logger.debug(f"Updating")

        try:
            self.current_point = self.bb_read_client.get(self.bb_key_source)
        except KeyError as e:
            self.logger.debug("person not found yet")
            self.current_point = None
            self.feedback_message = "Person not found yet"
            return py_trees.common.Status.RUNNING

        # first check if the point is the same as the previous one (aka vision was too slow)
        if (self.last_point is not None) and (self.current_point.header.stamp.sec == self.last_point.header.stamp.sec):
            self.feedback_message = f"Point has not been updated ({self.current_point.header.stamp.sec} vs {self.last_point.header.stamp.sec}), waiting..."
            return py_trees.common.Status.RUNNING
        
        self.last_point = self.current_point

        # then check if the person has stopped
        if self.target_stopped():
            self.feedback_message = f"Person stopped (within {self.threshold_m} meters for more than {self.threshold_t} seconds"
            return py_trees.common.Status.FAILURE
        
        if not self.moved:
            self.feedback_message = f"Person has been stopped within {self.threshold_m}m for {time.time() - self.anchor_time} seconds"
            return py_trees.common.Status.RUNNING
            # return py_trees.common.Status.SUCCESS
        
        # if the point has been updated and the person is still moving, return success and pass this on
        self.feedback_message = "Person location has been updated"
        return py_trees.common.Status.SUCCESS

        

        