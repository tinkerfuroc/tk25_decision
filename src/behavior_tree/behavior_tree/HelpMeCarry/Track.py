import py_trees

from behavior_tree.TemplateNodes.Vision import BtNode_TrackPerson
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoGrasp
from .nodes import BtNode_ProcessTrack

TRACKING_NAMESPACE = "track"
TRACKING_POINT_KEY = "person"

def createFollowPerson():
    track = py_trees.composites.Selector(name="Track Person", memory=True)
    track_repeat = py_trees.decorators.Repeat(name="Repeat tracking indefinitely", child=track, num_success=-1)
    track.add_child(BtNode_TrackPerson(name="Detect Person and Save", bb_namespace=TRACKING_NAMESPACE, bb_key=TRACKING_POINT_KEY))
    track.add_child(py_trees.behaviours.Success(name="Dummy for movement if person is lost"))

    follow = py_trees.composites.Sequence(name="Follow", memory=True)
    follow_inverter = py_trees.decorators.Inverter(name="follow inverter", child=follow)
    follow_repeat = py_trees.decorators.Repeat(name="Repeat follow until success", child=follow_inverter, num_success=-1)
    follow.add_child(BtNode_ProcessTrack(name="Process track results", bb_namespace=TRACKING_NAMESPACE, bb_key_source=TRACKING_POINT_KEY, threshold_m=0.2, threshold_frames=3))
    follow.add_child(BtNode_GotoGrasp(name="Goto grasp position", bb_source=TRACKING_NAMESPACE+"/"+TRACKING_POINT_KEY))

    return py_trees.composites.Parallel(name="Follow Person", policy=py_trees.common.ParallelPolicy.SuccessOnSelected([follow_repeat]), children=[track_repeat, follow_repeat])