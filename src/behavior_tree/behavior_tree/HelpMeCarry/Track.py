import py_trees

from behavior_tree.TemplateNodes.Vision import BtNode_TrackPerson
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoGrasp
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from .nodes import BtNode_ProcessTrack

TRACKING_NAMESPACE = "track"
TRACKING_POINT_KEY = "person"

def createFollowPerson():
    track = py_trees.composites.Selector(name="Track Person", memory=True)
    track_repeat = py_trees.decorators.Repeat(name="Repeat tracking indefinitely", child=track, num_success=-1)
    track.add_child(BtNode_TrackPerson(name="Detect Person and Save", bb_namespace=TRACKING_NAMESPACE, bb_key=TRACKING_POINT_KEY))
    # TODO: add node for what to do when vision looses track of person
    track.add_child(py_trees.behaviours.Success(name="Dummy for movement if person is lost"))

    follow = py_trees.composites.Sequence(name="Follow", memory=True)
    follow_inverter = py_trees.decorators.Inverter(name="follow inverter", child=follow)
    follow_repeat = py_trees.decorators.Retry(name="Repeat follow until success", child=follow_inverter, num_failures=999)
    follow.add_child(BtNode_ProcessTrack(name="Process track results", bb_namespace=TRACKING_NAMESPACE, bb_key_source=TRACKING_POINT_KEY, threshold_m=0.2, threshold_frames=3))
    follow.add_child(BtNode_Announce(name="Announce person updated", bb_source=None, message="Person identified"))
    follow.add_child(BtNode_GotoGrasp(name="Goto grasp position", bb_source=TRACKING_NAMESPACE+"/"+TRACKING_POINT_KEY))
    follow.add_child(BtNode_Announce(name="Announce reached destination", bb_source=None, message="Searching again"))

    return py_trees.composites.Parallel(name="Follow Person", policy=py_trees.common.ParallelPolicy.SuccessOnSelected([follow_repeat]), children=[track_repeat, follow_repeat])
    # return py_trees.composites.Parallel(name="Follow Person", policy=py_trees.common.ParallelPolicy.SuccessOnAll(), children=[track_repeat, follow_repeat])

def createFollowPersonAudio():
    root = py_trees.composites.Sequence(name="root", memory=True)

    root.add_child(BtNode_Announce(name="Announce start", bb_source=None, message="Starting to follow person"))
    follow = createFollowPerson()
    follow_wrapper = py_trees.composites.Selector(name="wrapper", memory=True, children=[follow, BtNode_Announce(name="follow failed", message="Following had failed")])
    root.add_child(follow_wrapper)
    root.add_child(BtNode_Announce(name="Announce finished", bb_source=None, message="Stopped following"))
    root.add_child(py_trees.behaviours.Running(name="Finished"))

    return root
