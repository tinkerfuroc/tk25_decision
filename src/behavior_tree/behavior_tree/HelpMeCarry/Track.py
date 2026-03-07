import py_trees

from behavior_tree.TemplateNodes.Vision import BtNode_TrackPerson
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from .nodes import BtNode_ProcessTrack

TRACKING_NAMESPACE = "track"
TRACKING_POINT_KEY = "person"
GRASP_POSE_KEY = "goal_pose"

def createFollowPerson():
    track = py_trees.composites.Selector(name="Track Person", memory=True)
    track_repeat = py_trees.decorators.Repeat(name="Repeat tracking indefinitely", child=track, num_success=-1)
    track.add_child(BtNode_TrackPerson(name="Detect Person and Save", bb_namespace=TRACKING_NAMESPACE, bb_key=TRACKING_POINT_KEY))
    
    # TODO: add node for what to do when vision looses track of person
    # track.add_child(py_trees.behaviours.Success(name="Dummy for movement if person is lost"))

    follow = py_trees.composites.Sequence(name="Follow", memory=True)
    follow_inverter = py_trees.decorators.Inverter(name="follow inverter", child=follow)
    follow_repeat = py_trees.decorators.Retry(name="Repeat follow until success", child=follow_inverter, num_failures=999)
    follow.add_child(BtNode_ProcessTrack(name="Process track results", bb_namespace=TRACKING_NAMESPACE, bb_key_source=TRACKING_POINT_KEY, threshold_m=0.2, threshold_t=5.0))
    follow.add_child(py_trees.decorators.FailureIsSuccess("FisS", BtNode_GotoAction(name="Goto person position", key=TRACKING_NAMESPACE+"/"+TRACKING_POINT_KEY)))
    
    return py_trees.composites.Parallel(name="Follow Person", policy=py_trees.common.ParallelPolicy.SuccessOnAll([follow_repeat]), children=[track_repeat, follow_repeat])
    # return py_trees.composites.Parallel(name="Follow Person", policy=py_trees.common.ParallelPolicy.SuccessOnAll(), children=[track_repeat, follow_repeat])

# def createRepeatTrack():
#     # track = py_trees.composites.Selector(name="Track Person", memory=True)
#     single_track = BtNode_TrackPerson(name="Detect Person and Save", bb_namespace=None, bb_key=TRACKING_POINT_KEY)
#     # track_repeat = py_trees.decorators.Repeat(name="Repeat tracking indefinitely", child=track, num_success=-1)
#     track_repeat = py_trees.decorators.Repeat(name="Repeat tracking indefinitely", child=py_trees.decorators.FailureIsSuccess(name="fail is success", child=single_track), num_success=-1)
#     # track.add_child(single_track)
#     # TODO: add node for what to do when vision looses track of person
#     # track.add_child(py_trees.behaviours.Success(name="Dummy for movement if person is lost"))


#     # lost_announce = BtNode_Announce(name="Announce lost person", bb_source=None, message="Out of camera")
#     # dummy = py_trees.behaviours.Success(name="Dummy for movement if person is lost")
#     # track.add_child(py_trees.composites.Parallel(name="Lost", policy=py_trees.common.ParallelPolicy.SuccessOnSelected([dummy]), children=[lost_announce, dummy]))

#     return track_repeat

# def createFollowPersonAction():
#     track_repeat = createRepeatTrack()
    
#     follow = py_trees.composites.Sequence(name="Follow", memory=True)
#     follow_inverter = py_trees.decorators.Inverter(name="follow inverter", child=follow)
#     follow_repeat = py_trees.decorators.Retry(name="Repeat follow until success", child=py_trees.decorators.Timeout(name="timeout after 8 sec", child=follow_inverter, duration=8.0), num_failures=999)
    
#     search_announce = BtNode_Announce(name="Announce searching", bb_source=None, message="Searching")
#     process_track = BtNode_ProcessTrack(name="Process track results", bb_namespace=None, bb_key_source=TRACKING_POINT_KEY, threshold_m=0.2, threshold_t=20.0)
#     search = py_trees.composites.Parallel(name="Search", policy=py_trees.common.ParallelPolicy.SuccessOnSelected([process_track]), children=[search_announce, process_track])
#     follow.add_child(search)

#     update_announce = BtNode_Announce(name="Announce person found", bb_source=None, message="Found")
#     calc_pose = BtNode_CalcGraspPose(name="Calc grasp pose", bb_source=TRACKING_POINT_KEY, bb_dest=GRASP_POSE_KEY)
#     update = py_trees.composites.Parallel(name="Update", policy=py_trees.common.ParallelPolicy.SuccessOnSelected([calc_pose]), children=[update_announce, calc_pose])
#     follow.add_child(update)

#     follow.add_child(BtNode_GotoAction(name="Goto pose", key=GRASP_POSE_KEY))
#     # follow.add_child(BtNode_Announce(name="Announce reached destination", bb_source=None, message="Reached"))

#     return py_trees.composites.Parallel(name="Follow Person", policy=py_trees.common.ParallelPolicy.SuccessOnSelected([follow_repeat]), children=[track_repeat, follow_repeat])

# def createFollowPersonAudio(action=False):
#     root = py_trees.composites.Sequence(name="root", memory=True)

#     root.add_child(BtNode_Announce(name="Announce start", bb_source=None, message="Starting to follow person"))
#     follow = createFollowPerson()
#     if action:
#         follow = createFollowPersonAction()        
#     follow_wrapper = py_trees.composites.Selector(name="wrapper", memory=True, children=[follow, BtNode_Announce(name="follow failed", bb_source=None, message="Following had failed")])
#     root.add_child(follow_wrapper)
#     root.add_child(BtNode_Announce(name="Announce finished", bb_source=None, message="Stopped following"))
#     root.add_child(py_trees.behaviours.Running(name="Finished"))

#     return root
