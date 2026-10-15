# TK27 manipulation callers

The behavior tree uses tk27_manipulation_msgs for grasp, placement and joint motion.
BtNode_Grasp retains its tree class name for mock configuration and supervision,
but its ROS Action type is Pick.

- /manipulation/pick sends prompt and optional plan_effort only. Text comes from
  a prompt key, explicit object label, literal prompt or the first detected
  object's class in existing discovery trees. Images, masks, use_mesh and stay
  are not sent. TK27 obtains a fresh vision capture and finishes after lift.
- /manipulation/place sends a framed TCP PoseStamped, retain_grasp_orientation
  and observed environment points. The existing point and orientation keys are
  converted at the adapter boundary. None or an internal zero orientation marker
  explicitly selects the coordinator's cached Pick orientation. The wire pose
  always has a valid quaternion.
- Both nodes require ROS STATUS_SUCCEEDED, success=true and status=0.
  Failures use error_msg, not the removed result.stage field.
- Feedback uses progress.stage/detail/elapsed_s; it has no delay_limit/status
  fields. ActionBase's timeout and cancellation behavior remains in use.
- BtNode_GraspWithPose writes None after success instead of fabricating a pose
  absent from the minimal Pick result.

Place finishes after release and retreat, without returning. It requires the
same coordinator to track a held object from its earlier Pick; a manually handed
object is not a tracked Pick result. No MoveToSavedPose caller existed in the
tree at migration time.

JointMove callers use /manipulation/joint_move with named joint_names/positions.
Existing seven-element decision poses map to joint1..joint7 in radians; named
pose dictionaries support other configured groups. The legacy service_name and
add_octomap constructor arguments remain accepted for tree compatibility, but
collision checking is always enabled and no bypass is sent. Feedback uses
stage/detail, and success requires both ROS and business success. JointMove
preserves a tracked object from Pick for later Place.

CartesianMove, Drop, Fold/FoldClothing and ScanAndPlace remain separate
legacy capabilities. They have no equivalent in the TK27 business API and are
not silently mapped to Pick, Place or empty-gripper MoveToSavedPose. This change
does not remove every legacy dependency from all competition trees.

behavior_tree declares its tk27_manipulation_msgs dependency. tkbuild --all now
builds decision after TK27 and vision. Rebuild with:

    cd /home/tinker/tk25_ws
    source install/setup.bash
    ./tkbuild tk25_decision --packages-select behavior_tree

Validation: 63 focused adapter, placement-tree and cancellation tests passed.
The installed module was imported outside its source tree and its Action types
and endpoints were checked. The broader supervision suite has an existing
unregistered BtNode_CheckLastNavMatchesTarget guard; the same failure was
reproduced against pre-change HEAD in an isolated source copy. No GPU service,
simulator or physical motion was started.
