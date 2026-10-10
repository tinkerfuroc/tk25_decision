"""Node-level test: can BtNode_CaptureCurrentPose capture the robot's current
pose and store it under several distinct labels *within one run*?

We can't drive a real robot here, so we inject a fake TF buffer whose
``lookup_transform`` returns a pose we control. We then "move" the robot
between captures by changing what the fake returns, capture into three
different blackboard labels, and assert that all three coexist with the
correct, distinct values.

Run::

    source /home/tinker/tk25_ws/install/setup.zsh
    BT_MOCK_MODE=false \
        /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python \
        src/behavior_tree/behavior_tree/GPSR/test_capture_pose.py
"""

import os
import sys

# Force real (non-mock) path so the node actually does a TF lookup.
os.environ["BT_MOCK_MODE"] = "false"

import rclpy
import py_trees
from geometry_msgs.msg import TransformStamped
from py_trees.blackboard import Blackboard

from behavior_tree.TemplateNodes.Navigation import BtNode_CaptureCurrentPose


class FakeTfBuffer:
    """Stand-in for tf2_ros.Buffer that returns a controllable transform.

    Set ``.xyz`` / ``.qzw`` to simulate the robot standing somewhere; every
    lookup_transform returns that pose. This lets one process simulate the
    robot moving between captures.
    """

    def __init__(self):
        self.xyz = (0.0, 0.0, 0.0)
        self.qzw = (0.0, 1.0)
        self.calls = []

    def lookup_transform(self, target_frame, source_frame, time, timeout=None):
        self.calls.append((target_frame, source_frame))
        ts = TransformStamped()
        ts.header.frame_id = target_frame
        ts.child_frame_id = source_frame
        ts.transform.translation.x = self.xyz[0]
        ts.transform.translation.y = self.xyz[1]
        ts.transform.translation.z = self.xyz[2]
        ts.transform.rotation.z = self.qzw[0]
        ts.transform.rotation.w = self.qzw[1]
        return ts


def capture_at(node, fake, bb_key, label_name, xyz, qzw):
    """Build a capture node bound to bb_key, point it at `fake`, tick once."""
    cap = BtNode_CaptureCurrentPose(f"capture {label_name}", bb_key=bb_key)
    cap.mock_mode = False                     # ensure the real TF path
    cap.setup(node=node)                      # creates a real Buffer/listener...
    cap.tf_buffer = fake                      # ...which we replace with the fake
    fake.xyz, fake.qzw = xyz, qzw             # "move" the robot to this spot
    cap.initialise()
    status = cap.update()
    return status


def main() -> int:
    rclpy.init()
    node = rclpy.create_node("test_capture_pose")
    fake = FakeTfBuffer()

    # Three labels, three distinct robot poses, captured in sequence.
    spots = [
        ("gpsr/dynloc/alice",   "alice",   (1.0, 2.0, 0.0), (0.0, 1.0)),
        ("gpsr/dynloc/bob",     "bob",     (5.5, -3.0, 0.0), (0.707, 0.707)),
        ("gpsr/dynloc/charlie", "charlie", (-2.0, 4.25, 0.0), (1.0, 0.0)),
    ]

    print("Capturing 3 dynamic locations within one run:\n")
    statuses = {}
    for bb_key, label, xyz, qzw in spots:
        st = capture_at(node, fake, bb_key, label, xyz, qzw)
        statuses[label] = st
        print(f"  capture '{label}' -> {st.name}  (robot at {xyz[:2]})")

    # Read all three back from the (global, in-process) blackboard.
    reader = Blackboard()
    print("\nReading all labels back from the blackboard:\n")
    ok = True
    for bb_key, label, xyz, qzw in spots:
        try:
            pose = reader.get(bb_key)
        except Exception as exc:
            print(f"  [FAIL] {label}: key {bb_key} missing ({exc})")
            ok = False
            continue
        got = (round(pose.pose.position.x, 3), round(pose.pose.position.y, 3))
        want = (xyz[0], xyz[1])
        match = got == want
        ok = ok and match
        print(f"  {label:8} {bb_key:22} -> ({got[0]}, {got[1]})  "
              f"frame={pose.header.frame_id}  {'OK' if match else 'MISMATCH want '+str(want)}")

    # Cross-checks: distinctness + persistence of earlier captures.
    a = reader.get("gpsr/dynloc/alice")
    b = reader.get("gpsr/dynloc/bob")
    c = reader.get("gpsr/dynloc/charlie")
    distinct = len({(p.pose.position.x, p.pose.position.y) for p in (a, b, c)}) == 3
    alice_survived = (a.pose.position.x, a.pose.position.y) == (1.0, 2.0)

    print("\nChecks:")
    print(f"  all three statuses SUCCESS : "
          f"{all(s == py_trees.common.Status.SUCCESS for s in statuses.values())}")
    print(f"  three labels coexist       : {distinct}")
    print(f"  earliest (alice) survived later captures : {alice_survived}")
    print(f"  orientation captured (bob qz≈0.707)      : {round(b.pose.orientation.z,3)}")

    node.destroy_node()
    rclpy.shutdown()

    verdict = ok and distinct and alice_survived
    print(f"\n{'PASS' if verdict else 'FAIL'}: BtNode_CaptureCurrentPose "
          f"{'CAN' if verdict else 'CANNOT'} capture + store multiple distinct "
          f"labels within one run.")
    return 0 if verdict else 1


if __name__ == "__main__":
    sys.exit(main())
