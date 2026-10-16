# YOLO26 detection in tk25_decision

`BtNode_ScanForYolo26` lives in
`behavior_tree.TemplateNodes.Vision` and calls
`/object_detection_yolo26` using
`tinker_vision_msgs_26/srv/ObjectDetectionGeneralist`.
The checkpoint and GPU inference live in the separate tk26_vision server.
The decision environment does not need Ultralytics or torch.

## Start the server and build the decision package

On tkmr:

```bash
cd /home/tinker/tk25_ws
./tkbuild tk25_decision --packages-select behavior_tree
source install/setup.bash
ros2 run object_detection_generalist yolo26_seg_node
```

The server defaults to `yolo26s-seg.pt`. Append
`--ros-args -p model_path:=yolo26m-seg.pt` or an absolute custom checkpoint
path to change the model. Inspect checkpoint labels with:

```bash
ros2 param get /yolo26_seg_node labels
```

## Add a node to a behavior tree

```python
from behavior_tree.TemplateNodes.Vision import BtNode_ScanForYolo26

# All checkpoint labels, nearest first; stores the entire response.
scan_all = BtNode_ScanForYolo26(
    name="ScanAllYolo26",
    bb_key="detected_objects",
)

# Only the named labels; preserves mask/object positional alignment.
scan_selected = BtNode_ScanForYolo26(
    name="ScanCupAndBottle",
    bb_key="selected_objects",
    object="cup . bottle",
    sort_closest=True,
    return_segments=True,
    return_rgb_image=True,
    return_depth_image=True,
)

# Read a string from the blackboard on every visit, including an empty string.
# A preceding node must write target_labels before this node is ticked.
scan_from_blackboard = BtNode_ScanForYolo26(
    name="ScanRequestedLabels",
    bb_source="target_labels",
    bb_key="requested_objects",
    use_orbbec=False,  # RealSense arm camera
)
```

Add the selected node to your normal tree and pass the ROS node during
`tree.setup(node=ros_node)`; keep the ROS executor spinning as usual.

Prompt rules:
- Neither `object` nor `bb_source`: sends `""`, detecting all labels.
- Explicit `object=""` also detects all labels and overrides `bb_source`.
- `object=None` plus `bb_source`: reads that blackboard string each visit.
- Nonempty prompts select exact checkpoint labels, case-insensitively.
  Use `cup . bottle`, commas or newlines for multiple labels; preserve
  spaces within labels such as `dining table`.
- Unknown labels return server failure and therefore BT `FAILURE`.
  The node always sends both VLM flags as false.

## Response and ordering

On `status=0`, the node returns `SUCCESS` and writes the complete response
to `bb_key`: `objects`, `segments`, `header`, `detection_source`,
`rgb_image`, and `depth_image`. Downstream generalist consumers can use it
without changing their response parsing. Pending calls return `RUNNING`;
empty detections or a server error return `FAILURE`. As with generalist,
failure does not overwrite a previous successful blackboard response;
downstream consumers must follow the BT status.

The default is `sort_closest=True` and `return_segments=True`.
Closest sorting uses camera-to-centroid distance and keeps masks aligned.
For height sorting, set `sort_closest=False, sort_highest=True`.
`transform_to_map=True` requests map-frame centroids subject to the same
camera/TF rules as generalist. All camera intake, geometry, distance gates
and ordering run on the server.

The service endpoint can be changed with `service_name="..."`.
Existing `BtNode_ScanForGeneralist` nodes also work if explicitly pointed
at `object_detection_yolo26` and given `object=""` for all labels, but the
dedicated node provides the correct defaults.

## Mock configuration

`behavior_tree/mock_config.json` registers `BtNode_ScanForYolo26` in the
vision subsystem with `KEYPRESS`, matching the generalist template.
With global mock disabled it calls the real service. Under enabled vision
mock, use `KEYPRESS` / `IMMEDIATE` for simulation, or set this node's
entry to `NO_MOCK` to use the real server. Check `BT_MOCK_MODE` and
`BT_MOCK_CONFIG` overrides when configuring a run.

This adds a reusable node; existing task trees retain their selected detectors.

## Announce detected labels in Chinese

Pass a label-to-Chinese dictionary to `BtNode_AnnounceDetections`. It reads
the detection response from the blackboard and calls the existing `announce`
service with the translated sentence.

```python
import py_trees
from behavior_tree.TemplateNodes.Vision import BtNode_ScanForYolo26
from behavior_tree.TemplateNodes.Audio import BtNode_AnnounceDetections

label_map = {
    "cup": "杯子",
    "bottle": "瓶子",
    "person": "人",
    "dining table": "餐桌",
}

root = py_trees.composites.Sequence(
    name="DetectAndAnnounceChinese",
    memory=True,
    children=[
        BtNode_ScanForYolo26(
            name="DetectObjects",
            bb_key="detections",
            object="",
            sort_closest=True,
        ),
        BtNode_AnnounceDetections(
            name="AnnounceObjectsChinese",
            bb_source="detections",
            label_map=label_map,
            bb_text_key="detection_announcement",  # optional generated text
        ),
    ],
)
```

For objects `cup, bottle, cup`, this announces `我看到了杯子、瓶子。`.
The node preserves the detector's order (so closest-first stays closest-first),
matches dictionary keys case-insensitively and skips unmapped labels.
By default, repeated Chinese names are announced once; use
`deduplicate=False` to announce each instance in order.

Optional `prefix` / `suffix` change the sentence. With no mapped labels the
node succeeds silently by default; set `empty_message="没有识别到已知物体。"`
to announce an alternative. Missing/malformed detection data or a nonzero
detection status returns `FAILURE` without calling TTS. The Sequence above
also stops before announcing if detection fails. Generated text is cleared
on each visit to prevent reusing a previous sentence.

It uses the same asynchronous completion and mock mechanism as
`BtNode_Announce`: `RUNNING` until TTS completes, `SUCCESS` for TTS status 0,
otherwise `FAILURE`. `BtNode_AnnounceDetections` is registered in the
`announcement` mock subsystem with `KEYPRESS`. The default endpoint is
`announce`; override `service_name` for another TextToSpeech endpoint.


