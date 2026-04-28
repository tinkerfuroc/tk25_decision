import py_trees
import time
from typing import List, Dict, Any, Optional
from behavior_tree.TemplateNodes.BaseBehaviors import ServiceHandler
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.ActionBase import ActionHandler
import math
from behavior_tree.messages import ObjectDetectionGeneralist, TextToSpeech, Listen, PhraseExtraction, PanTiltCommand
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String

class BtNode_DetectCallingCustomer(ServiceHandler):
    def __init__(self,
                 name: str,
                 bb_dest_key: str,
                 service_name: str = "object_detection_generalist",
                 timeout: float = 10.0):
        super().__init__(name, service_name, ObjectDetectionGeneralist)
        self.bb_dest_key = bb_dest_key
        self.timeout = timeout
        self.start_time = None
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="customer_location",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_dest_key)
        )
    
    def initialise(self):
        self.start_time = time.time()
        request = ObjectDetectionGeneralist.Request()
        request.prompt = "person waving or calling"
        # tk23's flags="detect_gesture" was a no-op in tk26; fall through to VLM+SAM
        # so the open-vocabulary prompt actually has a chance of matching.
        request.use_vlm_sam_fallback = True
        request.camera = "orbbec"
        request.target_frame = "map"
        self.response = self.client.call_async(request)
        self.feedback_message = "Looking for calling or waving customer"
    
    def update(self):
        if time.time() - self.start_time > self.timeout:
            self.feedback_message = "Timeout: No calling customer detected"
            return py_trees.common.Status.FAILURE
            
        if self.response.done():
            result = self.response.result()
            if result.status == 0 and len(result.objects) > 0:
                customer_obj = result.objects[0]
                customer_pose = PoseStamped()
                customer_pose.header = result.header
                customer_pose.pose.position = customer_obj.centroid
                customer_pose.pose.orientation.w = 1.0
                
                self.blackboard.customer_location = customer_pose
                self.feedback_message = f"Detected calling customer at position"
                return py_trees.common.Status.SUCCESS
            else:
                self.initialise()
                return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.RUNNING

class BtNode_TakeOrder(ServiceHandler):
    def __init__(self, 
                 name: str,
                 bb_dest_key: str,
                 service_name: str = "phrase_extraction_service",
                 timeout: float = 15.0):
        super().__init__(name, service_name, PhraseExtraction)
        self.bb_dest_key = bb_dest_key
        self.timeout = timeout
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="customer_order",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_dest_key)
        )
    
    def initialise(self):
        request = PhraseExtraction.Request()
        request.timeout = self.timeout
        request.wordlist = [
            "apple", "banana", "orange", "milk", "coffee", "tea", 
            "water", "juice", "bread", "sandwich", "cake", "cookie"
        ]
        self.response = self.client.call_async(request)
        self.feedback_message = "Listening for customer order"
    
    def update(self):
        if self.response.done():
            result = self.response.result()
            if result.status == 0:
                self.blackboard.customer_order = result.phrase
                self.feedback_message = f"Received order: {result.phrase}"
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = f"Failed to understand order: {result.error_message}"
                return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING

class BtNode_ConfirmOrder(BtNode_Announce):
    def __init__(self, 
                 name: str,
                 bb_order_key: str,
                 service_name: str = "announce"):
        super(BtNode_Announce, self).__init__(name, service_name, TextToSpeech)
        self.bb_order_key = bb_order_key
        self.bb_source = None
        self.given_msg = None
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="order",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_order_key)
        )
    
    def initialise(self):
        order = self.blackboard.order
        self.given_msg = f"I understand your order is {order}. Is this correct?"
        return super().initialise()

class BtNode_CommunicateWithBarman(BtNode_Announce):
    def __init__(self, 
                 name: str,
                 bb_order_key: str,
                 service_name: str = "announce"):
        super(BtNode_Announce, self).__init__(name, service_name, TextToSpeech)
        self.bb_order_key = bb_order_key
        self.bb_source = None
        self.given_msg = None
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="order",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_order_key)
        )
    
    def initialise(self):
        order = self.blackboard.order
        self.given_msg = f"Hello, I need to place an order: {order}. Please prepare these items."
        return super().initialise()

class BtNode_DetectTray(ServiceHandler):
    def __init__(self, 
                 name: str,
                 bb_dest_key: str,
                 service_name: str = "object_detection"):
        super().__init__(name, service_name, ObjectDetection)
        self.bb_dest_key = bb_dest_key
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="tray_location",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_dest_key)
        )
    
    def initialise(self):
        request = ObjectDetection.Request()
        request.prompt = "tray"
        request.flags = "scan"
        request.camera = "orbbec"
        request.target_frame = "map"
        self.response = self.client.call_async(request)
        self.feedback_message = "Looking for available tray"
    
    def update(self):
        if self.response.done():
            result = self.response.result()
            if result.status == 0 and len(result.objects) > 0:
                tray_obj = result.objects[0]
                tray_point = PointStamped()
                tray_point.header = result.header
                tray_point.point = tray_obj.centroid
                
                self.blackboard.tray_location = tray_point
                self.feedback_message = "Found available tray"
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = "No tray found"
                return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING

class BtNode_ServeOrder(BtNode_Announce):
    def __init__(self, 
                 name: str,
                 bb_order_key: str,
                 service_name: str = "announce"):
        super(BtNode_Announce, self).__init__(name, service_name, TextToSpeech)
        self.bb_order_key = bb_order_key
        self.bb_source = None
        self.given_msg = None
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="order",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_order_key)
        )
    
    def initialise(self):
        order = self.blackboard.order
        self.given_msg = f"Here is your order: {order}. Enjoy your meal!"
        return super().initialise()

class BtNode_ScanForCallingCustomer(ServiceHandler):
    def __init__(self, 
                 name: str,
                 bb_dest_key: str,
                 service_name: str = "object_detection",
                 timeout: float = 30.0):
        super().__init__(name, service_name, ObjectDetection)
        self.bb_dest_key = bb_dest_key
        self.timeout = timeout
        self.start_time = None
        self.scan_positions = [
            (-45.0, 30.0),  
            (0.0, 30.0),
            (45.0, 30.0),   
            (-30.0, 45.0), 
            (30.0, 45.0),  
        ]
        self.current_position = 0
        self.position_start_time = None
        self.scan_duration_per_position = 2.0
        self.camera_initialized = False
        
        self.pan_tilt_publisher = None
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="customer_location",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_dest_key)
        )
    
    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.pan_tilt_publisher = self.node.create_publisher(PanTiltCommand, '/pan_tilt_controller/cmd', 10)

    def move_camera_to_position(self, x_angle, y_angle):
        msg = PanTiltCommand()
        msg.mode = PanTiltCommand.ABSOLUTE
        msg.pan_rad = math.radians(float(x_angle))
        msg.tilt_rad = math.radians(float(y_angle))
        msg.speed_raw = 0
        msg.accel_raw = 0
        self.pan_tilt_publisher.publish(msg)
    
    def initialise(self):
        self.start_time = time.time()
        self.current_position = 0
        self.position_start_time = time.time()
        self.camera_initialized = False
        
        # 首先将相机调整到水平位置（Y轴30度对应实际的水平位置）
        self.move_camera_to_position(0.0, 30.0)
        
        # 等待相机移动到水平位置
        time.sleep(2.0)
        
        self.camera_initialized = True
        
        # 然后移动到第一个扫描位置
        x_angle, y_angle = self.scan_positions[self.current_position]
        self.move_camera_to_position(x_angle, y_angle)
        
        # 等待相机移动到第一个扫描位置
        time.sleep(1.0)
        
        self.start_detection()
        self.feedback_message = f"Camera initialized to horizontal, starting scan at position {self.current_position + 1}/{len(self.scan_positions)}"
    
    def start_detection(self):
        request = ObjectDetection.Request()
        request.prompt = "person waving or calling"
        request.flags = "detect_gesture"
        request.camera = "orbbec"
        request.target_frame = "map"
        self.response = self.client.call_async(request)
    
    def update(self):
        if time.time() - self.start_time > self.timeout:
            self.feedback_message = "Timeout: No calling customer found after scanning all positions"
            return py_trees.common.Status.FAILURE
        
        if self.response.done():
            result = self.response.result()
            if result.status == 0 and len(result.objects) > 0:
                customer_obj = result.objects[0]
                customer_pose = PoseStamped()
                customer_pose.header = result.header
                customer_pose.pose.position = customer_obj.centroid
                customer_pose.pose.orientation.w = 1.0
                
                self.blackboard.customer_location = customer_pose
                self.feedback_message = f"Found calling customer at position {self.current_position + 1}"
                return py_trees.common.Status.SUCCESS
            else:
                if time.time() - self.position_start_time > self.scan_duration_per_position:
                    self.current_position += 1
                    
                    if self.current_position >= len(self.scan_positions):
                        self.feedback_message = "No calling customer found in any scan position"
                        return py_trees.common.Status.FAILURE
                    
                    x_angle, y_angle = self.scan_positions[self.current_position]
                    self.move_camera_to_position(x_angle, y_angle)
                    
                    time.sleep(1.0)
                    
                    self.position_start_time = time.time()
                    self.start_detection()
                    self.feedback_message = f"Moving to scan position {self.current_position + 1}/{len(self.scan_positions)}"
                else:
                    self.start_detection()
                    self.feedback_message = f"Scanning position {self.current_position + 1}/{len(self.scan_positions)}..."

                return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.RUNNING


# ---------------------------------------------------------------------------
# Restaurant two-order pipeline helpers (blackboard-only, no ROS I/O).
# ---------------------------------------------------------------------------

def _point_stamped_to_pose_stamped(point_stamped: PointStamped) -> PoseStamped:
    """Convert a PointStamped (from waving-person detection) to a PoseStamped.
    Orientation is identity — nav2 will handle final approach yaw."""
    ps = PoseStamped()
    ps.header = Header(stamp=point_stamped.header.stamp, frame_id=point_stamped.header.frame_id or "map")
    ps.pose = Pose(
        position=Point(x=point_stamped.point.x, y=point_stamped.point.y, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    return ps


class BtNode_SelectNextCustomer(py_trees.behaviour.Behaviour):
    """
    Pop the next unassigned waving customer from `bb_key_all_poses` (list[PointStamped])
    and `bb_key_all_pictures` (list[str]), publish:
      - `bb_key_cur_id`      ← int
      - `bb_key_cur_pose`    ← PoseStamped
      - `bb_key_cur_picture` ← str (empty if unavailable)

    Tracks already-selected ids across ticks so the outer Retry decorator does not
    re-pick the same person.

    FAILURE when no unassigned candidates remain.
    """
    def __init__(self,
                 name: str,
                 bb_key_all_poses: str,
                 bb_key_all_pictures: Optional[str],
                 bb_key_cur_id: str,
                 bb_key_cur_pose: str,
                 bb_key_cur_picture: str,
                 ):
        super().__init__(name)
        self._seen_ids = set()
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(
            key="all_poses",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_all_poses),
        )
        self._has_pictures_src = bb_key_all_pictures is not None
        if self._has_pictures_src:
            self.bb.register_key(
                key="all_pictures",
                access=py_trees.common.Access.READ,
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_all_pictures),
            )
        self.bb.register_key(
            key="cur_id",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_cur_id),
        )
        self.bb.register_key(
            key="cur_pose",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_cur_pose),
        )
        self.bb.register_key(
            key="cur_picture",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_cur_picture),
        )

    def update(self):
        try:
            poses = self.bb.all_poses
        except Exception:
            self.feedback_message = "SelectNextCustomer: no waving-person list on blackboard"
            return py_trees.common.Status.FAILURE
        if not poses:
            self.feedback_message = "SelectNextCustomer: empty waving-person list"
            return py_trees.common.Status.FAILURE

        pictures = []
        if self._has_pictures_src:
            try:
                pictures = self.bb.all_pictures or []
            except Exception:
                pictures = []

        for idx, pt in enumerate(poses):
            if idx in self._seen_ids:
                continue
            self._seen_ids.add(idx)
            self.bb.cur_id = idx
            self.bb.cur_pose = _point_stamped_to_pose_stamped(pt)
            self.bb.cur_picture = pictures[idx] if idx < len(pictures) else ""
            self.feedback_message = f"Selected customer id={idx}"
            return py_trees.common.Status.SUCCESS

        self.feedback_message = "SelectNextCustomer: all candidates exhausted"
        return py_trees.common.Status.FAILURE


class BtNode_RecordOrder(py_trees.behaviour.Behaviour):
    """
    Append `{id, pose, picture_path, items, delivered_items}` to `bb_key_order_list`.
    Reads current-customer fields plus `bb_key_cur_order_items`.
    `items` is stored as a list: if the source is a string it is wrapped in a single-item list.
    """
    def __init__(self,
                 name: str,
                 bb_key_order_list: str,
                 bb_key_cur_id: str,
                 bb_key_cur_pose: str,
                 bb_key_cur_picture: str,
                 bb_key_cur_order_items: str,
                 ):
        super().__init__(name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(
            key="order_list",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_order_list),
        )
        self.bb.register_key(
            key="cur_id",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_cur_id),
        )
        self.bb.register_key(
            key="cur_pose",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_cur_pose),
        )
        self.bb.register_key(
            key="cur_picture",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_cur_picture),
        )
        self.bb.register_key(
            key="cur_order_items",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_cur_order_items),
        )

    def update(self):
        try:
            existing = self.bb.order_list
        except Exception:
            existing = None
        order_list = list(existing) if existing else []

        items_src = self.bb.cur_order_items
        if isinstance(items_src, str):
            items = [items_src] if items_src else []
        elif items_src is None:
            items = []
        else:
            items = list(items_src)

        order = {
            "id": self.bb.cur_id,
            "pose": self.bb.cur_pose,
            "picture_path": self.bb.cur_picture,
            "items": items,
            "delivered_items": [],
        }
        order_list.append(order)
        self.bb.order_list = order_list
        self.feedback_message = f"Recorded order id={order['id']} items={items}"
        return py_trees.common.Status.SUCCESS


class BtNode_FormatOrdersForBarman(py_trees.behaviour.Behaviour):
    """
    Build a human-readable string listing all orders on the blackboard and publish
    it to `bb_key_barman_text`. Example: "Order 1 is water and juice. Order 2 is cola and chips."
    """
    def __init__(self,
                 name: str,
                 bb_key_order_list: str,
                 bb_key_barman_text: str,
                 ):
        super().__init__(name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(
            key="order_list",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_order_list),
        )
        self.bb.register_key(
            key="barman_text",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_barman_text),
        )

    def update(self):
        try:
            orders = self.bb.order_list or []
        except Exception:
            orders = []
        if not orders:
            self.bb.barman_text = "I have no orders to place at this time."
            self.feedback_message = "No orders on blackboard"
            return py_trees.common.Status.SUCCESS
        parts = []
        for i, order in enumerate(orders, start=1):
            items = order.get("items") or []
            if not items:
                continue
            parts.append(f"Order {i} is {' and '.join(items)}.")
        text = "Hello. I would like to place the following orders. " + " ".join(parts)
        self.bb.barman_text = text
        self.feedback_message = text
        return py_trees.common.Status.SUCCESS


class BtNode_IterateOrderItems(py_trees.behaviour.Behaviour):
    """
    Pop the next undelivered item from the order list and publish current-item fields
    to the blackboard. Each invocation advances by one item.

    Writes:
      - `bb_key_cur_item`           ← str
      - `bb_key_cur_order_id`       ← int
      - `bb_key_cur_order_pose`     ← PoseStamped
      - `bb_key_cur_order_picture`  ← str
      - `bb_key_cur_order_summary`  ← str (human-readable "item for order N")

    FAILURE when no more undelivered items remain (parent loop terminates).
    """
    def __init__(self,
                 name: str,
                 bb_key_order_list: str,
                 bb_key_cur_item: str,
                 bb_key_cur_order_id: str,
                 bb_key_cur_order_pose: str,
                 bb_key_cur_order_picture: str,
                 bb_key_cur_order_summary: str,
                 ):
        super().__init__(name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(
            key="order_list",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_order_list),
        )
        for src_key, remap in (
            ("cur_item", bb_key_cur_item),
            ("cur_order_id", bb_key_cur_order_id),
            ("cur_order_pose", bb_key_cur_order_pose),
            ("cur_order_picture", bb_key_cur_order_picture),
            ("cur_order_summary", bb_key_cur_order_summary),
        ):
            self.bb.register_key(
                key=src_key,
                access=py_trees.common.Access.WRITE,
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", remap),
            )

    def update(self):
        try:
            orders = self.bb.order_list or []
        except Exception:
            orders = []
        for order in orders:
            delivered = set(order.get("delivered_items") or [])
            for item in order.get("items") or []:
                if item in delivered:
                    continue
                self.bb.cur_item = item
                self.bb.cur_order_id = order["id"]
                self.bb.cur_order_pose = order["pose"]
                self.bb.cur_order_picture = order.get("picture_path", "")
                self.bb.cur_order_summary = f"{item} for order {order['id']}"
                self.feedback_message = f"Next item: {item} (order {order['id']})"
                return py_trees.common.Status.SUCCESS
        self.feedback_message = "All items delivered"
        return py_trees.common.Status.FAILURE


class BtNode_MarkItemDelivered(py_trees.behaviour.Behaviour):
    """
    Append `bb_key_cur_item` to the matching order's `delivered_items` list.
    Used after a successful Place (or after the show-picture fallback, which we treat
    as delivered-for-tracking so the loop advances).
    """
    def __init__(self,
                 name: str,
                 bb_key_order_list: str,
                 bb_key_cur_order_id: str,
                 bb_key_cur_item: str,
                 ):
        super().__init__(name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(
            key="order_list",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_order_list),
        )
        self.bb.register_key(
            key="cur_order_id",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_cur_order_id),
        )
        self.bb.register_key(
            key="cur_item",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_cur_item),
        )

    def update(self):
        try:
            orders = self.bb.order_list or []
        except Exception:
            orders = []
        order_id = self.bb.cur_order_id
        item = self.bb.cur_item
        for order in orders:
            if order["id"] == order_id:
                delivered = list(order.get("delivered_items") or [])
                if item not in delivered:
                    delivered.append(item)
                order["delivered_items"] = delivered
                self.bb.order_list = orders
                self.feedback_message = f"Marked delivered: {item} for order {order_id}"
                return py_trees.common.Status.SUCCESS
        self.feedback_message = f"MarkItemDelivered: order id={order_id} not found"
        return py_trees.common.Status.FAILURE