"""
================================================================================
COMPONENT: Main Controller Node (Master Logic & FSM)
================================================================================
1. STATE MACHINE FLOW (FSM)
--------------------------------------------------------------------------------
[0] INIT: Wait for hardware initialization -> [1] SEARCH.
[1] SEARCH: Chassis explores. Vision publishes 'camera_link' coords; master transforms
            and stores both 'map' coords (for Nav) and 'camera_link' coords (for Arm).
            If an object-box pair (same color) exists (both have map_pose) -> [2] MOVE_TO_OBJECT.
[2] MOVE_TO_OBJECT: Master sends object's 'map' pose to Nav -> wait -> [3] GRASP.
[3] GRASP: Master sends object's current 'camera_link' pose to Arm (no timer used).
           Success criterion (event-driven, no timer): while in GRASP, the master
           monitors incoming vision messages; if the current color OBJECT stops
           receiving coordinate updates for several consecutive vision messages,
           treat as SUCCESS -> [4] MOVE_TO_BOX. Otherwise keep waiting (no timeouts).
[4] MOVE_TO_BOX: Master sends box 'map' pose to Nav -> wait -> [5] DROP.
[5] DROP: Master sends box's current 'camera_link' pose to Arm (no timer used).
          Success criterion (event-driven, no timer): while in DROP, if the current
          color OBJECT continues to receive no new updates for several consecutive
          vision messages, treat as SUCCESS; cycle counter++. If < 3 -> [1] SEARCH,
          else -> Shutdown.

Notes:
- GRASP/DROP success is inferred from the ABSENCE of new vision updates for the
  current color's OBJECT across multiple consecutive vision messages (no wall-clock timers).
- This controller maintains both map-space (for navigation) and camera_link-space (for arm)
  data per color. cam_pose is always updated; map_pose is updated when TF is available.
- SEARCH state only toggles an "explore" command channel; an external explorer should react
  to /nav/cmd_explore. If none exists, add a random-goal publisher in SEARCH (not included here).
--------------------------------------------------------------------------------

2. OUTPUT CONTROL MATRIX (Pure Boolean)
--------------------------------------------------------------------------------
| STATE            | NAV_EXPLORE | NAV_GOTO | ARM_GRASP | ARM_DROP |
|------------------|-------------|----------|-----------|----------|
| [0] INIT         | False       | False    | False     | False    |
| [1] SEARCH       | True        | False    | False     | False    |
| [2] MOVE_OBJECT  | False       | True     | False     | False    |
| [3] GRASP        | False       | False    | True      | False    |
| [4] MOVE_BOX     | False       | True     | False     | False    |
| [5] DROP         | False       | False    | False     | True     |
================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped

from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs

# Custom interfaces
from my_robot_interfaces.msg import ObjectTarget, GripperState, CamArmPose

# ==============================================================================
# STATE MACHINE CONSTANTS
# ==============================================================================
STATE_INIT           = 0
STATE_SEARCH         = 1
STATE_MOVE_TO_OBJECT = 2
STATE_GRASP          = 3
STATE_MOVE_TO_BOX    = 4
STATE_DROP           = 5

class RobotControlNode(Node):
    def __init__(self):
        super().__init__(
            'robot_control_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, False)]
        )

        self.current_state = STATE_INIT
        self.cycle_count = 0
        self.max_cycles = 3

        # Per-color storage:
        # map_hash = {
        #   color: {
        #       'object': {'map_pose': PointStamped | None, 'cam_pose': {x,y,z,timestamp} | None, 'timestamp': rclpy.time | None},
        #       'box':    {...}
        #   }
        # }
        self.map_hash = {}
        self.active_target_color = "NONE"

        # --- Event-driven grasp/drop success (no timers):
        self.vision_msg_tick = 0        # increments on each vision message
        self.last_obj_update_tick = -1  # last tick when active color OBJECT was observed
        self.grasp_miss_threshold = 5   # consecutive messages without target-object update to accept as success
        self.drop_miss_threshold = 5    # same as above for DROP
        self.grasp_in_progress = False
        self.drop_in_progress = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.pub_nav_point   = self.create_publisher(PointStamped, '/nav/goal_point', 10)
        self.pub_nav_explore = self.create_publisher(Bool, '/nav/cmd_explore', 10)
        self.pub_arm_pose    = self.create_publisher(CamArmPose, '/arm/grasp_pose', 10)
        self.pub_arm_grip    = self.create_publisher(GripperState, '/arm/grasp_status', 10)
        self.pub_arm_init    = self.create_publisher(GripperState, '/arm/initial_position', 10)



        # Subscribers
        self.sub_vision_target = self.create_subscription(ObjectTarget, '/detected_object', self.vision_target_callback, 10)
        self.sub_move_fb       = self.create_subscription(Bool, '/nav/goal_reached', self.move_feedback_callback, 10)

        # Output Control Matrix (nav_explore used; others are internal semantics)
        self.control_matrix = {
            STATE_INIT:           {"nav_explore": False, "nav_goto": False, "arm_grasp": False, "arm_drop": False},
            STATE_SEARCH:         {"nav_explore": True,  "nav_goto": False, "arm_grasp": False, "arm_drop": False},
            STATE_MOVE_TO_OBJECT: {"nav_explore": False, "nav_goto": True,  "arm_grasp": False, "arm_drop": False},
            STATE_GRASP:          {"nav_explore": False, "nav_goto": False, "arm_grasp": True,  "arm_drop": False},
            STATE_MOVE_TO_BOX:    {"nav_explore": False, "nav_goto": True,  "arm_grasp": False, "arm_drop": False},
            STATE_DROP:           {"nav_explore": False, "nav_goto": False, "arm_grasp": False, "arm_drop": True}
        }

        # INIT readiness checker
        self.init_timer = self.create_timer(1.0, self.check_hardware_readiness)

        # Control loop (no timer-based verification for grasp/drop)
        self.control_loop_timer = self.create_timer(0.1, self.execute_control_loop)

        self.get_logger().info('Master Control Node Initialized. Entering INIT state.')

    # ==========================================================================
    # CORE: TIMER-DRIVEN LOGIC (SEARCH/NAV intents)
    # ==========================================================================
    def execute_control_loop(self):
        # --- [1] SEARCH: find a complete color pair (object + box) with valid map_pose ---
        if self.current_state == STATE_SEARCH:
            for color, items in self.map_hash.items():
                obj = items.get('object') or {}
                box = items.get('box') or {}
                if obj.get('map_pose') is not None and box.get('map_pose') is not None:
                    self.active_target_color = color
                    self.transition_to_state(STATE_MOVE_TO_OBJECT, f"Pair found: {color.upper()}")
                    break

        # --- [2] & [4] NAV target dispatch (publish map coordinates) ---
        elif self.current_state in [STATE_MOVE_TO_OBJECT, STATE_MOVE_TO_BOX]:
            target_type = 'object' if self.current_state == STATE_MOVE_TO_OBJECT else 'box'
            entry = (self.map_hash.get(self.active_target_color, {}) or {}).get(target_type)
            if entry is not None and entry.get('map_pose') is not None:
                # (7) Refresh timestamp to "now" before publishing
                mp = entry['map_pose']
                mp.header.stamp = self.get_clock().now().to_msg()
                self.pub_nav_point.publish(mp)

        # --- [3] GRASP & [5] DROP: success/failure evaluated in vision_target_callback ---
        elif self.current_state == STATE_GRASP:
            pass
        elif self.current_state == STATE_DROP:
            pass

    # ==========================================================================
    # CORE: STATE TRANSITIONS & HARDWARE TRIGGERS
    # ==========================================================================
    def transition_to_state(self, new_state, log_msg=""):
        self.current_state = new_state

        if log_msg:
            self.get_logger().info(f"--- STATE: {new_state} | {log_msg} ---")

        matrix = self.control_matrix[self.current_state]
        self.pub_nav_explore.publish(Bool(data=matrix["nav_explore"]))

        if new_state == STATE_GRASP:
            self.grasp_in_progress = True
            self.drop_in_progress = False
            # Dispatch arm command, then set miss-count baseline immediately (2)
            self.trigger_arm_action(is_grasp=True)
            self.last_obj_update_tick = self.vision_msg_tick

        elif new_state == STATE_DROP:
            self.grasp_in_progress = False
            self.drop_in_progress = True
            # Dispatch arm command, then set miss-count baseline immediately (2)
            self.trigger_arm_action(is_grasp=False)
            self.last_obj_update_tick = self.vision_msg_tick

        elif new_state in [STATE_MOVE_TO_OBJECT, STATE_MOVE_TO_BOX, STATE_SEARCH]:
            self.grasp_in_progress = False
            self.drop_in_progress = False

    # ==========================================================================
    # LOGIC: VISION DATA PUMP, TF, & EVENT-DRIVEN GRASP/DROP VERIFICATION
    # ==========================================================================
    def vision_target_callback(self, msg: ObjectTarget):
        """
        Store both local camera_link coordinates (for Arm) and transformed map coordinates (for Nav).
        Event-driven success checks for GRASP/DROP rely on counting consecutive messages *without*
        updates to the active color's OBJECT (no timers).
        """
        # Advance the global tick for each incoming vision message
        self.vision_msg_tick += 1

        color, item_type = msg.color, msg.name

        if color != "unknown":
            # Ensure per-color dict exists
            if color not in self.map_hash:
                self.map_hash[color] = {'object': {}, 'box': {}}
            # Ensure per-type dict exists (note: might be names beyond 'object'/'box' if upstream publishes them)
            if item_type not in self.map_hash[color]:
                self.map_hash[color][item_type] = {}

            slot = self.map_hash[color][item_type]

            # (1) Always store raw camera_link data (independent of TF)
            slot['cam_pose'] = {
                'x': msg.x,
                'y': msg.y,
                'z': msg.z,
                'timestamp': self.get_clock().now()
            }

            # (1) Transform to map and store opportunistically
            try:
                pt = PointStamped()
                pt.header.frame_id = 'camera_link'
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.point.x, pt.point.y, pt.point.z = msg.x, msg.y, msg.z

                # No timeout version; catch exceptions if TF is not ready
                t = self.tf_buffer.lookup_transform('map', 'camera_link', rclpy.time.Time())
                map_pt = tf2_geometry_msgs.do_transform_point(pt, t)

                slot['map_pose'] = map_pt
                slot['timestamp'] = self.get_clock().now()
            except TransformException:
                # TF may be unavailable momentarily; skip map update for this message.
                pass

            # Track object visibility for the ACTIVE color only
            if (color == self.active_target_color) and (item_type == 'object'):
                self.last_obj_update_tick = self.vision_msg_tick

        # ===== Event-driven verification for GRASP (no timers) =====
        if self.current_state == STATE_GRASP and self.grasp_in_progress:
            if self.last_obj_update_tick >= 0:
                miss_count = self.vision_msg_tick - self.last_obj_update_tick
                if miss_count >= self.grasp_miss_threshold:
                    self.get_logger().info(
                        f"GRASP success inferred (no OBJECT updates for color '{self.active_target_color}' "
                        f"in {miss_count} consecutive vision messages)."
                    )
                    self.grasp_in_progress = False
                    self.transition_to_state(STATE_MOVE_TO_BOX, "Heading to Target Box.")

        # ===== Event-driven verification for DROP (no timers, per (6) still using miss-threshold) =====
        if self.current_state == STATE_DROP and self.drop_in_progress:
            if self.last_obj_update_tick >= 0:
                miss_count = self.vision_msg_tick - self.last_obj_update_tick
                if miss_count >= self.drop_miss_threshold:
                    self.get_logger().info(
                        f"DROP success inferred (no OBJECT updates for color '{self.active_target_color}' "
                        f"in {miss_count} consecutive vision messages)."
                    )
                    self.drop_in_progress = False

                    # Complete cycle & cleanup
                    self.cycle_count += 1
                    completed_color = self.active_target_color

                    if completed_color in self.map_hash:
                        del self.map_hash[completed_color]

                    self.active_target_color = "NONE"

                    if self.cycle_count >= self.max_cycles:
                        self.get_logger().info('ALL 3 CYCLES COMPLETED. MISSION ACCOMPLISHED.')
                        # Keeping original shutdown style as 8) was not requested
                        raise SystemExit
                    else:
                        self.transition_to_state(STATE_SEARCH, f"Cycle {self.cycle_count} done! Resuming Search.")

    # ==========================================================================
    # LOGIC: COMMAND DISPATCH
    # ==========================================================================
    # def trigger_arm_action(self, is_grasp: bool):
    #     """
    #     Dispatch camera_link coordinates to the arm (position only).
    #     NOTE: The CamArmPose message here carries position only. If your arm requires orientation,
    #           extend the message or set defaults in the arm node accordingly.
    #     """
    #     target_type = 'object' if is_grasp else 'box'

    #     if target_type == 'object':
    #         grip_msg = GripperState()
    #         grip_msg.grip = True   #open - True
    #         self.pub_arm_grip.publish(grip_msg)
    #         #wait 5 sec

        
    #     color_dict = self.map_hash.get(self.active_target_color, {})
    #     entry = color_dict.get(target_type)
    #     if entry and entry.get('cam_pose'):
    #         local_pos = entry['cam_pose']
    #         pose_msg = CamArmPose()
    #         pose_msg.x = float(local_pos['x'])
    #         pose_msg.y = float(local_pos['y'])
    #         pose_msg.z = float(local_pos['z'])
    #         self.get_logger().info(
    #             f"Dispatching camera_link pose to Arm: "
    #             f"x={pose_msg.x:.3f}, y={pose_msg.y:.3f}, z={pose_msg.z:.3f}"
    #         )
    #         self.pub_arm_pose.publish(pose_msg)

    #     # Dispatch Gripper State
    #     if target_type == 'object':
    #         grip_msg = GripperState()
    #         grip_msg.grip = False   #open - True
    #         self.pub_arm_grip.publish(grip_msg)

    #         #wait
    #         init_pose = True
    #         self.pub_arm_init.publish(True)
    #     else:
    #         grip_msg = GripperState()
    #         grip_msg.grip = True
    #         self.pub_arm_grip.publish(grip_msg)

    #         #wait 5 sec
    #         init_pose = True
    #         self.pub_arm_init.publish(True)

    def trigger_arm_action(self, is_grasp: bool):
        target_type = 'object' if is_grasp else 'box'

        if target_type == 'object':
            grip_msg = GripperState()
            grip_msg.grip = True  # open gripper
            self.pub_arm_grip.publish(grip_msg)

        # Publish arm pose immediately
        color_dict = self.map_hash.get(self.active_target_color, {})
        entry = color_dict.get(target_type)
        if entry and entry.get('cam_pose'):
            local_pos = entry['cam_pose']
            pose_msg = CamArmPose()
            pose_msg.x = float(local_pos['x'])
            pose_msg.y = float(local_pos['y'])
            pose_msg.z = float(local_pos['z'])
            self.get_logger().info(
                f"Dispatching camera_link pose to Arm: "
                f"x={pose_msg.x:.3f}, y={pose_msg.y:.3f}, z={pose_msg.z:.3f}"
            )
            self.pub_arm_pose.publish(pose_msg)

        # Schedule non-blocking wait before final gripper/init dispatch
        self.get_logger().info("Scheduling follow-up gripper/init after 5 seconds")
        self.create_timer(10.0, lambda: self._after_gripper_wait(is_grasp), once=True)
    
    def _after_gripper_wait(self, is_grasp: bool):
        """Called after non-blocking wait to continue arm action."""
        target_type = 'object' if is_grasp else 'box'

        # Dispatch Gripper State
        grip_msg = GripperState()
        if target_type == 'object':
            grip_msg.grip = False  # close gripper
        else:
            grip_msg.grip = True   # leave open or default
        self.pub_arm_grip.publish(grip_msg)

        # Send init pose
        self.pub_arm_init.publish(True)

        

    # ==========================================================================
    # LOGIC: NAVIGATION FEEDBACK
    # ==========================================================================
    def move_feedback_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.current_state == STATE_MOVE_TO_OBJECT:
            self.transition_to_state(STATE_GRASP, "Arrived at Object. Initiating Grasp Sequence.")
            # (2) Baseline is now set inside transition_to_state

        elif self.current_state == STATE_MOVE_TO_BOX:
            self.transition_to_state(STATE_DROP, "Arrived at Box. Initiating Drop Sequence.")
            # (2) Baseline is now set inside transition_to_state

    # ==========================================================================
    # AUXILIARY: HARDWARE CHECK
    # ==========================================================================
    def check_hardware_readiness(self):
        """
        Verify minimal I/O graph readiness:
          - Vision publishes /detected_object
          - Nav subscribes to /nav/goal_point
          - Arm subscribes to /arm/grasp_pose
        """
        if self.current_state != STATE_INIT:
            self.init_timer.cancel()
            return

        camera_ready = self.count_publishers('/detected_object') > 0
        # Use count_subscribers for topics we publish
        nav_ready    = self.count_subscribers('/nav/goal_point') > 0
        arm_ready    = self.count_subscribers('/arm/grasp_pose') > 0

        if camera_ready and nav_ready and arm_ready:
            self.get_logger().info('>>> ALL SYSTEMS GO! Hardware verified. <<<')
            self.init_timer.cancel()
            self.transition_to_state(STATE_SEARCH, "Commencing Global Patrol.")

def main():
    rclpy.init()
    node = RobotControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()