"""
================================================================================
COMPONENT: Main Controller Node (Master Logic & FSM)
================================================================================
1. STATE MACHINE FLOW (FSM)
--------------------------------------------------------------------------------
[0] INIT: Probes the hardware graph. Checks if required node topics (Vision, Nav, 
          Arm) have active publishers/subscribers. Once ready -> [1] SEARCH.

[1] SEARCH: Master sends explore signal to Nav (/nav/cmd_explore). Vision publishes 
            'camera_link' coords (/detected_object). Master stores local coords, 
            calculates 'map' coords, and tracks pair status. 
            Pair Status Flags: [notfound, paired, deferred, solved].
            Transition: Prioritizes 'paired' colors. If all others are 'solved', 
            it processes 'deferred'. Once target locked -> [2] MOVE_TO_OBJECT.

[2] MOVE_TO_OBJECT: Master sends object's 'map' pose to Nav (/nav/goal_point). 
                    Vision continuously updates coordinates. 
                    Wait for Nav success (/nav/goal_reached) -> [3] GRASP.

[3] GRASP: Master publishes latest 'camera_link' object coords to Arm (/arm/grasp_pose)
           and close signal to gripper (/arm/grasp_status). After a short delay, 
           Master sends reset command (/arm/initial_position).
           Verification: Master checks vision updates. 
             - If object is absent -> SUCCESS -> [4] MOVE_TO_BOX.
             - If object still present -> Publish latest coords and open gripper to retry.
             - If 5 consecutive attempts fail -> Mark as 'deferred' -> [1] SEARCH.

[4] MOVE_TO_BOX: Master sends box 'map' pose to Nav (/nav/goal_point). Vision 
                 continuously updates coordinates. 
                 Wait for Nav success (/nav/goal_reached) -> [5] DROP.

[5] DROP: Master publishes box's 'camera_link' pose to Arm (/arm/grasp_pose) to 
          place the object. After a short delay, Master commands arm to reset 
          (/arm/initial_position) and closes the gripper. 
          Marks current pair as 'solved'. Transition -> [1] SEARCH.

--------------------------------------------------------------------------------
2. PUB/SUB INTERFACE REFERENCE
--------------------------------------------------------------------------------
Publishers (Controller -> Actuators):
  - /nav/goal_point       (PointStamped) : Target map coordinates for Nav
  - /nav/cmd_explore      (Bool)         : Enable/Disable random exploration
  - /nav/target_info      (String)       : Send current task (color + name) to Nav ONCE
  - /arm/grasp_pose       (CamArmPose)   : Target camera_link coordinates for Arm
  - /arm/grasp_status     (GripperState) : Control gripper (close/open)
  - /arm/initial_position (GripperState) : Command arm to return to safe/initial pose

Subscribers (Sensors -> Controller):
  - /detected_object      (ObjectTarget) : 3D coordinates from Vision
  - /nav/goal_reached     (Bool)         : Success feedback from Nav
--------------------------------------------------------------------------------
3. OUTPUT CONTROL MATRIX (Pure Boolean)
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
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PointStamped
import copy

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
        """
        Initializes the node, defines FSM variables, sets up ROS 2 publishers/subscribers, 
        and initializes transform listeners and control loop timers.
        """
        super().__init__(
            'robot_control_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, False)]
        )

        self.current_state = STATE_INIT
        self.cycle_count = 0
        self.max_cycles = 3

        self.map_hash = {}
        self.active_target_color = "NONE"

        self.vision_msg_tick = 0        
        self.last_obj_update_tick = -1  
        self.grasp_miss_threshold = 5   
        self.drop_miss_threshold = 5    

        self.grasp_attempts = 0         
        self.arm_action_done = False    
        self.check_start_tick = 0       
        self.arm_timer = None           

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.pub_nav_point       = self.create_publisher(PointStamped, '/nav/goal_point', 10)
        self.pub_nav_explore     = self.create_publisher(Bool, '/nav/cmd_explore', 10)
        self.pub_nav_target_info = self.create_publisher(String, '/nav/target_info', 10) 
        self.pub_arm_pose        = self.create_publisher(CamArmPose, '/arm/grasp_pose', 10)
        self.pub_arm_grip        = self.create_publisher(GripperState, '/arm/grasp_status', 10)
        self.pub_arm_init        = self.create_publisher(GripperState, '/arm/initial_position', 10)

        # Subscribers
        self.sub_vision_target = self.create_subscription(ObjectTarget, '/detected_object', self.vision_target_callback, 10)
        self.sub_vision_state  = self.create_subscription(Bool, '/detection_state', self.vision_state_callback, 10)
        self.sub_move_fb       = self.create_subscription(Bool, '/nav/goal_reached', self.move_feedback_callback, 10)

        # Output Control Matrix
        self.control_matrix = {
            STATE_INIT:           {"nav_explore": False, "nav_goto": False, "arm_grasp": False, "arm_drop": False},
            STATE_SEARCH:         {"nav_explore": True,  "nav_goto": False, "arm_grasp": False, "arm_drop": False},
            STATE_MOVE_TO_OBJECT: {"nav_explore": False, "nav_goto": True,  "arm_grasp": False, "arm_drop": False},
            STATE_GRASP:          {"nav_explore": False, "nav_goto": False, "arm_grasp": True,  "arm_drop": False},
            STATE_MOVE_TO_BOX:    {"nav_explore": False, "nav_goto": True,  "arm_grasp": False, "arm_drop": False},
            STATE_DROP:           {"nav_explore": False, "nav_goto": False, "arm_grasp": False, "arm_drop": True}
        }

        # Timers
        self.init_timer = self.create_timer(1.0, self.check_hardware_readiness)
        self.control_loop_timer = self.create_timer(0.1, self.execute_control_loop)
        self.get_logger().info('FSM Control Node Initialized. Entering INIT state. Waiting for hardware check...')

    def execute_control_loop(self):
        """
        High-frequency control loop. Evaluates state machine logic: processes search pairings, 
        publishes navigation targets, and handles event-driven verification for grasp/drop success.
        """
        if self.current_state == STATE_SEARCH:
            target_color = None
            has_paired = False
            all_others_solved = True
            deferred_color = None
            for color, items in self.map_hash.items():
                status = items.get('status', 'notfound')
                if status == 'paired':
                    target_color = color
                    has_paired = True
                    break
                elif status == 'deferred':
                    deferred_color = color
                elif status != 'solved':
                    all_others_solved = False
            if has_paired:
                self.active_target_color = target_color
                self.transition_to_state(STATE_MOVE_TO_OBJECT, f"Pair found: {target_color.upper()}")
            elif deferred_color and all_others_solved:
                self.active_target_color = deferred_color
                self.transition_to_state(STATE_MOVE_TO_OBJECT, f"Processing deferred: {deferred_color.upper()}")

        elif self.current_state == STATE_GRASP:
            if self.arm_action_done:
                miss_count = self.vision_msg_tick - self.last_obj_update_tick
                ticks_since_check = self.vision_msg_tick - self.check_start_tick
                if miss_count >= self.grasp_miss_threshold:
                    self.get_logger().info(f"GRASP SUCCESS. Object absent for {miss_count} ticks.")
                    self.arm_action_done = False
                    self.transition_to_state(STATE_MOVE_TO_BOX, "Heading to Target Box.")
                elif ticks_since_check > self.grasp_miss_threshold * 2:
                    self.grasp_attempts += 1
                    self.arm_action_done = False
                    if self.grasp_attempts >= 5:
                        self.get_logger().info("GRASP FAILED 5 TIMES. Marking 'deferred'.")
                        if self.active_target_color in self.map_hash:
                            self.map_hash[self.active_target_color]['status'] = 'deferred'
                        self.transition_to_state(STATE_SEARCH, "Resuming Search.")
                    else:
                        self.get_logger().info(f"GRASP FAILED. Retrying Attempt {self.grasp_attempts+1}/5.")
                        grip_msg = GripperState()
                        grip_msg.grip = True  
                        self.pub_arm_grip.publish(grip_msg)
                        self.trigger_arm_action(is_grasp=True)

        elif self.current_state == STATE_DROP:
            if self.arm_action_done:
                self.get_logger().info("DROP SUCCESS.")
                self.arm_action_done = False
                if self.active_target_color in self.map_hash:
                    self.map_hash[self.active_target_color]['status'] = 'solved'
                self.cycle_count += 1
                self.active_target_color = "NONE"
                if self.cycle_count >= self.max_cycles:
                    self.get_logger().info('ALL 3 CYCLES COMPLETED. MISSION ACCOMPLISHED.')
                    raise SystemExit
                else:
                    self.transition_to_state(STATE_SEARCH, f"Cycle {self.cycle_count} done! Resuming Search.")

    def transition_to_state(self, new_state, log_msg=""):
        """
        Handles state machine transitions, applies the corresponding output control matrix, 
        and resets tracking variables for the new state.
        """
        self.current_state = new_state
        if log_msg:
            self.get_logger().info(f"--- STATE: {new_state} | {log_msg} ---")
        matrix = self.control_matrix[self.current_state]
        self.pub_nav_explore.publish(Bool(data=matrix["nav_explore"]))

        if new_state in [STATE_MOVE_TO_OBJECT, STATE_MOVE_TO_BOX]:
            target_type = 'object' if new_state == STATE_MOVE_TO_OBJECT else 'box'
            # Publish target info ONCE
            nav_info_msg = String()
            nav_info_msg.data = f"color:{self.active_target_color},name:{target_type}"
            self.pub_nav_target_info.publish(nav_info_msg)
            self.get_logger().info(f"Published task to Nav ONCE: {nav_info_msg.data}")
            # Publish target map pose ONCE
            entry = (self.map_hash.get(self.active_target_color, {}) or {}).get(target_type)
            if entry is not None and entry.get('map_pose') is not None:
                mp = copy.deepcopy(entry['map_pose'])
                mp.header.stamp = self.get_clock().now().to_msg()
                self.pub_nav_point.publish(mp)
                self.get_logger().info(f"Published Nav map_pose target ONCE.")
            else:
                self.get_logger().error(f"CRITICAL: Trying to move to {target_type}, but map_pose is missing! Forcing rollback to SEARCH.")
                self.current_state = STATE_SEARCH
                if self.active_target_color in self.map_hash:
                    self.map_hash[self.active_target_color]['status'] = 'deferred'
                self.pub_nav_explore.publish(Bool(data=True))

        if new_state == STATE_GRASP:
            self.arm_action_done = False
            self.grasp_attempts = 1
            self.trigger_arm_action(is_grasp=True)
            self.last_obj_update_tick = self.vision_msg_tick

        elif new_state == STATE_DROP:
            self.arm_action_done = False
            self.trigger_arm_action(is_grasp=False)
            self.last_obj_update_tick = self.vision_msg_tick

        elif new_state in [STATE_MOVE_TO_OBJECT, STATE_MOVE_TO_BOX, STATE_SEARCH]:
            self.arm_action_done = False

    def vision_state_callback(self, msg: Bool):
        self.vision_msg_tick += 1

    def vision_target_callback(self, msg: ObjectTarget):
        """
        Processes vision detections. Stores raw camera_link poses, calculates global map 
        coordinates via TF, and updates FSM pair statuses.
        """
        color, item_type = msg.color, msg.name
        if color != "unknown":
            if color not in self.map_hash:
                self.map_hash[color] = {'object': {}, 'box': {}, 'status': 'notfound'}
            if item_type not in self.map_hash[color]:
                self.map_hash[color][item_type] = {}
            slot = self.map_hash[color][item_type]
            slot['cam_pose'] = {
                'x': msg.x, 'y': msg.y, 'z': msg.z,
                'timestamp': self.get_clock().now()
            }
            try:
                pt = PointStamped()
                pt.header.frame_id = 'camera_link'
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.point.x, pt.point.y, pt.point.z = msg.x, msg.y, msg.z
                t = self.tf_buffer.lookup_transform('map', 'camera_link', rclpy.time.Time())
                map_pt = tf2_geometry_msgs.do_transform_point(pt, t)
                slot['map_pose'] = map_pt
                slot['timestamp'] = self.get_clock().now()
            except TransformException as e:
                # FIXED: Silent TF exceptions are now logged as warnings.
                self.get_logger().warn(f"TF Transform failed (camera_link -> map): {e}")
                pass
            obj_data = self.map_hash[color].get('object', {})
            box_data = self.map_hash[color].get('box', {})
            if obj_data.get('map_pose') is not None and box_data.get('map_pose') is not None:
                if self.map_hash[color].get('status', 'notfound') == 'notfound':
                    self.map_hash[color]['status'] = 'paired'
            if (color == self.active_target_color) and (item_type == 'object'):
                self.last_obj_update_tick = self.vision_msg_tick

    def trigger_arm_action(self, is_grasp: bool):
        """
        Initiates the asynchronous arm sequence (Grasp or Drop). Configures initial 
        gripper state, publishes the target pose, and schedules the non-blocking wait.
        """
        target_type = 'object' if is_grasp else 'box'
        if is_grasp:
            grip_msg = GripperState()
            grip_msg.grip = True
            self.pub_arm_grip.publish(grip_msg)
            self.get_logger().info("GRASP sequence started: Gripper opened.")
        color_dict = self.map_hash.get(self.active_target_color, {})
        entry = color_dict.get(target_type)
        if entry and entry.get('cam_pose'):
            local_pos = entry['cam_pose']
            pose_msg = CamArmPose()
            pose_msg.x = float(local_pos['x'])
            pose_msg.y = float(local_pos['y'])
            pose_msg.z = float(local_pos['z'])
            action_name = "GRASP" if is_grasp else "DROP"
            self.get_logger().info(
                f"[{action_name}] Dispatching camera_link pose to Arm: "
                f"x={pose_msg.x:.3f}, y={pose_msg.y:.3f}, z={pose_msg.z:.3f}"
            )
            self.pub_arm_pose.publish(pose_msg)
        self.get_logger().info(f"Waiting 10 seconds for arm to move to the {target_type}...")
        if self.arm_timer:
            self.arm_timer.cancel()
        self.arm_timer = self.create_timer(10.0, lambda: self._after_gripper_wait(is_grasp))
    
    def _after_gripper_wait(self, is_grasp: bool):
        """
        Callback executed after the main physical movement delay. Finalizes the grasp 
        action or initiates the drop release and safe retreat sequence.
        """
        if self.arm_timer:
            self.arm_timer.cancel()
            self.arm_timer = None
        if is_grasp:
            self.get_logger().info("10s passed. [GRASP] Closing gripper and returning to init pose.")
            grip_msg = GripperState()
            grip_msg.grip = False
            self.pub_arm_grip.publish(grip_msg)
            init_msg = GripperState()
            init_msg.grip = True 
            self.pub_arm_init.publish(init_msg)
            self.arm_action_done = True
            self.check_start_tick = self.vision_msg_tick
        else:
            self.get_logger().info("10s passed. [DROP] Opening gripper and returning to init pose. Waiting 3s to close.")
            grip_msg_open = GripperState()
            grip_msg_open.grip = True
            self.pub_arm_grip.publish(grip_msg_open)
            init_msg = GripperState()
            init_msg.grip = True 
            self.pub_arm_init.publish(init_msg)
            self.arm_timer = self.create_timer(3.0, self._final_drop_cleanup)

    def _final_drop_cleanup(self):
        """
        Final callback in the drop sequence to safely close the gripper after the 
        arm has released the object and retreated to its initial pose.
        """
        if self.arm_timer:
            self.arm_timer.cancel()
            self.arm_timer = None
        self.get_logger().info("3s passed. [DROP] Closing gripper to complete the cycle.")
        grip_msg_close = GripperState()
        grip_msg_close.grip = False
        self.pub_arm_grip.publish(grip_msg_close)
        self.arm_action_done = True
        self.check_start_tick = self.vision_msg_tick

    def move_feedback_callback(self, msg: Bool):
        """
        Listens for navigation success feedback to trigger FSM transitions 
        towards the next manipulation state (GRASP or DROP).
        """
        # FIXED: Prevent infinite deadlock if navigation fails
        if not msg.data:
            self.get_logger().warn("Navigation failed (msg.data is False)! Marking target as deferred.")
            if self.active_target_color in self.map_hash:
                self.map_hash[self.active_target_color]['status'] = 'deferred'
            self.transition_to_state(STATE_SEARCH, "Nav failure. Resuming Search to avoid deadlock.")
            return
        if self.current_state == STATE_MOVE_TO_OBJECT:
            self.transition_to_state(STATE_GRASP, "Arrived at Object. Initiating Grasp Sequence.")
        elif self.current_state == STATE_MOVE_TO_BOX:
            self.transition_to_state(STATE_DROP, "Arrived at Box. Initiating Drop Sequence.")

    def check_hardware_readiness(self):
        """
        Validates that necessary sensors and actuators are online by probing the ROS 2 graph 
        before commencing the FSM logic.
        """
        if self.current_state != STATE_INIT:
            self.init_timer.cancel()
            return
        camera_ready = self.count_publishers('/detected_object') > 0 
        nav_ready    = self.count_subscribers('/nav/goal_point') > 0
        arm_ready    = self.count_subscribers('/arm/grasp_pose') > 0
        if camera_ready and nav_ready and arm_ready:
            self.get_logger().info('>>> ALL SYSTEMS GO! Hardware verified. <<<')
            self.init_timer.cancel()
            self.transition_to_state(STATE_SEARCH, "Commencing Global Patrol.")

def main():
    """
    Standard ROS 2 entry point. Initializes and spins the RobotControlNode.
    """
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
