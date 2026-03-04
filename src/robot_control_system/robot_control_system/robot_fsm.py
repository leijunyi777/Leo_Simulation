"""
=======================================================================================
Robot Finite State Machine (FSM) Node - Central Controller
=======================================================================================
Design Philosophy:
    This node serves as the central "brain" (FSM/PLC) of the robotic system, 
    adhering strictly to an event-driven and signal-interlock architecture.
    It orchestrates the mission by exchanging Boolean signals with sub-system nodes.
    Coordinate management (saving/popping matching color pairs) is handled by the 
    Vision/TF modules.

System State Machine Flow Diagram: [MODIFIED] for Color-Matching Task
    [0] INIT (Hardware Self-Check)
          | (All Sub-systems Online)
          v
    [1] SEARCH (Global Patrol for Target)
          | (/detected_pair == True) <-- Vision found matching color Object & Box
          v
    [2] MOVE_TO_OBJECT (Navigate to Colored Object)
          | (/move_feedback == True)
          v
    [3] GRASP (Manipulator Pick)
          | (/manipulator_feedback == True)
          v
    [4] MOVE_TO_BOX (Navigate to matching Colored Box)
          | (/move_feedback == True)
          v
    [5] DROP (Manipulator Place)
          | (/manipulator_feedback == True)
          +---> Completed 3 cycles? ---> [Exit Mission]
          | (Else)
          +---> [Loop back to 1 SEARCH]

Output Control Matrix: [MODIFIED]
    | State        | Moving | Arm_Active | Going_to_Box | Description
    |--------------|--------|------------|--------------|-------------------------
    | SEARCH       |  True  |   False    |   False      | Chassis explores, arm stowed.
    | MOVE_TO_OBJ  |  True  |   False    |   False      | Chassis approaches object.
    | GRASP        |  False |   True     |   False      | Chassis halts, arm picks.
    | MOVE_TO_BOX  |  True  |   False    |   True       | Chassis approaches box.
    | DROP         |  False |   True     |   True       | Chassis halts, arm places.
=======================================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import sys

from my_robot_interfaces.msg import ErrorStatus

# === State Machine Constants (Enumerations) ===
STATE_INIT = 0           
STATE_SEARCH = 1         
STATE_MOVE_TO_OBJECT = 2 # [MODIFIED] Renamed from MOVE_TO_TARGET for clarity
STATE_GRASP = 3          
STATE_MOVE_TO_BOX = 4    # [MODIFIED] Renamed from RETURN to represent moving to the colored box
STATE_DROP = 5           

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        self.current_state = STATE_INIT
        self.cycle_count = 0  
        self.max_cycles = 3   # 3 colored objects to process
        # [MODIFIED] Added to store the specific color currently being processed
        self.current_target_color = ""
        self.hardware_ready = {'camera': False, 'nav': False, 'arm': False}
        self.latest_object_pose = None 
        self.latest_box_pose = None

        # === Action Publishers ===
        self.pub_moving = self.create_publisher(Bool, '/status/moving', 10)         
        self.pub_arm = self.create_publisher(Bool, '/status/arm_active', 10)        
        # [MODIFIED] Renamed topic from '/status/returning' to '/status/going_to_box'
        self.pub_to_box = self.create_publisher(Bool, '/status/going_to_box', 10)
        # [MODIFIED] Added Reset signal to tell Vision node to look for the next color
        self.pub_reset_vision = self.create_publisher(Bool, '/status/reset_vision', 10)      

        # === Sensor / Feedback Subscribers ===
        # 1. Vision Node Feedback (Trigger: Search -> Move)
        # [MODIFIED] Updated topic name to match VisionNode output
        self.sub_detect = self.create_subscription(
            Bool, '/detected_pair', self.detect_callback, 10)
        self.sub_object_pose = self.create_subscription(
            PoseStamped, '/target_object_pose', self.object_pose_callback, 10)
        self.sub_box_pose = self.create_subscription(
            PoseStamped, '/target_box_pose', self.box_pose_callback, 10)

        # 2. Navigation Node Feedback 
        self.sub_move_fb = self.create_subscription(
            Bool, '/move_feedback', self.move_feedback_callback, 10)
            
        # 3. Manipulator Node Feedback 
        self.sub_manip_fb = self.create_subscription(
            Bool, '/manipulator_feedback', self.manip_feedback_callback, 10)
            
        # 4. Global Error Monitoring
        self.sub_error = self.create_subscription(
            ErrorStatus, '/status_error', self.error_callback, 10)
            
        self.get_logger().info(f'Node initialized. State: INIT. Cycle: {self.cycle_count}')
        
        self.init_timer = self.create_timer(1.0, self.check_hardware_readiness)

    def check_hardware_readiness(self):
        if self.current_state != STATE_INIT:
            self.init_timer.cancel()
            return
            
        # [MODIFIED] Updated topic name to match new subscription
        if not self.hardware_ready['camera']:
            if self.count_publishers('/detected_pair') > 0:
                self.hardware_ready['camera'] = True
                self.get_logger().info('[Check] Camera Node: ONLINE ✅')
                
        if not self.hardware_ready['nav']:
            if self.count_publishers('/move_feedback') > 0:
                self.hardware_ready['nav'] = True
                self.get_logger().info('[Check] Navigation Node: ONLINE ✅')
                
        if not self.hardware_ready['arm']:
            if self.count_publishers('/manipulator_feedback') > 0:
                self.hardware_ready['arm'] = True
                self.get_logger().info('[Check] Manipulator Node: ONLINE ✅')
                
        if all(self.hardware_ready.values()):
            self.get_logger().info('='*40)
            self.get_logger().info('>>> ALL SYSTEMS GO! Starting Mission... <<<')
            self.get_logger().info('='*40)
            self.current_state = STATE_SEARCH
            
            # [MODIFIED] Updated variable name to going_to_box
            self.publish_state_flags(moving=True, arm=False, going_to_box=False)
            self.init_timer.cancel() 

    # [MODIFIED] Renamed 'returning' argument to 'going_to_box'
    def publish_state_flags(self, moving: bool, arm: bool, going_to_box: bool):
        msg_moving = Bool()
        msg_moving.data = moving
        self.pub_moving.publish(msg_moving)

        msg_arm = Bool()
        msg_arm.data = arm
        self.pub_arm.publish(msg_arm)

        msg_box = Bool()
        msg_box.data = going_to_box
        self.pub_to_box.publish(msg_box)

        self.get_logger().info(f'Status Published -> Moving: {moving}, Arm: {arm}, Going to Box: {going_to_box}')

    def error_callback(self, msg):
        if msg.error_state:
            self.get_logger().fatal(f'CRITICAL ERROR RECEIVED. Error Number: {msg.error_number}')
            self.get_logger().fatal('Terminating Node due to hardware fault...')
            raise SystemExit


    def detect_callback(self, msg):
        if not msg.data:
            return
        if self.current_state == STATE_SEARCH:
            self.get_logger().info('Event: Full Pair (Obj+Box) triggered. Starting Mission.')
            self.current_state = STATE_MOVE_TO_OBJECT
            self.publish_state_flags(moving=True, arm=False, going_to_box=False)


    def object_pose_callback(self, msg):
            """Callback to update the latest object coordinates in FSM memory."""
            self.latest_object_pose = msg
            # Only log debug info during SEARCH to avoid console noise
            if self.current_state == STATE_SEARCH:
                self.get_logger().debug(
                    f"Received objectpose cache: "
                    f"x={msg.pose.position.x:.2f}, "
                    f"y={msg.pose.position.y:.2f}, "
                    f"z={msg.pose.position.z:.2f}"
                    )

    # [MODIFIED] Added Box Pose Callback
    def box_pose_callback(self, msg):
        """Callback to update the latest box coordinates in FSM memory."""
        self.latest_box_pose = msg
        # Only log debug info during SEARCH to avoid console noise
        if self.current_state == STATE_SEARCH:
            self.get_logger().debug(
                    f"Received box pose cache: "
                    f"x={msg.pose.position.x:.2f}, "
                    f"y={msg.pose.position.y:.2f}, "
                    f"z={msg.pose.position.z:.2f}"
                    )


    def move_feedback_callback(self, msg):
        if not msg.data:
            return
        # [MODIFIED] Scenario 1: Arrived at the Object
        if self.current_state == STATE_MOVE_TO_OBJECT:
            self.get_logger().info('Event: Arrived at Object -> Switching to GRASP state')
            self.current_state = STATE_GRASP
            self.publish_state_flags(moving=False, arm=True, going_to_box=False)
        # [MODIFIED] Scenario 2: Arrived at the Colored Box
        elif self.current_state == STATE_MOVE_TO_BOX:
            self.get_logger().info('Event: Arrived at Colored Box -> Switching to DROP state')
            self.current_state = STATE_DROP
            self.publish_state_flags(moving=False, arm=True, going_to_box=True)

    def manip_feedback_callback(self, msg):
        if not msg.data:
            return
        # [MODIFIED] Scenario 1: Object grasped, now move to the corresponding box
        if self.current_state == STATE_GRASP:
            self.get_logger().info('Event: Object Grasped -> Switching to MOVE_TO_BOX state')
            self.current_state = STATE_MOVE_TO_BOX
            # set going_to_box=True so Navigation node knows which coordinate to target
            self.publish_state_flags(moving=True, arm=False, going_to_box=True)
        # Scenario 2: Object dropped in the box
        elif self.current_state == STATE_DROP:
            self.cycle_count += 1
            self.get_logger().info(f'Event: Object Dropped. Cycle count: {self.cycle_count}')
            # [MODIFIED] Send Reset signal to Vision Node after a successful drop
            reset_msg = Bool()
            reset_msg.data = True
            self.pub_reset_vision.publish(reset_msg)
            self.get_logger().info('Sent Reset signal to Vision Node.')
            if self.cycle_count >= self.max_cycles:
                self.get_logger().info('Task Completed (n=3). Shutting down node.')
                raise SystemExit 
            else:
                self.get_logger().info('-> Restarting cycle: Switching to SEARCH state')
                self.current_state = STATE_SEARCH
                self.latest_object_pose = None
                self.latest_box_pose = None
                self.publish_state_flags(moving=True, arm=False, going_to_box=False)


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()