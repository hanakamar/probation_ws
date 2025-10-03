#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import BoundingBoxArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from mavros_msgs.srv import SetMode, CommandBool

class TestAUVMotion(Node):
    def __init__(self):
        super().__init__('test_auv_motion')

        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        # Subscribers
        self.sub_boxes = self.create_subscription(BoundingBoxArray, '/main_camera/detection/bounding_boxes', self.vision_callback, 10)

        # Service clients for arming and mode change
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # State
        # State
        self.current_gate = None
        self.detected = False
        self.flag = 0
        self.armed = False
        self.guided = False

        # Timers
        self.init_timer = self.create_timer(0.5, self.initialize_vehicle)
        self.vel_timer = self.create_timer(0.1, self.publish_velocity)   # 10 Hz
        self.detected_timer = self.create_timer(0.1, self.detected_objects)   # 10 Hz
        self.down_timer = self.create_timer(0.1, self.move_down)     # 10 Hz
        self.move_timer = self.create_timer(0.1, self.move_straight)     # 10 Hz

        self.get_logger().info("TestAUVMotion node started. Will arm and switch to GUIDED mode first...")

    # --- Initialize vehicle: arm and switch to GUIDED mode ---
    def initialize_vehicle(self):
        # Arm the vehicle
        if not self.armed:
            if self.arm_client.service_is_ready():
                req = CommandBool.Request()
                req.value = True
                future = self.arm_client.call_async(req)
                future.add_done_callback(self.arm_response)
                self.get_logger().info('Sending arming request...')
            return

        # Switch to GUIDED mode
        if not self.guided:
            if self.set_mode_client.service_is_ready():
                req = SetMode.Request()
                req.custom_mode = 'GUIDED'
                future = self.set_mode_client.call_async(req)
                future.add_done_callback(self.mode_response)
                self.get_logger().info('Sending GUIDED mode request...')
            return

        # Vehicle ready, cancel initialization timer
        self.init_timer.cancel()
        self.get_logger().info('Vehicle armed and in GUIDED mode. Ready for motion commands.')

    # --- Callbacks ---
    def arm_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.armed = True
                self.get_logger().info('Vehicle armed successfully.')
            else:
                self.get_logger().error('Failed to arm vehicle.')
        except Exception as e:
            self.get_logger().error(f'Arming service call failed: {e}')

    def mode_response(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.guided = True
                self.get_logger().info('Vehicle set to GUIDED mode successfully.')
            else:
                self.get_logger().error('Failed to set GUIDED mode.')
        except Exception as e:
            self.get_logger().error(f'Set mode service call failed: {e}')

    # --- Vision callback ---
    def vision_callback(self, msg):
        self.detected = False
        self.current_gate = None
        if hasattr(msg, 'bounding_boxes') and msg.bounding_boxes:
            for box in msg.bounding_boxes:
                if hasattr(box, 'label_name') and 'gate' in box.label_name.lower():
                    self.current_gate = box
                    self.detected = True
                    self.last_gate_seen = box
                    self.last_seen_time = self.get_clock().now()
                    self.get_logger().info(
                        f'Gate detected: x={box.x:.3f}, y={box.y:.3f}, w={box.w:.3f}, h={box.h:.3f}'
                    )
                    break
    def detected_objects(self):
        if (self.flag ==1 and self.detected == True):
            if not (self.current_gate.x < 0.55 and self.current_gate.x > 0.45):
                if (self.current_gate.x > 0.55):
                    self.get_logger().info(f"Gate is at {self.current_gate.x:.3f} ")
                    twist = Twist()
                    twist.linear.x = 0.0  # forward
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0  # stay at current depth
                    twist.angular.z = -0.1
                    self.vel_pub.publish(twist)
                    self.get_logger().info("Rotating to right horizontally align with gate")
                    return
                else:
                    self.get_logger().info(f"Gate is at {self.current_gate.x:.3f} ")
                    twist = Twist()
                    twist.linear.x = 0.0  # forward
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0  # stay at current depth
                    twist.angular.z = 0.1
                    self.vel_pub.publish(twist)
                    self.get_logger().info("Rotating to left horizontally align with gate")
                    return
            if (self.current_gate.x < 0.55 and self.current_gate.x > 0.45):
                self.get_logger().info("Reached right horizontal position")
                self.flag = 2
                self.vel_timer.cancel()
                return
    def move_down(self):
        if (self.flag == 2 and self.detected == True):
            if not (self.current_gate.y < 0.55 and self.current_gate.y > 0.45):
                self.get_logger().info(f"Gate is at {self.current_gate.y:.3f} ")
                twist = Twist()
                twist.linear.x = 0.0  # forward
                twist.linear.y = 0.0
                twist.linear.z = - 0.5  # move down
                twist.angular.z = 0.0
                self.vel_pub.publish(twist)
                self.get_logger().info("Moving down to align with gate")
                return
            if (self.current_gate.y < 0.55 and self.current_gate.y > 0.45):
                self.get_logger().info("Reached right vertical position")
                self.flag = 3
                self.down_timer.cancel()
                return
            
    def move_straight(self):
        if (self.flag == 3 and self.detected == True):
            if not (self.detected):
                self.get_logger().info("Completed gate alignment and moving forward")
                self.move_timer.cancel()
                return
            twist = Twist()
            twist.linear.x = 0.5  # forward
            twist.linear.y = 0.0
            twist.linear.z = 0.0  # stay at current depth
            twist.angular.z = 0.0
            self.vel_pub.publish(twist)
            self.get_logger().info("Moving straight towards gate")
            return
    # --- Motion commands ---
    def publish_velocity(self):
        if not (self.armed and self.guided):
            return  # don't move until armed and in GUIDED mode
        if (self.detected and self.current_gate):
            twist = Twist()
            twist.linear.x = 0.  # forward
            twist.linear.y = 0.0
            twist.linear.z = 0.0  # stay at current depth
            twist.angular.z = 0.0
            self.vel_pub.publish(twist)
            self.get_logger().info("Noticed gate")
            self.flag = 1
            self.vel_timer.cancel()
            return  # don't move if a gate is detected
        
        
        if not (self.detected and self.current_gate and self.flag == 1):
            twist = Twist()
            twist.linear.x = 0.0  # forward
            twist.linear.y = 0.0
            twist.linear.z = 0.0  # stay at current depth
            twist.angular.z = 0.5
            self.vel_pub.publish(twist)
            self.get_logger().info("Publishing velocity command (forward)")

def main(args=None):
    rclpy.init(args=args)
    node = TestAUVMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    #cancel timer for moving rotation once detected
    #move front towards gate??
