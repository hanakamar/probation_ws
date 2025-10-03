#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import BoundingBoxArray
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Float64
from rclpy.time import Time


class GateNavigator(Node):
    def __init__(self):
        super().__init__('gate_navigator')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Subscribers
        self.sub_boxes = self.create_subscription(
            BoundingBoxArray,
            '/main_camera/detection/bounding_boxes',
            self.vision_callback,
            10
        )
        self.sub_depth = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.depth_callback,
            10
        )

        # Service clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # State
        self.current_gate = None
        self.detected = False
        self.last_gate_seen = None
        self.last_seen_time = None

        self.current_depth = 0.0
        self.target_depth = -2.0
        self.depth_reached = False
        self.mode_changed = False
        self.armed = False

        # Timers
        self.setpoint_stream_count = 0
        self.setpoint_stream_timer = self.create_timer(0.1, self.stream_setpoints)
        self.navigation_timer = self.create_timer(0.1, self.navigation_loop)

    # --- SETUP / MODE ---
    def stream_setpoints(self):
        """Send dummy velocity setpoints first so PX4/Ardupilot accepts offboard control"""
        if self.setpoint_stream_count < 50:
            twist = Twist()
            self.cmd_pub.publish(twist)
            self.setpoint_stream_count += 1
            if self.setpoint_stream_count == 1:
                self.get_logger().info('Starting setpoint streaming...')
        elif not self.mode_changed:
            self.change_mode_guided()
            self.arm_vehicle()
            self.mode_changed = True
            self.get_logger().info('Setpoint streaming finished.')
            self.setpoint_stream_timer.cancel()

    def change_mode_guided(self):
        if not self.set_mode_client.service_is_ready():
            self.get_logger().info('Waiting for /mavros/set_mode service...')
            return
        req = SetMode.Request()
        req.custom_mode = 'GUIDED'
        self.set_mode_client.call_async(req)

    def arm_vehicle(self):
        if not self.arming_client.service_is_ready():
            self.get_logger().info('Waiting for /mavros/cmd/arming service...')
            return
        req = CommandBool.Request()
        req.value = True
        self.arming_client.call_async(req)
        self.armed = True
        self.get_logger().info('Sent arming request.')

    # --- DEPTH CONTROL ---
    def depth_callback(self, msg):
        self.current_depth = msg.data
        if not self.depth_reached and self.current_depth <= self.target_depth + 0.1:
            self.depth_reached = True
            self.get_logger().info('Target depth reached!')

    def move_to_depth(self):
        pose = PoseStamped()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = self.target_depth  # NED: negative = down
        self.pos_pub.publish(pose)
        self.get_logger().info(f"Sending depth setpoint: {self.target_depth}m (current {self.current_depth:.2f})")

    # --- VISION ---
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

    # --- NAVIGATION ---
    def navigation_loop(self):
        # Step 1: Move to target depth
        if not self.depth_reached:
            self.move_to_depth()
            return

        twist = Twist()
        # Step 2: If gate detected or recently seen, navigate
        gate_to_use = None
        if self.current_gate:
            gate_to_use = self.current_gate
        elif self.last_gate_seen and self.last_seen_time:
            dt = (self.get_clock().now() - self.last_seen_time).nanoseconds / 1e9
            if dt < 3.0:  # up to 3s memory
                gate_to_use = self.last_gate_seen

        if gate_to_use:
            center_x = gate_to_use.x + gate_to_use.w / 2.0
            error_x = 0.5 - center_x  # center image at 0.5
            k_yaw = 1.5
            twist.angular.z = k_yaw * error_x
            twist.linear.x = 0.5  # slow forward
            self.get_logger().info(f'Moving toward gate, error_x={error_x:.2f}')
        else:
            # Step 3: Rotate slowly to search
            twist.angular.z = 0.3
            self.get_logger().info('Searching for gate...')

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = GateNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
