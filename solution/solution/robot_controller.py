import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from auro_interfaces.msg import StringWithPose, Item, ItemList
from auro_interfaces.srv import ItemRequest

from tf_transformations import euler_from_quaternion
import angles

from enum import Enum
import random
import math

LINEAR_VELOCITY  = 0.3 # Metres per second
ANGULAR_VELOCITY = 0.5 # Radians per second

TURN_LEFT = 1 # Postive angular velocity turns left
TURN_RIGHT = -1 # Negative angular velocity turns right

SCAN_THRESHOLD = 0.5  # Metres per second
SCAN_FRONT = 0
SCAN_LEFT = 1
SCAN_BACK = 2
SCAN_RIGHT = 3

class State(Enum):
   SEARCHING = 0
   COLLECTING = 1
   DELIVERING = 2
   
    
class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        
        self.state = State.SEARCHING
        
        self.declare_parameter('robot_id', 'robot1')
        self.robot_id = self.get_parameter('robot_id').value

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value
        
        self.x = self.initial_x
        self.y = self.initial_y
        self.yaw = self.initial_yaw
        self.items = ItemList()
        
        self.scan_triggered = [False] * 4
        self.previous_yaw = self.yaw
        self.turn_angle = 0.0
        self.turn_direction = TURN_LEFT
        
        client_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/robot1/odom',
            self.odom_callback,
            10, callback_group=timer_callback_group)
            
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/robot1/scan',
            self.scan_callback,
            10, callback_group=timer_callback_group)

        self.pick_up_service = self.create_client(ItemRequest, '/robot1/pick_up_item', callback_group=client_callback_group)

        self.item_subscriber = self.create_subscription(
            ItemList,
            '/robot1/items',
            self.item_callback,
            10, callback_group=timer_callback_group
        )

        self.cmd_vel_publisher = self.create_publisher(Twist, 'robot1/cmd_vel', 10)

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)
	
    
    def odom_callback(self, msg):
        # Extract quaternion from the Odometry message
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        
        # Convert quaternion to euler angles
        _, _, self.yaw = euler_from_quaternion(quaternion)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
    	
    	
    def scan_callback(self, msg):
        # Group scan ranges into 4 segments
        front_ranges = msg.ranges[331:359] + msg.ranges[0:30] # 30 to 331 degrees (30 to -30 degrees)
        left_ranges  = msg.ranges[31:90] # 31 to 90 degrees (31 to 90 degrees)
        back_ranges  = msg.ranges[91:270] # 91 to 270 degrees (91 to -90 degrees)
        right_ranges = msg.ranges[271:330] # 271 to 330 degrees (-30 to -91 degrees)

        # Store True/False values for each sensor segment
        self.scan_triggered[SCAN_FRONT] = min(front_ranges) < SCAN_THRESHOLD 
        self.scan_triggered[SCAN_LEFT]  = min(left_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_BACK]  = min(back_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_RIGHT] = min(right_ranges) < SCAN_THRESHOLD
    

    def item_callback(self, msg):
        self.items = msg


    def control_loop(self):
        match self.state:
            case State.SEARCHING:
                self.searching()
            case State.COLLECTING:
                self.collecting()
            case State.DELIVERING:
                self.delivering()


    def searching(self):
        # Obstacle in front
        if self.scan_triggered[SCAN_FRONT]:
            self.previous_yaw = self.yaw
            self.turn_angle = random.uniform(90, 180)
            self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
            self.get_logger().info(f"Obstacle in front, turning {'left' if self.turn_direction == TURN_LEFT else 'right'} by {self.turn_angle:.2f} degrees")
            self.turn()
        
        # Obstacle to the left or right
        if self.scan_triggered[SCAN_LEFT] or self.scan_triggered[SCAN_RIGHT]:
            self.previous_yaw = self.yaw
            self.turn_angle = 45
            if self.scan_triggered[SCAN_LEFT] and self.scan_triggered[SCAN_RIGHT]:
                self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                self.get_logger().info(f"Obstacle to both sides, turning {'left' if self.turn_direction == TURN_LEFT else 'right'} by {self.turn_angle:.2f} degrees")
            elif self.scan_triggered[SCAN_LEFT]:
                self.turn_direction = TURN_RIGHT
                self.get_logger().info(f"Obstacle to the left, turning right by {self.turn_angle:.2f} degrees")
            else:
                self.turn_direction = TURN_LEFT
                self.get_logger().info(f"Obstacle to the right, turning left by {self.turn_angle:.2f} degrees")
            self.turn()

        # Item detected
        if len(self.items.data) > 0:
            self.state = State.COLLECTING
            return
        
        # No obstacles: Move forward
        msg = Twist()
        msg.linear.x = LINEAR_VELOCITY
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info("Moving forward")


    def turn(self):
        # Execute turning
        msg = Twist()
        msg.angular.z = self.turn_direction * ANGULAR_VELOCITY
        self.cmd_vel_publisher.publish(msg)

        # Calculate turn completion
        yaw_difference = angles.normalize_angle(self.yaw - self.previous_yaw)
        if math.fabs(yaw_difference) >= math.radians(self.turn_angle):
            self.get_logger().info("Turn completed, resuming forward movement")
    

    def collecting(self):
        # No items detected
        if len(self.items.data) == 0:
            self.state = State.SEARCHING
            return
        
        item = self.items.data[0]
        estimated_distance = 32.4 * float(item.diameter) ** -0.75
        self.get_logger().info(f'Estimated distance {estimated_distance}')
        
        if estimated_distance <= 0.35:
            rqt = ItemRequest.Request()
            rqt.robot_id = self.robot_id
            try:
                future = self.pick_up_service.call_async(rqt)
                self.executor.spin_until_future_complete(future)
                response = future.result()
                if response.success:
                    self.get_logger().info('Item picked up.')
                    self.state = State.DELIVERING
                    self.items.data = []
                else:
                    self.get_logger().info('Unable to pick up item: ' + response.message)
                except Exception as e:
                    self.get_logger().info('Exception ' + e) 

                        msg = Twist()

        msg.linear.x = 0.25 * estimated_distance
        msg.angular.z = item.x / 320.0
        self.cmd_vel_publisher.publish(msg)  


    def delivering(self):
        self.get_logger().info('State Delivering')

    def destroy_node(self):
        msg = Twist()
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f"Stopping: {msg}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)
    node = RobotController()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
