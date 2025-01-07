import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose

from assessment_interfaces.msg import Item, ItemList, Zone, ZoneList
from auro_interfaces.srv import ItemRequest

from tf_transformations import euler_from_quaternion, quaternion_from_euler
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

class SearchingMode(Enum):
	FORWARD = 0
	TURNING = 1

class RobotController(Node):

	def __init__(self):
		super().__init__('robot_controller')

		self.state = State.SEARCHING
		self.searching_mode = SearchingMode.FORWARD

		self.declare_parameter('robot_id', 'robot1')
		self.robot_id = self.get_parameter('robot_id').value

		self.declare_parameter('x', 0.0)
		self.declare_parameter('y', 0.0)
		self.declare_parameter('yaw', 0.0)

		self.initial_x = self.get_parameter('x').get_parameter_value().double_value
		self.initial_y = self.get_parameter('y').get_parameter_value().double_value
		self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value

		self.goal_distance = random.uniform(4.0, 8.0)

		self.x = self.initial_x
		self.y = self.initial_y
		self.yaw = self.initial_yaw
		self.items = ItemList()
		self.zones = ZoneList()
		self.picked_up_item = None

		self.scan_triggered = [False] * 4
		self.previous_yaw = self.yaw
		self.previous_x = self.initial_x
		self.previous_y = self.initial_y
		self.turn_angle = 0.0
		self.turn_direction = TURN_LEFT

		client_callback_group = MutuallyExclusiveCallbackGroup()
		timer_callback_group = MutuallyExclusiveCallbackGroup()

		self.odom_subscriber = self.create_subscription(
			Odometry,
			f'/{self.robot_id}/odom',
			self.odom_callback,
			10, callback_group=timer_callback_group)

		self.scan_subscriber = self.create_subscription(
			LaserScan,
			f'/{self.robot_id}/scan',
			self.scan_callback,
			10, callback_group=timer_callback_group)

		self.pick_up_service = self.create_client(ItemRequest, '/pick_up_item', callback_group=client_callback_group)
		self.offload_service = self.create_client(ItemRequest, '/offload_item', callback_group=client_callback_group)

		self.item_subscriber = self.create_subscription(
			ItemList,
			f'/{self.robot_id}/items',
			self.item_callback,
			10, callback_group=timer_callback_group)
			
		self.zone_subscriber = self.create_subscription(
			ZoneList,
			f'/{self.robot_id}/zone',
			self.zone_callback,
			10, callback_group=timer_callback_group)

		self.cmd_vel_publisher = self.create_publisher(Twist, f'/{self.robot_id}/cmd_vel', 10)
		self.nav_to_pose_client = ActionClient(self, NavigateToPose, f'/{self.robot_id}/navigate_to_pose')

		self.timer_period = 0.1 # 100 milliseconds = 10 Hz
		self.timer = self.create_timer(self.timer_period, self.control_loop)

	def odom_callback(self, msg):
		orientation = msg.pose.pose.orientation
		quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]

		_, _, self.yaw = euler_from_quaternion(quaternion)
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y

	def scan_callback(self, msg):
		front_ranges = msg.ranges[331:359] + msg.ranges[0:30]
		left_ranges  = msg.ranges[31:90]
		back_ranges  = msg.ranges[91:270]
		right_ranges = msg.ranges[271:330]

		self.scan_triggered[SCAN_FRONT] = min(front_ranges) < SCAN_THRESHOLD 
		self.scan_triggered[SCAN_LEFT]  = min(left_ranges)  < SCAN_THRESHOLD
		self.scan_triggered[SCAN_BACK]  = min(back_ranges)  < SCAN_THRESHOLD
		self.scan_triggered[SCAN_RIGHT] = min(right_ranges) < SCAN_THRESHOLD

	def item_callback(self, msg):
		self.items = msg
		
	def zone_callback(self, msg):
		self.zones = msg
		
	def navigate_to_pose(self, x, y, yaw=0.0):
	
		if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
			self.get_logger().error("NavigateToPose action server not available!")
			return False
			
		goal_pose = PoseStamped()
		goal_pose.header.frame_id = "map"
		goal_pose.header.stamp = self.get_clock().now().to_msg()
		
		goal_pose.pose.position.x = x
		goal_pose.pose.position.y = y
		goal_pose.pose.orientation.z = math.sin(0)
		goal_pose.pose.orientation.w = math.cos(0)
		
		self.get_logger().info(f"Navigating to goal: x={x}, y={y}")
		future = self.nav_to_pose_client.send_goal_async(goal_msg)
		
		rclpy.spin_until_future_complete(self, future)
		
		if not future.result().accepted:
			self.get_logger().error("Goal rejected by server!")
			return False
			
		goal_handle = future.result()
		result_future = goal_handle.get_result_async()
		rclpy.spin_until_future_complete(self, result_future)
		
		if result_future.result().status == NavigateToPose.Result.SUCCEEDED:
			self.get_logger().info("Navigation succeeded!")
			return True
		else:
			self.get_logger().error("Navigation failed!")
			return False	

	def control_loop(self):
		match self.state:
			case State.SEARCHING:
				self.searching()
			case State.COLLECTING:
				self.collecting()
			case State.DELIVERING:
				self.delivering()

	def searching(self):
		if self.searching_mode == SearchingMode.FORWARD:
			self.forward()
		elif self.searching_mode == SearchingMode.TURNING:
			self.turning()

	def forward(self):
		self.get_logger().info("Forward state")

		if self.scan_triggered[SCAN_FRONT]:
			self.previous_yaw = self.yaw
			self.searching_mode = SearchingMode.TURNING
			self.turn_angle = random.uniform(150, 170)
			self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
			return

		if self.scan_triggered[SCAN_LEFT] or self.scan_triggered[SCAN_RIGHT]:
			self.previous_yaw = self.yaw
			self.searching_mode = SearchingMode.TURNING
			self.turn_angle = 45
			if self.scan_triggered[SCAN_LEFT] and self.scan_triggered[SCAN_RIGHT]:
				self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
			elif self.scan_triggered[SCAN_LEFT]:
				self.turn_direction = TURN_RIGHT
			else:
				self.turn_direction = TURN_LEFT
			return

		if len(self.items.data) > 0:
			self.state = State.COLLECTING
			return

		msg = Twist()
		msg.linear.x = LINEAR_VELOCITY
		self.cmd_vel_publisher.publish(msg)
		difference_x = self.x - self.previous_x
		difference_y = self.y - self.previous_y
		distance_travelled = math.sqrt(difference_x ** 2 + difference_y ** 2)

		if distance_travelled >= self.goal_distance:
			self.previous_yaw = self.yaw
			self.searching_mode = SearchingMode.TURNING
			self.turn_angle = random.uniform(30, 70)
			self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])

	def turning(self):
		if len(self.items.data) > 0:
			self.state = State.COLLECTING
			return

		msg = Twist()
		msg.angular.z = self.turn_direction * ANGULAR_VELOCITY
		self.cmd_vel_publisher.publish(msg)

		yaw_difference = angles.normalize_angle(self.yaw - self.previous_yaw)

		if math.fabs(yaw_difference) >= math.radians(self.turn_angle):
			self.previous_x = self.x
			self.previous_y = self.y 
			self.goal_distance = random.uniform(1.0, 2.0)
			self.searching_mode = SearchingMode.FORWARD

	def collecting(self):
	
		if len(self.items.data) == 0:
		    self.state = State.SEARCHING
		    return
	
		item = max(self.items.data, key=lambda x: x.diameter)
		estimated_distance = 32.4 * float(item.diameter) ** -0.75

		if estimated_distance <= 0.36:
			rqt = ItemRequest.Request()
			rqt.robot_id = self.robot_id
			try:
				future = self.pick_up_service.call_async(rqt)
				self.executor.spin_until_future_complete(future)
				response = future.result()
				if response.success:
					self.get_logger().info('Item picked up.')
					self.picked_up_item = item
					self.state = State.DELIVERING
					self.items.data = []
					msg = Twist()
					self.cmd_vel_publisher.publish(msg)
					return
				else:
					self.get_logger().info('Unable to pick up item: ' + response.message)
			except Exception as e:
				self.get_logger().info('Exception ' + str(e))

		msg = Twist()
		msg.linear.x = 0.40 * estimated_distance
		msg.angular.z = item.x / 320.0
		self.cmd_vel_publisher.publish(msg)

	def delivering(self):
		self.get_logger().info('State Delivering')
		if self.picked_up_item == None:
		    self.state = State.SEARCHING
		    self.searching_mode = SearchingMode.FORWARD
		    return
		    
		if self.picked_up_item.colour == "RED":
		    self.navigate_to_pose(-3.42, -2.46)

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
		executor.spin()
	except KeyboardInterrupt:
		pass
	except ExternalShutdownException:
		sys.exit(1)
	finally:
		node.destroy_node()
		rclpy.try_shutdown()


if __name__ == '__main__':
	main()
