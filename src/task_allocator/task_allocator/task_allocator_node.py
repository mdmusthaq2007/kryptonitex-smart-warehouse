import math

import rclpy
from rclpy.node import Node

from task_msgs.msg import RobotState, Task, TaskAssignment


class TaskAllocator(Node):
    """Local bid-based allocator for one warehouse task at a time."""

    def __init__(self):
        super().__init__('task_allocator')
        self.declare_parameter('minimum_battery_percent', 15.0)
        self.declare_parameter('workload_weight', 2.0)
        self.declare_parameter('battery_weight', 0.05)
        self.declare_parameter('priority_weight', 1.5)

        self.robots = {}
        self.task = None
        self.create_subscription(Task, '/task', self.task_callback, 10)
        self.create_subscription(RobotState, '/robot_states', self.robot_state_callback, 20)
        self.assignment_pub = self.create_publisher(TaskAssignment, '/task_assignment', 10)

        self.get_logger().info('Kryptonitex task allocator ready.')

    def robot_state_callback(self, msg: RobotState):
        self.robots[msg.robot_id] = msg

    def task_callback(self, msg: Task):
        self.task = msg
        self.allocate()

    def bid(self, robot: RobotState) -> float:
        distance = math.hypot(robot.x - self.task.x, robot.y - self.task.y)
        workload_penalty = robot.workload * float(self.get_parameter('workload_weight').value)
        battery_penalty = (100.0 - robot.battery) * float(self.get_parameter('battery_weight').value)
        priority_bonus = self.task.priority * float(self.get_parameter('priority_weight').value)
        return distance + workload_penalty + battery_penalty - priority_bonus

    def allocate(self):
        if self.task is None or not self.robots:
            return

        min_battery = float(self.get_parameter('minimum_battery_percent').value)
        candidates = {
            name: robot for name, robot in self.robots.items()
            if robot.battery >= min_battery
        }
        if not candidates:
            self.get_logger().warn('No robot satisfies the minimum battery threshold.')
            return

        winner = min(candidates, key=lambda name: self.bid(candidates[name]))
        assignment = TaskAssignment()
        assignment.robot_id = winner
        assignment.x = self.task.x
        assignment.y = self.task.y
        self.assignment_pub.publish(assignment)
        self.get_logger().info(
            f'assigned task ({self.task.x:.2f}, {self.task.y:.2f}) to {winner}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
