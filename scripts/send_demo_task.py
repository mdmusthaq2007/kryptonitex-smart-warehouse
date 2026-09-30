#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from task_msgs.msg import RobotState, Task

class Demo(Node):
    def __init__(self):
        super().__init__('warehouse_demo_task')
        self.robot_pub = self.create_publisher(RobotState, '/robot_states', 10)
        self.task_pub = self.create_publisher(Task, '/task', 10)
        self.timer = self.create_timer(1.0, self.publish)
        self.sent = False
    def publish(self):
        r = RobotState(); r.robot_id='kryptonitex_forklift'; r.x=0.0; r.y=0.0; r.battery=95.0; r.workload=0
        self.robot_pub.publish(r)
        if not self.sent:
            t = Task(); t.x=10.0; t.y=10.0; t.priority=5; self.task_pub.publish(t); self.sent=True

def main():
    rclpy.init(); n=Demo(); rclpy.spin(n)

if __name__ == '__main__': main()
