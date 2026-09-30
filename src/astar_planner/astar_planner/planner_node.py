import heapq
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from task_msgs.msg import RobotState, TaskAssignment


class WarehouseAStar:
    def __init__(self, sdf_path, min_x=-24.0, max_x=24.0, min_y=-24.0, max_y=24.0,
                 resolution=0.25, robot_radius=0.8):
        self.min_x, self.max_x = min_x, max_x
        self.min_y, self.max_y = min_y, max_y
        self.resolution = resolution
        self.width = int(round((max_x - min_x) / resolution))
        self.height = int(round((max_y - min_y) / resolution))
        self.blocked = set()
        self.inflate = int(math.ceil(robot_radius / resolution))
        self.load_rack_obstacles(sdf_path)

    def world_to_grid(self, x, y):
        return int(round((x - self.min_x) / self.resolution)), int(round((y - self.min_y) / self.resolution))

    def grid_to_world(self, gx, gy):
        return self.min_x + gx * self.resolution, self.min_y + gy * self.resolution

    def inside(self, p):
        x, y = p
        return 0 <= x < self.width and 0 <= y < self.height

    def load_rack_obstacles(self, sdf_path):
        root = ET.parse(sdf_path).getroot()
        world = root.find('world')
        if world is None:
            raise ValueError('SDF does not contain a <world> element')

        for model in world.findall('model'):
            name = model.get('name', '')
            if '_rack_' not in name or not name.startswith('z'):
                continue
            model_pose = self.parse_pose(model.findtext('pose', '0 0 0 0 0 0'))
            for collision in model.findall('.//collision'):
                geom = collision.find('geometry')
                box = geom.find('box') if geom is not None else None
                if box is None:
                    continue
                size = [float(v) for v in box.findtext('size', '').split()]
                if len(size) != 3 or size[2] <= 0:
                    continue
                local = self.parse_pose(collision.findtext('pose', '0 0 0 0 0 0'))
                yaw = model_pose[5]
                lx, ly = local[0], local[1]
                cx = model_pose[0] + math.cos(yaw) * lx - math.sin(yaw) * ly
                cy = model_pose[1] + math.sin(yaw) * lx + math.cos(yaw) * ly
                half_x = 0.5 * (abs(size[0] * math.cos(yaw)) + abs(size[1] * math.sin(yaw)))
                half_y = 0.5 * (abs(size[0] * math.sin(yaw)) + abs(size[1] * math.cos(yaw)))
                gx0, gy0 = self.world_to_grid(cx - half_x, cy - half_y)
                gx1, gy1 = self.world_to_grid(cx + half_x, cy + half_y)
                for gx in range(gx0 - self.inflate, gx1 + self.inflate + 1):
                    for gy in range(gy0 - self.inflate, gy1 + self.inflate + 1):
                        if self.inside((gx, gy)):
                            self.blocked.add((gx, gy))

    @staticmethod
    def parse_pose(text):
        vals = [float(v) for v in text.split()]
        vals += [0.0] * (6 - len(vals))
        return vals[:6]

    def nearest_free(self, p):
        if self.inside(p) and p not in self.blocked:
            return p
        for radius in range(1, 20):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    q = (p[0] + dx, p[1] + dy)
                    if self.inside(q) and q not in self.blocked:
                        return q
        return None

    def plan(self, start_xy, goal_xy):
        start = self.nearest_free(self.world_to_grid(*start_xy))
        goal = self.nearest_free(self.world_to_grid(*goal_xy))
        if start is None or goal is None:
            return []

        diagonal = [(1, 0), (-1, 0), (0, 1), (0, -1),
                    (1, 1), (1, -1), (-1, 1), (-1, -1)]
        open_heap = [(0.0, start)]
        came = {}
        g = {start: 0.0}

        def h(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == goal:
                path = [current]
                while current in came:
                    current = came[current]
                    path.append(current)
                path.reverse()
                return [self.grid_to_world(*p) for p in path]

            for dx, dy in diagonal:
                nxt = (current[0] + dx, current[1] + dy)
                if not self.inside(nxt) or nxt in self.blocked:
                    continue
                # Do not cut a diagonal through a rack corner.
                if dx and dy and ((current[0] + dx, current[1]) in self.blocked or
                                  (current[0], current[1] + dy) in self.blocked):
                    continue
                step = math.hypot(dx, dy)
                candidate = g[current] + step
                if candidate < g.get(nxt, float('inf')):
                    came[nxt] = current
                    g[nxt] = candidate
                    heapq.heappush(open_heap, (candidate + h(nxt, goal), nxt))
        return []


class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.declare_parameter('sdf_path', '')
        self.declare_parameter('min_x', -24.0)
        self.declare_parameter('max_x', 24.0)
        self.declare_parameter('min_y', -24.0)
        self.declare_parameter('max_y', 24.0)
        self.declare_parameter('resolution', 0.25)
        self.declare_parameter('robot_radius', 0.8)
        sdf = Path(str(self.get_parameter('sdf_path').value)).expanduser()
        if not sdf.is_absolute():
            sdf = Path.cwd() / sdf
        if not sdf.exists():
            raise FileNotFoundError(f'Warehouse SDF not found: {sdf}')
        self.planner = WarehouseAStar(
            str(sdf),
            float(self.get_parameter('min_x').value), float(self.get_parameter('max_x').value),
            float(self.get_parameter('min_y').value), float(self.get_parameter('max_y').value),
            float(self.get_parameter('resolution').value), float(self.get_parameter('robot_radius').value))
        self.robots = {}
        self.path_pub = self.create_publisher(PathMsg, '/astar/path', 10)
        self.create_subscription(RobotState, '/robot_states', self.robot_callback, 20)
        self.create_subscription(TaskAssignment, '/task_assignment', self.assignment_callback, 10)
        self.get_logger().info(f'A* ready; blocked cells: {len(self.planner.blocked)}')

    def robot_callback(self, msg):
        self.robots[msg.robot_id] = (msg.x, msg.y)

    def assignment_callback(self, msg):
        start = self.robots.get(msg.robot_id)
        if start is None:
            self.get_logger().warn(f'No state received yet for {msg.robot_id}')
            return
        points = self.planner.plan(start, (msg.x, msg.y))
        path = PathMsg()
        path.header.frame_id = 'map'
        for x, y in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)
        self.get_logger().info(f'Published A* path for {msg.robot_id}: {len(points)} points')


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
