"""ROS 2 node for the Backward Value Iteration demo.

Publishes the map, the BVI cost-to-go, the optimal path and an animated robot
for RViz2.

All algorithmic work lives in :mod:`bvi_nav.grid_map` and :mod:`bvi_nav.bvi`;
this file only turns their output into ROS messages. The node is namespace- and
frame-parameterised so the next assignment can spawn several of them.
"""

from __future__ import annotations

from typing import List, Optional

from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .bvi import backward_value_iteration, extract_path, verify
from .grid_map import Cell, GridMap

try:  # tf2_ros is optional; the node degrades gracefully without it.
    from tf2_ros import TransformBroadcaster
except ImportError:  # pragma: no cover
    TransformBroadcaster = None


def latched_qos(depth: int = 1) -> QoSProfile:
    """Reliable + transient_local, so RViz2 gets the data whenever it connects."""
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def cost_color(value: float, max_value: float) -> ColorRGBA:
    """Green (cheap, near the goal) -> yellow -> red (expensive, far away)."""
    t = 0.0 if max_value <= 0.0 else min(max(value / max_value, 0.0), 1.0)
    if t < 0.5:                       # green -> yellow
        r, g = 2.0 * t, 1.0
    else:                             # yellow -> red
        r, g = 1.0, 2.0 * (1.0 - t)
    return ColorRGBA(r=float(r), g=float(g), b=0.0, a=0.95)


class BVINode(Node):

    def __init__(self) -> None:
        super().__init__('bvi_node')

        # ----------------------------------------------------- parameters
        self.declare_parameter('seed', 42)
        self.declare_parameter('grid_size', 30)
        self.declare_parameter('obstacle_ratio', 0.20)
        self.declare_parameter('start', [-1, -1])     # [-1, -1] => choose automatically
        self.declare_parameter('goal', [-1, -1])      # [-1, -1] => choose automatically
        self.declare_parameter('step_rate', 4.0)      # animation ticks per second
        self.declare_parameter('loop', True)          # restart the animation at the end
        self.declare_parameter('loop_pause', 1.5)     # seconds held at the goal
        self.declare_parameter('show_labels', True)   # numeric V labels
        self.declare_parameter('label_stride', 1)     # draw every n-th label
        self.declare_parameter('publish_tf', True)    # broadcast map -> base_link
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('republish_period', 1.0)

        p = self.get_parameter
        size = int(p('grid_size').value)
        self.frame_id: str = str(p('frame_id').value)
        self.robot_frame: str = str(p('robot_frame').value)
        self.show_labels: bool = bool(p('show_labels').value)
        self.label_stride: int = max(1, int(p('label_stride').value))
        self.loop: bool = bool(p('loop').value)
        self.loop_pause: float = float(p('loop_pause').value)

        # ------------------------------------------------ map and planner
        self.map = GridMap(width=size, height=size,
                           obstacle_ratio=float(p('obstacle_ratio').value),
                           seed=int(p('seed').value))
        self.start, self.goal = self.map.pick_start_goal(
            self._param_cell('start'), self._param_cell('goal'))

        self.result = backward_value_iteration(self.map, self.goal)
        self.path: List[Cell] = extract_path(self.map, self.result, self.start)
        self._report()

        # -------------------------------------------------------- publishers
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', latched_qos())
        self.heatmap_pub = self.create_publisher(Marker, 'cost_heatmap', latched_qos())
        self.labels_pub = self.create_publisher(MarkerArray, 'cost_labels', latched_qos())
        self.start_goal_pub = self.create_publisher(MarkerArray, 'start_goal', latched_qos())
        self.path_pub = self.create_publisher(Path, 'optimal_path', latched_qos())
        self.path_line_pub = self.create_publisher(Marker, 'optimal_path_line', latched_qos())
        self.robot_pub = self.create_publisher(Marker, 'robot_marker', QoSProfile(depth=1))
        self.trail_pub = self.create_publisher(Marker, 'robot_trail', QoSProfile(depth=1))

        self.tf_broadcaster = (TransformBroadcaster(self)
                               if bool(p('publish_tf').value) and TransformBroadcaster
                               else None)

        # --------------------------------------------- cached static messages
        self.map_msg = self._build_map()
        self.heatmap_msg = self._build_heatmap()
        self.labels_msg = self._build_labels()
        self.start_goal_msg = self._build_start_goal()
        self.path_msg = self._build_path()
        self.path_line_msg = self._build_path_line()

        # ------------------------------------------------------------ timers
        self.publish_static()
        self.create_timer(float(p('republish_period').value), self.publish_static)

        self.step_index = 0
        self.hold_ticks = 0
        step_rate = max(0.1, float(p('step_rate').value))
        self.create_timer(1.0 / step_rate, self.animate)
        self.animate()   # show the robot at the start position immediately (Part 1)

        self.get_logger().info('bvi_node ready - set RViz2 Fixed Frame to '
                               f"'{self.frame_id}'")

    # ------------------------------------------------------------- helpers
    def _param_cell(self, name: str) -> Optional[List[int]]:
        raw = list(self.get_parameter(name).value or [])
        if len(raw) != 2 or int(raw[0]) < 0 or int(raw[1]) < 0:
            return None
        return [int(raw[0]), int(raw[1])]

    def _report(self) -> None:
        log = self.get_logger().info
        log('=' * 62)
        cells = self.map.width * self.map.height
        log(f'Grid {self.map.width}x{self.map.height}, seed={self.map.seed}, '
            f'obstacles={self.map.obstacle_count()} '
            f'({100.0 * self.map.obstacle_count() / cells:.1f}%)')
        log(f'Start={self.start}  Goal={self.goal}')
        log(f'BVI converged in {self.result.iterations} sweeps '
            f'({self.result.updates} value updates)')
        free = len(self.map.free_cells())
        log(f'Reachable states: {self.result.reachable} of {free} free cells')
        log(f'V(start) = {self.result.v(self.start):.0f}, '
            f'max finite V = {self.result.max_finite():.0f}')
        log(f'Optimal path length: {len(self.path) - 1} moves')
        log('-' * 62)
        for name, ok, detail in verify(self.map, self.result, self.start, self.path):
            log(f"[{'PASS' if ok else 'FAIL'}] {name}{' - ' + detail if detail else ''}")
        log('=' * 62)

    def _header(self, msg_header) -> None:
        msg_header.frame_id = self.frame_id
        msg_header.stamp = self.get_clock().now().to_msg()

    def _marker(self, ns: str, mid: int, mtype: int) -> Marker:
        m = Marker()
        self._header(m.header)
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.lifetime = Duration(seconds=0).to_msg()   # never auto-expire
        return m

    @staticmethod
    def center(cell: Cell, z: float = 0.0) -> Point:
        """Cell (x, y) -> world coordinates of its centre, (x + 0.5, y + 0.5)."""
        return Point(x=cell[0] + 0.5, y=cell[1] + 0.5, z=float(z))

    # ------------------------------------------------------ message builders
    def _build_map(self) -> OccupancyGrid:
        msg = OccupancyGrid()
        self._header(msg.header)
        msg.info.resolution = 1.0
        msg.info.width = self.map.width
        msg.info.height = self.map.height
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        # Row-major: index = y * width + x.
        msg.data = [100 if self.map.grid[y][x] else 0
                    for y in range(self.map.height)
                    for x in range(self.map.width)]
        return msg

    def _build_heatmap(self) -> Marker:
        m = self._marker('cost_heatmap', 0, Marker.CUBE_LIST)
        m.scale.x = m.scale.y = 1.0
        m.scale.z = 0.05
        m.color.a = 1.0
        vmax = self.result.max_finite()
        for cell in self.map.free_cells():
            if not self.result.is_reachable(cell):
                continue                     # unreachable cells stay uncoloured
            m.points.append(self.center(cell, 0.05))
            m.colors.append(cost_color(self.result.v(cell), vmax))
        return m

    def _build_labels(self) -> MarkerArray:
        arr = MarkerArray()
        if not self.show_labels:
            clear = Marker()
            self._header(clear.header)
            clear.ns = 'cost_labels'
            clear.action = Marker.DELETEALL
            arr.markers.append(clear)
            return arr
        for cell in self.map.free_cells():
            if not self.result.is_reachable(cell):
                continue                     # never label obstacles or unreachable cells
            x, y = cell
            if x % self.label_stride or y % self.label_stride:
                continue
            m = self._marker('cost_labels', y * self.map.width + x,
                             Marker.TEXT_VIEW_FACING)
            m.pose.position = self.center(cell, 0.2)
            m.scale.z = 0.35
            m.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
            m.text = f'{int(self.result.v(cell))}'
            arr.markers.append(m)
        return arr

    def _build_start_goal(self) -> MarkerArray:
        arr = MarkerArray()
        specs = (
            ('start', 0, self.start, ColorRGBA(r=0.0, g=0.9, b=0.1, a=1.0), 'START'),
            ('goal', 1, self.goal, ColorRGBA(r=0.9, g=0.0, b=0.0, a=1.0), 'GOAL'),
        )
        for ns, mid, cell, color, text in specs:
            body = self._marker(ns, mid, Marker.CYLINDER)
            body.pose.position = self.center(cell, 0.25)
            body.scale.x = body.scale.y = 0.8
            body.scale.z = 0.5
            body.color = color
            arr.markers.append(body)

            label = self._marker(ns, mid + 100, Marker.TEXT_VIEW_FACING)
            label.pose.position = self.center(cell, 1.4)
            label.scale.z = 1.0
            label.color = color
            label.text = text
            arr.markers.append(label)
        return arr

    def _build_path(self) -> Path:
        msg = Path()
        self._header(msg.header)
        for cell in self.path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position = self.center(cell, 0.3)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def _build_path_line(self) -> Marker:
        m = self._marker('optimal_path_line', 0, Marker.LINE_STRIP)
        m.scale.x = 0.25
        m.color = ColorRGBA(r=0.1, g=0.4, b=1.0, a=1.0)
        m.points = [self.center(c, 0.3) for c in self.path]
        return m

    def _build_robot(self, cell: Cell) -> Marker:
        m = self._marker('robot', 0, Marker.SPHERE)
        m.pose.position = self.center(cell, 0.5)
        m.scale.x = m.scale.y = m.scale.z = 0.9
        m.color = ColorRGBA(r=1.0, g=0.3, b=0.9, a=1.0)
        return m

    def _build_trail(self, upto: int) -> Marker:
        m = self._marker('robot_trail', 0, Marker.LINE_STRIP)
        if upto < 1:                       # a LINE_STRIP needs at least two points
            m.action = Marker.DELETE
            return m
        m.scale.x = 0.35
        m.color = ColorRGBA(r=1.0, g=1.0, b=0.2, a=0.9)
        m.points = [self.center(c, 0.45) for c in self.path[:upto + 1]]
        return m

    # -------------------------------------------------------------- publish
    def publish_static(self) -> None:
        """(Re)publish everything that does not move, once per second as a fallback."""
        for msg in (self.map_msg, self.path_msg):
            self._header(msg.header)
        for msg in (self.heatmap_msg, self.path_line_msg):
            self._header(msg.header)
        for marker in list(self.labels_msg.markers) + list(self.start_goal_msg.markers):
            self._header(marker.header)

        self.map_pub.publish(self.map_msg)
        self.heatmap_pub.publish(self.heatmap_msg)
        self.labels_pub.publish(self.labels_msg)
        self.start_goal_pub.publish(self.start_goal_msg)
        self.path_pub.publish(self.path_msg)
        self.path_line_pub.publish(self.path_line_msg)

    def animate(self) -> None:
        """Advance the robot one cell along the optimal path and republish it."""
        if self.hold_ticks > 0:
            self.hold_ticks -= 1
            if self.hold_ticks == 0:
                self.step_index = 0
            self._publish_robot()
            return

        cell = self.path[self.step_index]
        self._publish_robot()

        if self.step_index < len(self.path) - 1:
            self.step_index += 1
        elif self.loop:
            rate = max(0.1, float(self.get_parameter('step_rate').value))
            self.hold_ticks = max(1, int(self.loop_pause * rate))
            self.get_logger().info(f'Goal {cell} reached - restarting the animation')

    def _publish_robot(self) -> None:
        cell = self.path[self.step_index]
        self.robot_pub.publish(self._build_robot(cell))
        self.trail_pub.publish(self._build_trail(self.step_index))
        if self.tf_broadcaster is not None:
            t = TransformStamped()
            self._header(t.header)
            t.child_frame_id = self.robot_frame
            t.transform.translation.x = cell[0] + 0.5
            t.transform.translation.y = cell[1] + 0.5
            t.transform.translation.z = 0.5
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BVINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
