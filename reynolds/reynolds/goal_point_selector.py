import threading

import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from my_robot_msgs.msg import GoalPoint
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry


class GoalPointSelector(Node):

    def __init__(self):
        super().__init__('GoalPointSelector')

        self.goalPoint = Point()
        self.active = False
        self.map_msg = None
        
        self.declare_parameter('path_l', 60) #sec
        self.path_l:int = self.get_parameter('path_l').value * 20 # type: ignore

        self.goal_pub = self.create_publisher(GoalPoint, '/goal_point', 10)
        
        self.path_sub = self.create_subscription(Odometry, '/robot_avg/odom',self.moved, 10)
        self.path:list[list[float]]=[]
        self.time = 0.0
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.get_map = self.create_subscription(
            OccupancyGrid, '/map', self.map_update, map_qos)
    
    
    def moved(self, msg:Odometry):
        self.path.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.pop_path()
    def pop_path(self):
        if len(self.path)>self.path_l:
            self.path.pop(0)

    def map_update(self, msg: OccupancyGrid):
        if self.map_msg is None:          # static map: keep only the first one
            self.map_msg = msg
            self.get_logger().info(f'Map received: {msg.info.width}x{msg.info.height}')

    def publish_goal(self):
        out = GoalPoint()
        out.active = self.active
        out.goal = self.goalPoint
        self.goal_pub.publish(out)

    def wait_for_map(self):
        while rclpy.ok() and self.map_msg is None:
            self.get_logger().info('Waiting for /map...', throttle_duration_sec=2.0)
            rclpy.spin_once(self, timeout_sec=0.5)

    def Ui(self):
        m = self.map_msg
        w, h, res = m.info.width, m.info.height, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y

        grid = np.array(m.data, dtype=np.int8).reshape((h, w))
        img = np.full(grid.shape, 0.5)   # unknown = gray
        img[grid == 0] = 1.0             # free = white
        img[grid >= 50] = 0.0            # occupied = black

        self.fig, self.ax = plt.subplots()
        self.ax.imshow(img, cmap='gray', vmin=0, vmax=1, origin='lower',
                       extent=[ox, ox + w * res, oy, oy + h * res])
        self.marker, = self.ax.plot([], [], 'r*', markersize=5)
        self.path_line, = self.ax.plot([], [], 'b-', linewidth=1.5)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('x [m]')
        self.ax.set_ylabel('y [m]')
        self.ax.set_title('Left click: set goal | Right click: clear goal')

        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
        self.timer = self.fig.canvas.new_timer(interval=500)
        self.timer.add_callback(self.update_plot)
        self.timer.start()

        plt.show()
        
    def update_plot(self):
        if len(self.path) > 0:
            path = np.array(self.path)

            self.path_line.set_data(path[:, 0], path[:, 1])

        self.fig.canvas.draw_idle()

    def on_click(self, event):
        if event.inaxes != self.ax or event.xdata is None:
            return
        toolbar = self.fig.canvas.toolbar
        if toolbar is not None and toolbar.mode != '':   # zoom/pan active
            return

        if event.button == 1:        # left click: select goal
            self.goalPoint = Point(x=float(event.xdata), y=float(event.ydata), z=0.0)
            self.active = True
            self.marker.set_data([event.xdata], [event.ydata])
            self.get_logger().info(f'Goal set: ({event.xdata:.2f}, {event.ydata:.2f})')
        elif event.button == 3:      # right click: deactivate
            self.active = False
            self.marker.set_data([], [])
            self.get_logger().info('Goal cleared')
        else:
            return

        self.publish_goal()
        self.fig.canvas.draw_idle()


def main(args=None):
    rclpy.init(args=args)
    node = GoalPointSelector()
    node.wait_for_map()                  # blocks until the map arrives
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.Ui()                        # blocks until the window is closed
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()