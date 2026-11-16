import rclpy
from rclpy.node import Node
from my_robot_msgs.msg import OdometryArray, GoalPoint
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


import numpy as np

class NeighborAlocator(Node):

    def __init__(self):
        super().__init__('NeighborAlocator')
        self.declare_parameter('spheroID',(0,1,2,3,4,5,6,7,8,9))
        self.ids:tuple = self.get_parameter('spheroID').value # type: ignore
       


        self.declare_parameter('fov', 270.0)
        self.fov:int = self.get_parameter('fov').value # type: ignore
        self.declare_parameter('max_distance', 3.0)
        self.max_distance:float = self.get_parameter('max_distance').value # type: ignore


        self.boids={}
        self.odoms={}
        self.obstacles = self.obstacles = np.empty((0, 2))
        self.goal = GoalPoint()
        self.goal.active=False

        self.odometry = [ # subscribe to all boids
            self.create_subscription(
                Odometry,
                f'/robot_{i}/odom',
                lambda msg, robot_id=i: self.update_boid(msg, robot_id), # type: ignore
                10
            )
            for i in self.ids # type: ignore
        ]

        self.reynodl_data_pub = [self.create_publisher(OdometryArray, f'robot_{i}/neigbors', 10) for i in self.ids]
        
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
)
        
        self.goal_sub = self.create_subscription(
            GoalPoint, '/goal_point', self.update_goal, 10)




        
    def update_boid(self, msg:Odometry, id):
        '''Save boid in dictionary {id: {position: , orientation: , linear: , angular: }}'''
        self.boids[id] = {'position': msg.pose.pose.position,
                            'orientation': msg.pose.pose.orientation,
                            'linear': msg.twist.twist.linear,
                            'angular':msg.twist.twist.angular}
        self.odoms[id] = msg
        self.send_neighbors_data(id, msg.header)
        
    def update_goal(self, msg:GoalPoint):
        self.goal = msg



    def send_neighbors_data(self, id_subject:int, header):
        msg = OdometryArray()
        msg.header=header
        neighbors:list[Odometry] = []
        obstacle = Point()
        for id in self.boids:
            if id != id_subject and self.is_neighbor(id_subject,id,self.max_distance,self.fov):
                neighbors.append(self.odoms[id])
        nearest_object:tuple = self.nearest_obstacle(self.boids[id_subject]['position']) # type: ignore
        
        if nearest_object is not None and self.is_neighbor_obj(id_subject, nearest_object, self.max_distance, self.fov):
            msg.has_obstacle = True
            obstacle.x = nearest_object[0] # type: ignore
            obstacle.y = nearest_object[1] # type: ignore  
        else:
            msg.has_obstacle = False
        msg.odometries = neighbors
        msg.obstacle = obstacle
        msg.has_goal = self.goal.active
        msg.goal = self.goal.goal
        
        self.reynodl_data_pub[id_subject].publish(msg)
            
            
        
        



    def is_neighbor(self, id_subject, id_object, max_distance, fov):
        ''''Checks if boid object is neigbor of boid subject. Returns True if it is, False otherwise.'''
        dx = self.boids[id_object]['position'].x - self.boids[id_subject]['position'].x
        dy = self.boids[id_object]['position'].y - self.boids[id_subject]['position'].y
        distance = np.sqrt(dx*dx + dy*dy)
        if (distance > max_distance): # cjhecks distance
            return False

        q = self.boids[id_subject]['orientation']
        yaw = np.arctan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y**2 + q.z**2)
        )
        angle = np.rad2deg(np.arctan2(dy,dx)- yaw)
        if (abs(angle) > fov/2): # checks angle
            return False
        return True
    
    def is_neighbor_obj(self, id_subject:int, pose_object:tuple[int,int], max_distance:float, fov:float):
        ''''Checks if object is neigbor of boid subject. Returns True if it is, False otherwise.'''
        dx = pose_object[0] - self.boids[id_subject]['position'].x
        dy = pose_object[1] - self.boids[id_subject]['position'].y
        distance = np.sqrt(dx*dx + dy*dy)

        # self.get_logger().info(
        # f"found obsticle dist: {distance} < {max_distance}"
        # )

        if (distance > max_distance): # cjhecks distance
            return False

        q = self.boids[id_subject]['orientation']
        yaw = np.arctan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y**2 + q.z**2)
        )
        angle = np.rad2deg(np.arctan2(dy,dx)- yaw)

        # self.get_logger().info(
        # f"found obsticle angle: {angle} < {fov/2}"
        # )

        if (abs(angle) > fov/2): # checks angle
            return False
        return True

    def map_callback(self, msg):

        self.get_logger().info(
        f"Map received: {msg.info.width}x{msg.info.height}"
        )

        grid = np.array(msg.data).reshape(
        (msg.info.height, msg.info.width)
        )

        # Cells considered occupied
        occupied_y, occupied_x = np.where(grid >= 50)

        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Cell-center coordinates in the map frame
        x_coords = origin_x + (occupied_x + 0.5) * resolution
        y_coords = origin_y + (occupied_y + 0.5) * resolution

        self.obstacles = np.column_stack((x_coords, y_coords))

        # self.get_logger().info(
        #         f"obsticles: {self.obstacles}"
        #         )

    def nearest_obstacle(self, pose:Point) -> tuple[int,int] | None:
        if len(self.obstacles) == 0:
            return None

        position = np.array([pose.x, pose.y])

        distances = np.linalg.norm(
            self.obstacles - position,
            axis=1
        )
        closest_index = np.argmin(distances)

        # self.get_logger().info(
        #                 f"found obsticle: {self.obstacles[closest_index]}"
        #                 )
        return self.obstacles[closest_index]
    
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(NeighborAlocator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()