import rclpy
from rclpy.node import Node
from my_robot_msgs.msg import OdometryArray, GoalPoint
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray

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
        self.declare_parameter('is_turning', True) 
        self.is_turning:bool = self.get_parameter('is_turning').value # type: ignore


        self.boids={}
        self.odoms:dict[int,Odometry]={}
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

        # pub for neighbors
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
        
        self.average_odom_pub = self.create_publisher(Odometry, '/robot_avg/odom', 10)
        self.last_avg_odom=Odometry()
        self.timer = self.create_timer(1/20, self.calc_avg_odom)
        
        # for acceleration ploting
        self.acc_x_pub = self.create_publisher(Float32MultiArray, 'robot_all/accels/x', 10)
        self.acc_x = Float32MultiArray()
        self.acc_x.data = [0.0]*len(self.ids)
        self.acc_y_pub = self.create_publisher(Float32MultiArray, 'robot_all/accels/y', 10)
        self.acc_y = Float32MultiArray()
        self.acc_y.data = [0.0]*len(self.ids)
        self.acc_w_pub = self.create_publisher(Float32MultiArray, 'robot_all/accels/w', 10)
        self.acc_w = Float32MultiArray()
        self.acc_w.data = [0.0]*len(self.ids)
        self.last_speed_pose = {i:{"speed":None,"ang":None, "stamp": None} for i in self.ids}  # type: ignore
        
        self.acc_avg_pub = self.create_publisher(Float32MultiArray, 'robot_avg/acc', 10)
        self.acc_avg = Float32MultiArray()
        self.acc_avg.data = [0.0, 0.0] # x and y
        self.tw_avg_pub = self.create_publisher(Twist, 'robot_avg/twist', 10)
        self.tw_avg = Twist()
        self.last_v = [0.0, 0.0]




        
    def update_boid(self, msg:Odometry, id):
        '''Save boid in dictionary {id: {position: , orientation: , linear: , angular: }}'''
        # p = msg.pose.pose.position
        s = msg.twist.twist.linear
        a = msg.twist.twist.angular
        t = msg.header.stamp.nanosec * 1e-9 + msg.header.stamp.sec
        self.boids[id] = {'position': msg.pose.pose.position,
                            'orientation': msg.pose.pose.orientation,
                            'linear': msg.twist.twist.linear,
                            'angular':msg.twist.twist.angular}
        self.odoms[id] = msg
        
        if (self.last_speed_pose[id]['stamp'] is not None):
            dtime = t - self.last_speed_pose[id]['stamp'] # type: ignore
            dsx = s.x - self.last_speed_pose[id]['speed'].x # type: ignore
            dsy = s.y - self.last_speed_pose[id]['speed'].y # type: ignore
            daz = s.z - self.last_speed_pose[id]['ang'].z # type: ignore
            self.acc_x.data[id] = dsx/dtime
            self.acc_y.data[id] = dsy/dtime
            self.acc_w.data[id] = daz/dtime
        self.last_speed_pose[id]={'stamp':t, 'speed':s, 'ang':a} # type: ignore

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
        nearest_object:tuple = self.nearest_obstacle(self.boids[id_subject]['position'], id_subject) # type: ignore
        
        if nearest_object is not None and self.is_neighbor_obj(id_subject, nearest_object, self.max_distance, self.fov):
            msg.has_obstacle = True # type: ignore
            obstacle.x = nearest_object[0] # type: ignore
            obstacle.y = nearest_object[1] # type: ignore  
        else:
            msg.has_obstacle = False # type: ignore
        msg.odometries = neighbors
        msg.obstacle = obstacle
        msg.has_goal = self.goal.active # type: ignore
        msg.goal = self.goal.goal
        
        self.reynodl_data_pub[id_subject].publish(msg)
        
        
    def calc_avg_odom(self):
        
        self.acc_x_pub.publish(self.acc_x)
        self.acc_y_pub.publish(self.acc_y)
        self.acc_w_pub.publish(self.acc_w)
        
        avg_twist = Twist()
        avg_acc = Float32MultiArray()
        avg_odom=Odometry()
        n = len(self.odoms)
        if len(self.odoms) == 0:
            return
        for odom in self.odoms.values():
            avg_odom.pose.pose.position.x += odom.pose.pose.position.x/n
            avg_odom.pose.pose.position.y += odom.pose.pose.position.y/n
            avg_odom.pose.pose.orientation.x += odom.pose.pose.orientation.x/n
            avg_odom.twist.twist.linear.x += odom.twist.twist.linear.x/n 
            avg_odom.twist.twist.linear.y += odom.twist.twist.linear.y/n
            avg_odom.twist.twist.angular.z += odom.twist.twist.angular.z/n
        avg_odom.header = odom.header
        
        delta_x:float = -self.last_avg_odom.pose.pose.position.x + avg_odom.pose.pose.position.x
        delta_y:float = -self.last_avg_odom.pose.pose.position.y + avg_odom.pose.pose.position.y
        t1 = avg_odom.header.stamp
        t2 =  self.last_avg_odom.header.stamp
        delta_t:float =  t1.sec + t1.nanosec * 1e-9 - t2.sec - t2.nanosec * 1e-9
        avg_twist.linear.x = delta_x/delta_t
        avg_twist.linear.y = delta_y/delta_t
        avg_acc.data = [(avg_twist.linear.x - self.last_v[0])/delta_t,
                        (avg_twist.linear.y - self.last_v[1])/delta_t]
        delta_p:float = np.sqrt(delta_x**2 + delta_y**2)
        self.last_avg_odom = avg_odom
        self.last_v = [avg_twist.linear.x,avg_twist.linear.y]
        if (delta_p >= 0.001):
            self.average_odom_pub.publish(avg_odom)
            self.tw_avg_pub.publish(avg_twist)
            self.acc_avg_pub.publish(avg_acc)
        return
        
        
            
        
            
            
        
        



    def is_neighbor(self, id_subject, id_object, max_distance, fov):
        ''''Checks if boid object is neigbor of boid subject. Returns True if it is, False otherwise.'''
        dx = self.boids[id_object]['position'].x - self.boids[id_subject]['position'].x
        dy = self.boids[id_object]['position'].y - self.boids[id_subject]['position'].y
        distance = np.sqrt(dx*dx + dy*dy)
        if (distance > max_distance): # cjhecks distance
            return False

        q = self.boids[id_subject]['orientation']
        v = self.boids[id_subject]['linear']
        if self.is_turning: # heading from orientation
            yaw = np.arctan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y**2 + q.z**2)
            )
        else: # heading from motion vector
            yaw = np.arctan2(v.y,v.x)
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

        # self.get_logger().info(
        # f"found obsticle angle: {angle} < {fov/2}"
        # )

        # if (abs(angle) > fov/2): # checks angle
            # return False
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

    def nearest_obstacle(self, pose:Point, id_subject) -> tuple[int,int] | None:
        if len(self.obstacles) == 0:
            return None

        position = np.array([pose.x, pose.y])

        distances = np.linalg.norm(
            self.obstacles - position,
            axis=1
        )
        # check if its in view
        i=len(distances)
        closest_index = np.argmin(distances)
        max_d = distances[np.argmax(distances)]
        while(i>0):
            closest_index = np.argmin(distances)
            q = self.boids[id_subject]['orientation']
            v = self.boids[id_subject]['linear']
            if self.is_turning: # heading from orientation
                yaw = np.arctan2(
                    2 * (q.w * q.z + q.x * q.y),
                    1 - 2 * (q.y**2 + q.z**2)
                )
            else: # heading from motion vector
                yaw = np.arctan2(v.y,v.x)
            
            dx = self.obstacles[closest_index][0] - pose.x
            dy = self.obstacles[closest_index][1] - pose.y 
            angle = np.rad2deg(np.arctan2(dy,dx)- yaw)
            if abs(angle) > self.fov/2:
                distances[closest_index] = max_d+1.0
            else:
                break

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