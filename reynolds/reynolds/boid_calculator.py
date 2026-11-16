import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

import numpy as np



class Boid_calculator(Node):

    def __init__(self):
        super().__init__('Boid_calculator')
        self.current_vector = None
        self.current_pose = None
        self.obstacles = self.obstacles = np.empty((0, 2))
        
        self.declare_parameter('coefs', [1.0,1.0,1.0])
        self.coefs:list = self.get_parameter('coefs').value # type: ignore
        self.declare_parameter('fov', 270.0)
        self.fov:int = self.get_parameter('fov').value # type: ignore
        self.declare_parameter('max_distance', 3.0)
        self.max_distance:float = self.get_parameter('max_distance').value # type: ignore
        self.declare_parameter('min_distance', 1.0)
        self.min_distance:float = self.get_parameter('min_distance').value # type: ignore
        self.declare_parameter('min_spaceing', 0.5)
        self.min_spaceing:float = self.get_parameter('min_spaceing').value # type: ignore
        self.declare_parameter('max_linear', 1.0)
        self.max_linear:float = self.get_parameter('max_linear').value # type: ignore
        self.declare_parameter('max_angular', 1.0)
        self.max_angular:float = self.get_parameter('max_angular').value # type: ignore

        self.declare_parameter('spheroID',(0,1,2,3,4,5,6,7,8,9))
        self.ids:tuple = self.get_parameter('spheroID').value # type: ignore
        
        
        self.boids = {}
        self.odometry = [ # subscribe to all boids
            self.create_subscription(
                Odometry,
                f'/robot_{i}/odom',
                lambda msg, robot_id=i: self.update_boid(msg, robot_id), # type: ignore
                10
            )
            for i in self.ids # type: ignore
        ]
        self.newboid = [self.create_publisher(Twist, f'robot_{i}/cmd_vel', 10) for i in self.ids] # type: ignore # publisher for all robots
        
        self.boid_compute_timer = self.create_timer(1.0/60, self.calculate_boids_callback) # type: ignore # 30hz update rate for boids
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        
    def update_boid(self, msg:Odometry, id):
        '''Save boid in dictionary {id: {position: , orientation: , linear: , angular: }}'''
        self.boids[id] = {'position': msg.pose.pose.position,
                          'orientation': msg.pose.pose.orientation,
                          'linear': msg.twist.twist.linear,
                          'angular':msg.twist.twist.angular}
        
    def calculate_boids_callback(self) -> None:
        for i in self.boids:
            self.calculate_boid(i)
        return
        
    def calculate_boid(self, id) -> None:
        if len(self.boids) == 0:
            return
        # check for neighbors of the boid
        neighborIDs:list[int] = []
        neighborIDs_min:list[int] = []
        for boid in self.boids:
            if boid == id:
                continue
            else:
                if self.is_neighbor(id, boid, self.max_distance, self.fov): # farr
                    neighborIDs.append(boid)
                    
                if self.is_neighbor(id, boid, self.min_distance, self.fov): # cloase
                    neighborIDs_min.append(boid)
        
        if len(neighborIDs) == 0:
            return
        xy_vector = self.compute_boid_vector(id,neighborIDs,neighborIDs_min)
        new_vector = [xy_vector[0], xy_vector[1]]
        
        newmsg = Twist()
        newmsg.linear.x = new_vector[0]
        newmsg.linear.y = 0.0
        newmsg.linear.z = 0.0
        newmsg.angular.x = 0.0
        newmsg.angular.y = 0.0
        newmsg.angular.z = new_vector[1]
        self.newboid[id].publish(newmsg)
        return
    
    
    def compute_boid_vector(self, id, neighborhoodIDs, cloaseIDs):
        '''Computes the boid vector for a given boid id and its neighborhoodIDs. Returns np.array [x, y]'''
        mypose = self.boids[id]['position']
        myq = self.boids[id]['orientation']
        myyaw = np.arctan2(
            2 * (myq.w * myq.z + myq.x * myq.y),
            1 - 2 * (myq.y**2 + myq.z**2)
        )
        current_linear = self.boids[id]['linear'].x
        current_angular = self.boids[id]['angular'].z
    
        separation = self.separation_vector(mypose, cloaseIDs)
        alignment = self.alignment_vector(neighborhoodIDs)
        cohesion = self.cohesion_vector(mypose, neighborhoodIDs)
        
        obstacle = self.nearest_obstical(mypose.x, mypose.y)
        
        obstical_separation = np.array([0.0, 0.0])
        if obstacle is not None:
            # self.get_logger().info(
            # f"obsticle: {obstacle} <"
            # )
            if self.is_neighbor_obj(id, obstacle, self.min_spaceing, self.fov):
                obstical_separation = self.separation_vector(mypose, obstacle=obstacle)
        
        # combine the three vectors with their respective coefficients
        boid_vector = (self.coefs[0] * separation +
                       self.coefs[1] * alignment +
                       self.coefs[2] * cohesion +
                       self.coefs[3] * obstical_separation)
        
        R = np.array([
        [ np.cos(myyaw), np.sin(myyaw)],
        [-np.sin(myyaw), np.cos(myyaw)]
        ])
        boid_vector = R @ boid_vector
        kp1=[0.5 , 0.8]
        
        distance_error = np.sign(boid_vector[0])*np.linalg.norm(boid_vector)*kp1[0]
        heading_error = np.arctan2(boid_vector[1], boid_vector[0])
        distance_error = np.linalg.norm(boid_vector*max(0.01, np.cos(heading_error)))*kp1[0]

        angular_speed = np.clip(kp1[1] * heading_error, -self.max_angular, self.max_angular)
        
        
        eang = current_angular - angular_speed
        elin = -distance_error # current_linear - distance_error # acceleration
        p = [0.25, 0.2]
        return [current_linear-elin*p[0], current_angular-eang*p[1]]
    
    # Reynold rules =======================================================================================================================
    def separation_vector(self, pose, neighborhoodIDs = [], obstacle = None):
        '''Takes (pose[x,y,z] and all neighbors) return separration vector np.array [x, y] in global frame'''
        
        if(obstacle is not None):
            dx = -(obstacle[0] - pose.x)
            dy = -(obstacle[1] - pose.y)
            dist = np.sqrt(dx**2 + dy**2)
            return np.array([dx/dist**2, dy/dist**2])

        total_vector = np.array([0.0,0.0])
        for id in neighborhoodIDs: # type: ignore
            dx = -(self.boids[id]['position'].x - pose.x)
            dy = -(self.boids[id]['position'].y - pose.y)
            dist = np.sqrt(dx**2 + dy**2)
            total_vector = total_vector + np.array([dx/dist**2, dy/dist**2]) # in global frame
        return total_vector/max(len(neighborhoodIDs),1) # type: ignore
    
    def alignment_vector(self, neighborhoodIDs):
        '''computes average heading and spead vector for all neighbors in global frame'''
        average_heading_vector =  np.array([0.0,0.0])
        for id in neighborhoodIDs:
            q = self.boids[id]['orientation']
            yaw_bot = np.arctan2(
                        2 * (q.w * q.z + q.x * q.y),
                        1 - 2 * (q.y**2 + q.z**2)
                    )
            linear_bot = self.boids[id]['linear'].x
            heading_x_bot = np.cos(yaw_bot)*linear_bot
            heading_y_bot = np.sin(yaw_bot)*linear_bot
            average_heading_vector = average_heading_vector + np.array([heading_x_bot, heading_y_bot])
        return average_heading_vector/max(len(neighborhoodIDs),1)
    
    def cohesion_vector(self, pose, neighborhoodIDs):
        '''computes average distance to all neigbors and uses that as vector to center of neigborhood in global frame'''
        total_vector = np.array([0.0,0.0])
        for id in neighborhoodIDs:
            dx = self.boids[id]['position'].x - pose.x
            dy = self.boids[id]['position'].y - pose.y
            total_vector = total_vector + np.array([dx, dy]) # in global frame
        return total_vector/max(len(neighborhoodIDs),1)
    # Reynold end =========================================================================================================================
    
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
    
    def is_neighbor_obj(self, id_subject, pose_object, max_distance, fov):
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
        
        self.get_logger().info(
                f"obsticles: {self.obstacles}"
                )
    
    def nearest_obstical(self, pose_x, pose_y):
        if len(self.obstacles) == 0:
            return None

        position = np.array([pose_x, pose_y])

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
    rclpy.spin(Boid_calculator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
