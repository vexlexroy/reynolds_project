import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from my_robot_msgs.msg import OdometryArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point


import numpy as np


class BoidCalculator_decent(Node):

    def __init__(self):
        super().__init__('BoidCalculator_decent')
        self.declare_parameter('id', 1) 
        self.id:int = self.get_parameter('id').value # type: ignore
        self.boids = {}
        self.last_time = None
        self.pose = None
        self.orient = None
        self.linear = None
        self.angular = None
        self.goal_point = None

        self.declare_parameter('coefs', [1.0,1.0,1.0,1.0,1.0]) # vector coefs (sep, alig, coh, obs, goal)
        self.coefs:list = self.get_parameter('coefs').value # type: ignore
        self.declare_parameter('mass', 1.0) 
        self.mass:float = self.get_parameter('mass').value # type: ignore
        self.declare_parameter('is_turning', True) 
        self.is_turning:bool = self.get_parameter('is_turning').value # type: ignore

        self.declare_parameter('max_linear', 1.0)
        self.max_linear:float = self.get_parameter('max_linear').value # type: ignore
        self.declare_parameter('min_linear', 0.0)
        self.min_linear:float = self.get_parameter('min_linear').value # type: ignore
        self.declare_parameter('max_acceleration', 1.0)
        self.max_acceleration:float = self.get_parameter('max_acceleration').value # type: ignore
        self.declare_parameter('max_decceleration', 1.0)
        self.max_decceleration:float = self.get_parameter('max_decceleration').value # type: ignore
        self.declare_parameter('max_angular', 3.0)
        self.max_angular:float = self.get_parameter('max_angular').value # type: ignore
        self.declare_parameter('turn_rate', 30.0)
        self.turn_rate:float = self.get_parameter('turn_rate').value # type: ignore



        #sub for neighboring robot data
        self.odoms = self.create_subscription(
            OdometryArray, f'robot_{self.id}/neigbors', self.odoms_callback, 10)

        #sub for self data
        self.state = self.create_subscription(
            Odometry, f'robot_{self.id}/odom', self.self_state_callback, 10)

        #pub for self velocity data
        self.newboid = self.create_publisher(Twist, f'robot_{self.id}/cmd_vel', 10) 



    # Reynold rules =======================================================================================================================
    def separation_vector(self, pose, neighborhoodIDs:list[int]):
        '''Takes (pose[x,y,z] and all neighbors) return separration vector np.array [x, y] in global frame'''
        total_vector = np.array([0.0,0.0])
        for id in neighborhoodIDs: # type: ignore
            dx = -(self.boids[id]['position'].x - pose.x)
            dy = -(self.boids[id]['position'].y - pose.y)
            dist = np.sqrt(dx**2 + dy**2)
            total_vector = total_vector + np.array([dx/dist**2, dy/dist**2]) # in global frame
        return total_vector/max(len(neighborhoodIDs),1) # type: ignore

    def alignment_vector(self, neighborhoodIDs:list[int]):
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

    def cohesion_vector(self, pose, neighborhoodIDs:list[int]):
        '''computes average distance to all neigbors and uses that as vector to center of neigborhood in global frame'''
        total_vector = np.array([0.0,0.0])
        for id in neighborhoodIDs:
            dx = self.boids[id]['position'].x - pose.x
            dy = self.boids[id]['position'].y - pose.y
            total_vector = total_vector + np.array([dx, dy]) # in global frame
        return total_vector/max(len(neighborhoodIDs),1)


    def get_obstacle_vector(self, pose, obstacle:Point):
        dx = -(obstacle.x - pose.x)
        dy = -(obstacle.y - pose.y)
        dist = np.sqrt(dx**2 + dy**2)
        return np.array([dx/dist**4, dy/dist**4])

    def get_goal_vector(self, pose, goalpose:Point):
        dx = (goalpose.x - pose.x)
        dy = (goalpose.y - pose.y)
        # dist = np.sqrt(dx**2 + dy**2)
        return np.array([dx, dy])
    # Reynold end =========================================================================================================================


    def odoms_callback(self, message:OdometryArray):
        time = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        msg = message.odometries
        obstacle = None
        goal = None
        if message.has_obstacle:
            obstacle = message.obstacle
        if message.has_goal:
            goal = message.goal
        self.boids = {}
        for id,x in enumerate(msg):
            self.boids[id] = {'position': x.pose.pose.position,
                                      'orientation': x.pose.pose.orientation,
                                      'linear': x.twist.twist.linear,
                                      'angular':x.twist.twist.angular}

        deltaT = 0
        if(self.last_time is not None):
            deltaT = time - self.last_time # type: ignore
            deltaT = deltaT * 10**-9 # nano sec to sec
            updated_vector = self.compute_boid_vector(deltaT,self.boids.keys(), obstacle, goal)
            if updated_vector is not None:        # <-- this guard
                update = Twist()
                if self.is_turning: # update x and yaw if turning
                    update.linear.x = updated_vector[0] # type: ignore
                    update.linear.y = 0.0
                    update.linear.z = 0.0
                    update.angular.x = 0.0
                    update.angular.y = 0.0
                    update.angular.z = updated_vector[1] # type: ignore
                else: # update linear x and y
                    update.linear.x = updated_vector[0] # type: ignore
                    update.linear.y = updated_vector[1] # type: ignore
                    update.linear.z = 0.0
                    update.angular.x = 0.0
                    update.angular.y = 0.0
                    update.angular.z = 0.0
                self.newboid.publish(update)
        self.last_time = time

        return


    def compute_boid_vector(self, deltaT, neighborhoodIDs, obstacle = None, goal = None):
        '''Computes the boid vector for a given boid id and its neighborhoodIDs. Returns np.array [x, y]'''
        if( self.pose is None): return None
        mypose = self.pose
        myq = self.orient 

        # yaw from quaternion rotation
        myyaw = np.arctan2(
            2 * (myq.w * myq.z + myq.x * myq.y), 1 - 2 * (myq.y**2 + myq.z**2))# type: ignore

        current_linear = np.sqrt(self.linear.x**2 + self.linear.y**2)  # type: ignore
        current_angular = self.angular.z # type: ignore

        separation = self.separation_vector(mypose, neighborhoodIDs)
        alignment = self.alignment_vector(neighborhoodIDs)
        cohesion = self.cohesion_vector(mypose, neighborhoodIDs)

        obstical_vector = np.array([0.0, 0.0])
        goal_vector = np.array([0.0, 0.0])
        if obstacle is not None:
            # self.get_logger().info(
            # f"obsticle: {obstacle} <"
            # )
            obstical_vector = self.get_obstacle_vector(mypose, obstacle=obstacle)

        if goal is not None:
            goal_vector = self.get_goal_vector(mypose, goalpose=goal)

        # combine the three vectors with their respective coefficients
        boid_vector = (self.coefs[0] * separation +
                        self.coefs[1] * alignment +
                        self.coefs[2] * cohesion +
                        self.coefs[3] * obstical_vector +
                        self.coefs[4] * goal_vector)

        # Rotate into robot system
        R = np.array([
        [ np.cos(myyaw), np.sin(myyaw)],
        [-np.sin(myyaw), np.cos(myyaw)]
        ])
        boid_vector = R @ boid_vector

        if self.is_turning: # using turning + linear x
            
            #heading / angular
            heading_error = np.arctan2(boid_vector[1], boid_vector[0])
            kp = self.turn_rate
            # angular_acc = pow(abs(heading_error),0.5) * kp * np.sign(heading_error)
            angular_acc = np.sin(heading_error/2)*kp

            #speed my model
            # alignment_factor = max(0.0, np.cos(heading_error))
            # desired_speed = self.max_linear * alignment_factor
            # acc = desired_speed - current_linear
            
            #physical speed model
            force_x = boid_vector[0]
            acc = force_x/self.mass
            
            acc = np.clip(acc, -self.max_acceleration, self.max_acceleration)
            
            new_vel = np.clip(current_linear + acc*deltaT, self.min_linear, self.max_linear) 
            new_ang = np.clip(0*current_angular + angular_acc, -self.max_angular, self.max_angular)
            
            
            

            return [new_vel, new_ang]

        else:
            force = np.linalg.norm(boid_vector)
            current_speed = np.array([self.linear.x, self.linear.y]) # type: ignore
            if force < 1e-9: # small force
                return current_speed
    
            #acceleration model
            acc = force * self.mass
            acc = min(self.max_acceleration, acc) 
            
            #speed calculation
            new_speed_vector = current_speed + (boid_vector / force) * acc * deltaT # new linear speed vector
            new_norm = np.linalg.norm(new_speed_vector)
            if new_norm < 1e-9: # small speed
                return np.array([0.0, 0.0])
            new_speed = min(self.max_linear, new_norm) # check against max speed
            return new_speed * (new_speed_vector / new_norm) # finall speed vector


    def self_state_callback(self, msg:Odometry):
        self.pose = msg.pose.pose.position
        self.orient = msg.pose.pose.orientation
        self.linear = msg.twist.twist.linear
        self.angular = msg.twist.twist.angular
        return


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BoidCalculator_decent())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
