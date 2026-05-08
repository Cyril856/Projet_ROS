import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
from std_srvs.srv import SetBool

class CorridorNavigation(Node):
    def __init__(self):
        super().__init__('corridor_navigation_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan', 
            self.lidar_callback,
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # controle de node
        self.active = False
        self.srv = self.create_service(SetBool, '/activate_corridor', self.handle_activation)

      # Callback du service
    def handle_activation(self, request, response):
        self.active = request.data
        response.success = True
        response.message = f"Corridor node state: {self.active}"
        return response

    def safe_min(self, ranges_slice, default=float('inf')):
        filtered = [x for x in ranges_slice if not math.isinf(x) and not math.isnan(x) and x > 0.0]  # Exclure les 0.0
        return min(filtered) if filtered else default

    def lidar_callback(self, msg):
        if self.active:
            self.get_logger().info("Corridor Node Started")
            regions = {
            'front' : self.safe_min(msg.ranges[345:360] + msg.ranges[0:15]),  # 30° devant
            'fleft' : self.safe_min(msg.ranges[16:75]),                      # 60° à gauche avant
            'left'  : self.safe_min(msg.ranges[76:120]),                     # 45° à gauche
            'right' : self.safe_min(msg.ranges[240:285]),                    # 45° à droite
            'fright': self.safe_min(msg.ranges[286:345])                    # 60° à droite avant  
            }
            self.take_action(regions)

    def take_action(self, regions):
        twist = Twist()
        linear_x = 0.0
        angular_z = 0.0

        state_description = ''

        stop_distance = 0.35
        velo_ang = 0.15
        if regions['front'] > stop_distance and regions['fleft'] > stop_distance and regions['fright'] > stop_distance:
            state_description = 'case 1 - nothing'
            linear_x = 0.1
            angular_z = 0.0
        elif regions['front'] < stop_distance and regions['fleft'] > stop_distance and regions['fright'] > stop_distance:
            state_description = 'case 2 - front'
            linear_x = 0.0
            angular_z = velo_ang
        elif regions['front'] > stop_distance and regions['fleft'] > stop_distance and regions['fright'] < stop_distance:
            state_description = 'case 3 - fright'
            linear_x = 0.0
            angular_z = velo_ang
        elif regions['front'] > stop_distance and regions['fleft'] < stop_distance and regions['fright'] > stop_distance:
            state_description = 'case 4 - fleft'
            linear_x = 0.0
            angular_z = -velo_ang
        elif regions['front'] < stop_distance and regions['fleft'] > stop_distance and regions['fright'] < stop_distance:
            state_description = 'case 5 - front and fright'
            linear_x = 0.0
            angular_z = velo_ang
        elif regions['front'] < stop_distance and regions['fleft'] < stop_distance and regions['fright'] > stop_distance:
            state_description = 'case 6 - front and fleft'
            linear_x = 0.0
            angular_z = -velo_ang
        elif regions['front'] < stop_distance and regions['fleft'] < stop_distance and regions['fright'] < stop_distance:
            state_description = 'case 7 - front and fleft and fright'
            self.get_logger().info(f"Regions - front: {regions['front']}, fleft: {regions['fleft']}, fright: {regions['fright']}")
            linear_x = 0.0
            angular_z = velo_ang
            # if regions['fleft'] < regions['fright'] :
            #     state_description = 'case 7.1 - front and fleft priority and fright'
            #     linear_x = 0.0
            #     angular_z = -velo_ang
            # else :
            #     state_description = 'case 7.2 - front and fleft and fright priority'
            #     linear_x = 0.0
            #     angular_z = velo_ang
        elif regions['front'] > stop_distance and regions['fleft'] < stop_distance and regions['fright'] < stop_distance:
            state_description = 'case 8 - fleft and fright'
            self.get_logger().info(f"Regions - front: {regions['front']}")
            linear_x = 0.15
            angular_z = 0.0
        else:
            state_description = 'unknown case'
            self.get_logger().info(f"Regions: {regions}")

        self.get_logger().info(state_description)
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CorridorNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()