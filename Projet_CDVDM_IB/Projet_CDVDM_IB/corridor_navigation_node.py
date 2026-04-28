import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

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
        self.get_logger().info("Corridor Navigation Node Started")

    def safe_min(self, ranges_slice, default=10):
        filtered = [x for x in ranges_slice if not math.isinf(x) and not math.isnan(x)]
        return min(filtered) if filtered else default

    def lidar_callback(self, msg):
        regions = {
        # Calculer la distance minimale pour chaque direction
        'front' : self.safe_min(msg.ranges[340:360] + msg.ranges[0:20]),  
        'fleft' : self.safe_min(msg.ranges[21:61]), 
        'left' : self.safe_min(msg.ranges[62:102]),
        'right' : self.safe_min(msg.ranges[258:298]),
        'fright' : self.safe_min(msg.ranges[299:339]),    
        }
        self.take_action(regions)

    def take_action(self, regions):
        twist = Twist()
        linear_x = 0.0
        angular_z = 0.0

        state_description = ''

        stop_distance = 0.15
        velo_ang = 0.15
        if regions['front'] > stop_distance and regions['fleft'] > stop_distance and regions['fright'] > stop_distance:
            state_description = 'case 1 - nothing'
            linear_x = 0.2
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
            linear_x = 0.0
            angular_z = -velo_ang
        elif regions['front'] > stop_distance and regions['fleft'] < stop_distance and regions['fright'] < stop_distance:
            state_description = 'case 8 - fleft and fright'
            linear_x = 0.15
            angular_z = 0.0
        else:
            state_description = 'unknown case'
            self.get_logger().info(regions)

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