import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
from std_srvs.srv import SetBool

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Paramètres ---
        self.stop_distance   = 0.3   # Distance de détection d'une bouteille (m)
        self.linear_speed    = 0.05   # Vitesse linéaire faible (m/s)
        self.angular_speed   = 0.3    # Vitesse angulaire d'évitement (rad/s)

        # --- Machine à états ---
        # 'straight'   : ligne droite, pas d'obstacle
        # 'avoiding'   : rotation en cours pour éviter la bouteille
        # 'returning'  : rotation inverse pour se recadrer
        self.state           = 'straight'
        self.avoid_direction = 0      # +1 = tourne gauche, -1 = tourne droite
        self.avoid_steps     = 0      # Nombre de callbacks en rotation d'évitement
        self.return_steps    = 0      # Compteur de retour (même durée)

        # controle de node
        self.active = False
        self.srv = self.create_service(SetBool, '/activate_obstacleavoidance', self.handle_activation)

    # Callback du service
    def handle_activation(self, request, response):
        self.active = request.data
        response.success = True
        response.message = f"Obstacle avoidance node state: {self.active}"
        return response

    def safe_min(self, ranges_slice, default=float('inf')):
        filtered = [x for x in ranges_slice if not math.isinf(x) and not math.isnan(x) and x > 0.0]
        return min(filtered) if filtered else default

    def lidar_callback(self, msg):
        if not self.active:
            return  # Ignore les données LiDAR si la node est inactive
        regions = {
            'front'  : self.safe_min(msg.ranges[350:360] + msg.ranges[0:10]),
            'fleft'  : self.safe_min(msg.ranges[11:55]),
            'left'   : self.safe_min(msg.ranges[56:120]),
            'right'  : self.safe_min(msg.ranges[240:305]),
            'fright' : self.safe_min(msg.ranges[306:349]),
        }
        self.take_action(regions)

    def take_action(self, regions):
        twist = Twist()
        twist.linear.x = self.linear_speed  # Toujours une légère avance

        # ------------------------------------------------------------------ #
        #  ÉTAT : straight — on scrute fleft et fright                        #
        # ------------------------------------------------------------------ #
        if self.state == 'straight':
            bottle_left  = regions['fleft']  < self.stop_distance
            bottle_right = regions['fright'] < self.stop_distance

            if bottle_left and not bottle_right:
                # Bouteille à gauche → tourner à droite (angular négatif)
                self.avoid_direction = -1
                self.state           = 'avoiding'
                self.avoid_steps     = 0
                self.get_logger().info("Bouteille à GAUCHE → rotation droite")

            elif bottle_right and not bottle_left:
                # Bouteille à droite → tourner à gauche (angular positif)
                self.avoid_direction = +1
                self.state           = 'avoiding'
                self.avoid_steps     = 0
                self.get_logger().info("Bouteille à DROITE → rotation gauche")

            elif bottle_left and bottle_right:
                # Les deux côtés bloqués : priorité au plus proche
                if regions['fleft'] < regions['fright']:
                    self.avoid_direction = -1   # plus proche à gauche → droite
                else:
                    self.avoid_direction = +1   # plus proche à droite → gauche
                self.state       = 'avoiding'
                self.avoid_steps = 0
                self.get_logger().info("Bouteilles des DEUX côtés → priorité au plus proche")

            else:
                # Voie libre
                twist.angular.z = 0.0
                self.get_logger().info("Ligne droite - aucun obstacle")

        # ------------------------------------------------------------------ #
        #  ÉTAT : avoiding — on tourne jusqu'à ce que la bouteille disparaisse #
        # ------------------------------------------------------------------ #
        elif self.state == 'avoiding':
            bottle_in_danger_zone = (
                regions['fleft']  < self.stop_distance or
                regions['fright'] < self.stop_distance
            )

            if bottle_in_danger_zone:
                # Continuer à tourner et compter les pas
                twist.angular.z = self.avoid_direction * self.angular_speed
                self.avoid_steps += 1
                self.get_logger().info(
                    f"Évitement en cours (pas={self.avoid_steps}, "
                    f"fleft={regions['fleft']:.2f}, fright={regions['fright']:.2f})"
                )
            else:
                # Bouteille hors du champ → passer en retour
                self.return_steps = self.avoid_steps   # mémorise la durée d'évitement
                self.state        = 'returning'
                twist.angular.z   = -self.avoid_direction * self.angular_speed
                self.get_logger().info(
                    f"Bouteille dégagée → retour ({self.return_steps} pas)"
                )

        # ------------------------------------------------------------------ #
        #  ÉTAT : returning — on tourne en sens inverse le même nombre de pas #
        # ------------------------------------------------------------------ #
        elif self.state == 'returning':
            if self.return_steps > 0:
                twist.angular.z    = -self.avoid_direction * self.angular_speed
                self.return_steps -= 1
                self.get_logger().info(f"Retour cap initial (restant={self.return_steps})")
            else:
                # Retour terminé → ligne droite
                twist.angular.z = 0.0
                self.state      = 'straight'
                self.get_logger().info("Cap initial retrouvé → ligne droite")

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt du nœud")
    finally:
        node.destroy_node()
        #rclpy.shutdown() # ne pas activer sinon risque de shutdown toutes les nodes

if __name__ == '__main__':
    main()