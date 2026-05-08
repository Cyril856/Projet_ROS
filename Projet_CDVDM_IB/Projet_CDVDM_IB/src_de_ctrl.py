import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class ProjectSequencer(Node):
    def __init__(self):
        super().__init__('project_sequencer')
        
        # Création des clients pour chaque défi [cite: 9, 152]
        self.client_obstacleavoidance = self.create_client(SetBool, '/activate_obstacleavoidance')
        self.client_corridor = self.create_client(SetBool, '/activate_corridor')
        
        #self.current_stage = 0  # Compteur de progression
        self.declare_parameter('current_stage', 0)
        # Timer pour vérifier l'état du projet toutes les secondes
        self.timer = self.create_timer(1.0, self.check_logic)

    def call_service(self, client, state):
        if not client.wait_for_service(timeout_sec=1.0):
            return
        req = SetBool.Request()
        req.data = state
        client.call_async(req)

    def check_logic(self):
        # Exemple de logique basée sur un compteur ou une condition
        current_stage = self.get_parameter('current_stage').value
        if current_stage == 0:
            self.call_service(self.client_obstacleavoidance, True)
            self.call_service(self.client_corridor, False)
            # Si une condition de fin de ligne est remplie, passer à l'étape suivante
            # self.current_stage += 1
            
        elif current_stage == 1:
            self.call_service(self.client_obstacleavoidance, False)
            self.call_service(self.client_corridor, True)

def main(args=None):
    rclpy.init(args=args)
    node = ProjectSequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()