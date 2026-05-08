import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import datetime
from sensor_msgs.msg import CompressedImage

class ProjectSequencer(Node):
    def __init__(self):
        super().__init__('project_sequencer')
        
        # Création des clients pour chaque défi(à part challenge 5)
        self.client_linefollowing = self.create_client(SetBool, '/activate_linefollowing') ## ajouter service dans la node
        self.client_obstacleavoidance = self.create_client(SetBool, '/activate_obstacleavoidance')
        self.client_corridor = self.create_client(SetBool, '/activate_corridor')
        self.client_goal = self.create_client(SetBool, '/activate_goal') ## ajouter service dans la node
        
        ## checker si un problème peut survenir s'il est appelé plusieurs fois pdt que l'autre node est activée
        self.camera_sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.listener_callback,
            10
        )
        self.blueline = False
        self.time_reset = datetime.datetime.now()

        #self.current_stage = 0  # Compteur de progression
        self.declare_parameter('current_stage', 0) # paramètre pour âtre accessible depuis le terminal : à tester !!
        
        self.check_logic() # 1er appel

        # Timer pour vérifier l'état du projet toutes les secondes
        ## checker si un problème peut survenir s'il est appelé plusieurs fois pdt que la node est activée

    def call_service(self, client, state):
        if not client.wait_for_service(timeout_sec=1.0):
            return
        req = SetBool.Request()
        req.data = state
        client.call_async(req)

    def listener_callback(self) :
        current_stage = self.get_parameter('current_stage').value
        
        ## blueline_detection
        
        #if bluelinedetect : self.blueline = True
        #else : self.blueline = False
        #

        # Decision
        self.time_now = datetime.datetime.now()

        dernier_appel = self.time_reset-self.time_now

        self.get_logger().info(f"Le dernier_appel date d'il y a : {dernier_appel} sec")

        if self.blueline and  dernier_appel > 3 : ## à calibrer, pas sur qu'il fonctionne
            current_stage+=1
            self.time_reset = datetime.datetime.now()
            self.set_parameter('current_stage', current_stage)
            self.check_logic()

        else : # "pas de changement"
            return

    def check_logic(self):
        # Exemple de logique basée sur un compteur ou une condition
        current_stage = self.get_parameter('current_stage').value

        if current_stage == 0:
            self.call_service(self.client_linefollowing, True)

        elif current_stage == 1:
            self.call_service(self.client_linefollowing, False)
            self.call_service(self.client_obstacleavoidance, True)
            
        elif current_stage == 2:
            self.call_service(self.client_obstacleavoidance, False)
            self.call_service(self.client_corridor, True)
        
        elif current_stage == 3:
            self.call_service(self.client_corridor, False)
            self.call_service(self.client_goal, True) ## ajouter  une condition d'arrêt ??

        else : 
            self.get_logger().warn(f"état de challenge impossible : {current_stage}, recommencer")

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