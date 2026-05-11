import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import datetime
from rclpy.parameter import Parameter

class ProjectSequencer(Node):
    def __init__(self):
        super().__init__('project_sequencer')
        
        # Création des clients pour chaque défi(à part challenge mediapipe) : pas de transition auto entre goal et mediapipe
        self.client_linefollow = self.create_client(SetBool, '/activate_linefollow') ## ajouter service dans la node
        self.client_obstacleavoidance = self.create_client(SetBool, '/activate_obstacleavoidance')
        self.client_corridor = self.create_client(SetBool, '/activate_corridor')
        self.client_goal = self.create_client(SetBool, '/activate_goal') ## ajouter service dans la node
        self.client_handteleop = self.create_client(SetBool, '/activate_handteleop')
        self.client_blueline = self.create_client(SetBool, '/activate_blueline')
        
        self.blueline = False
        self.time_reset = datetime.datetime.now()

        self.current_stage = 0  # Compteur de progression
        #self.declare_parameter('current_stage', 0) # paramètre pour âtre accessible depuis le terminal : à tester !!
        
        #self.check_logic() # 1er appel ## cas ou on a la détection de ligne

        # Timer pour vérifier l'état du projet toutes les secondes
        self.timer = self.create_timer(1, self.check_logic) ## à commenter lorsqu'on aura la détection de ligne
        ## checker si un problème peut survenir s'il est appelé plusieurs fois pdt que la node est activée

        # detection de ligne bleue
        self.blueline_active = False
        self.srv = self.create_service(SetBool, '/blueline_status', self.handle_blueline)

      # Callback du service
    def handle_blueline(self, request, response):
        self.blueline_active = request.data
        response.success = True
        response.message = f"Changement de challenge state: {self.blueline_active}"
        self.upgrade_stage()
        return response

    def call_service(self, client, state):
        if not client.wait_for_service(timeout_sec=1.0):
            return
        req = SetBool.Request()
        req.data = state
        client.call_async(req)

    def upgrade_stage(self) :
        current_stage = self.get_parameter('current_stage').value
        
        # Decision
        dernier_appel = (datetime.datetime.now() - self.time_reset).total_seconds()

        self.get_logger().info(f"Le dernier_appel date d'il y a : {dernier_appel} sec")

        if dernier_appel > 3.0 and current_stage <= 5 : ## à calibrer, pas sur qu'il fonctionne
            current_stage+=1
            self.time_reset = datetime.datetime.now()
            self.set_parameters([Parameter('current_stage', Parameter.Type.INTEGER, current_stage)])
            self.check_logic()

        else : # "pas de changement"
            return

    def check_logic(self):
        # Exemple de logique basée sur un compteur ou une condition
        current_stage = self.get_parameter('current_stage').value

        if current_stage == 1:
            self.call_service(self.client_linefollow, True)
            self.call_service(self.client_blueline, True)

        elif current_stage == 2:
            self.call_service(self.client_linefollow, False)
            self.call_service(self.client_obstacleavoidance, True)
            
        elif current_stage == 3:
            self.call_service(self.client_obstacleavoidance, False)
            self.call_service(self.client_corridor, True)
        
        elif current_stage == 4:
            self.call_service(self.client_corridor, False)
            self.call_service(self.client_linefollow, True)

        elif current_stage == 5:
            self.call_service(self.client_linefollow, False)
            self.call_service(self.client_blueline, False)

        # A lancer à la main
        elif current_stage == 6:    
            self.call_service(self.client_goal, True) ## ajouter  une condition d'arrêt ?? dès qu'il passe les poteaux ?

        elif current_stage == 7:
            self.call_service(self.client_goal, False)
            self.call_service(self.client_handteleop, True)

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
        rclpy.shutdown() # le seul à l'avoir car il gère tout, dans le launch, son arrêt termine le programme de tt facon

if __name__ == '__main__':
    main()