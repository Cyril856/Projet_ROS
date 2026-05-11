import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import threading

class HSVCalibration(Node):
    def __init__(self):
        super().__init__('hsv_calibration')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed', #/camera
            self.listener_callback,
            10
        )

        self.image = None
        self.horizon_y = 130

        # Windows
        cv2.namedWindow("Original")
        cv2.namedWindow("Mask")
        cv2.namedWindow("HSV Calibration")

        # Red
        cv2.createTrackbar("Red H min1", "HSV Calibration", 0,   179, self.nothing)
        cv2.createTrackbar("Red H max1", "HSV Calibration", 10,  179, self.nothing)
        cv2.createTrackbar("Red H min2", "HSV Calibration", 160, 179, self.nothing)
        cv2.createTrackbar("Red H max2", "HSV Calibration", 179, 179, self.nothing)
        cv2.createTrackbar("Red S min",  "HSV Calibration", 20,  255, self.nothing)
        cv2.createTrackbar("Red S max",  "HSV Calibration", 255, 255, self.nothing)
        cv2.createTrackbar("Red V min",  "HSV Calibration", 140, 255, self.nothing)
        cv2.createTrackbar("Red V max",  "HSV Calibration", 255, 255, self.nothing)

        # Green
        cv2.createTrackbar("Green H min", "HSV Calibration", 40,  179, self.nothing)
        cv2.createTrackbar("Green H max", "HSV Calibration", 80,  179, self.nothing)
        cv2.createTrackbar("Green S min", "HSV Calibration", 20,  255, self.nothing)
        cv2.createTrackbar("Green S max", "HSV Calibration", 255, 255, self.nothing)
        cv2.createTrackbar("Green V min", "HSV Calibration", 100, 255, self.nothing)
        cv2.createTrackbar("Green V max", "HSV Calibration", 255, 255, self.nothing)

        # Yellow
        cv2.createTrackbar("Yellow H min", "HSV Calibration", 20,  179, self.nothing)
        cv2.createTrackbar("Yellow H max", "HSV Calibration", 35,  179, self.nothing)
        cv2.createTrackbar("Yellow S min", "HSV Calibration", 100, 255, self.nothing)
        cv2.createTrackbar("Yellow S max", "HSV Calibration", 255, 255, self.nothing)
        cv2.createTrackbar("Yellow V min", "HSV Calibration", 100, 255, self.nothing)
        cv2.createTrackbar("Yellow V max", "HSV Calibration", 255, 255, self.nothing)

        # Blue
        cv2.createTrackbar("Blue H min", "HSV Calibration", 100, 179, self.nothing)
        cv2.createTrackbar("Blue H max", "HSV Calibration", 130, 179, self.nothing)
        cv2.createTrackbar("Blue S min", "HSV Calibration", 150, 255, self.nothing)
        cv2.createTrackbar("Blue V min", "HSV Calibration", 50,  255, self.nothing)

    def nothing(self, x):
        pass

    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is not None:
            self.image = image.copy()

    def get_trackbar_values(self):
        values = {
            'red': [
                cv2.getTrackbarPos("Red H min1", "HSV Calibration"),
                cv2.getTrackbarPos("Red H max1", "HSV Calibration"),
                cv2.getTrackbarPos("Red H min2", "HSV Calibration"),
                cv2.getTrackbarPos("Red H max2", "HSV Calibration"),
                cv2.getTrackbarPos("Red S min",  "HSV Calibration"),
                cv2.getTrackbarPos("Red S max",  "HSV Calibration"),
                cv2.getTrackbarPos("Red V min",  "HSV Calibration"),
                cv2.getTrackbarPos("Red V max",  "HSV Calibration")
            ],
            'green': [
                cv2.getTrackbarPos("Green H min", "HSV Calibration"),
                cv2.getTrackbarPos("Green H max", "HSV Calibration"),
                cv2.getTrackbarPos("Green S min", "HSV Calibration"),
                cv2.getTrackbarPos("Green S max", "HSV Calibration"),
                cv2.getTrackbarPos("Green V min", "HSV Calibration"),
                cv2.getTrackbarPos("Green V max", "HSV Calibration")
            ],
            'yellow': [
                cv2.getTrackbarPos("Yellow H min", "HSV Calibration"),
                cv2.getTrackbarPos("Yellow H max", "HSV Calibration"),
                cv2.getTrackbarPos("Yellow S min", "HSV Calibration"),
                cv2.getTrackbarPos("Yellow S max", "HSV Calibration"),
                cv2.getTrackbarPos("Yellow V min", "HSV Calibration"),
                cv2.getTrackbarPos("Yellow V max", "HSV Calibration")
            ],
            'blue' : [
                cv2.getTrackbarPos("Blue H min", "HSV Calibration"),
                cv2.getTrackbarPos("Blue H max", "HSV Calibration"),
                cv2.getTrackbarPos("Blue S min", "HSV Calibration"),
                cv2.getTrackbarPos("Blue V min", "HSV Calibration")

            ]
        }
        return values

    def print_values(self, v):
        # On récupère les positions des trackbars
        r_h1_min = cv2.getTrackbarPos("R_H1_min", "HSV Calibration")
        r_h1_max = cv2.getTrackbarPos("R_H1_max", "HSV Calibration")
        r_h2_min = cv2.getTrackbarPos("R_H2_min", "HSV Calibration")
        r_h2_max = cv2.getTrackbarPos("R_H2_max", "HSV Calibration")
        r_s_min  = cv2.getTrackbarPos("R_S_min",  "HSV Calibration")
        r_v_min  = cv2.getTrackbarPos("R_V_min",  "HSV Calibration")

        g_h_min  = cv2.getTrackbarPos("G_H_min", "HSV Calibration")
        g_h_max  = cv2.getTrackbarPos("G_H_max", "HSV Calibration")
        g_s_min  = cv2.getTrackbarPos("G_S_min", "HSV Calibration")
        g_v_min  = cv2.getTrackbarPos("G_V_min", "HSV Calibration")

        y_h_min  = cv2.getTrackbarPos("Y_H_min", "HSV Calibration")
        y_h_max  = cv2.getTrackbarPos("Y_H_max", "HSV Calibration")
        y_s_min  = cv2.getTrackbarPos("Y_S_min", "HSV Calibration")
        y_v_min  = cv2.getTrackbarPos("Y_V_min", "HSV Calibration")

        b_h_min = cv2.getTrackbarPos("Blue H min", "HSV Calibration")
        b_h_max = cv2.getTrackbarPos("Blue H max", "HSV Calibration")
        b_s_min = cv2.getTrackbarPos("Blue S min", "HSV Calibration")
        b_v_min = cv2.getTrackbarPos("Blue V min", "HSV Calibration")

        # Le texte formaté pour ton code
        output = f"""
############################################################
# COPIER-COLLER DANS TON NODE (INIT)
############################################################
self.horizon = {self.horizon_y}
self.lower_red1 = np.array([{r_h1_min}, {r_s_min}, {r_v_min}])
self.upper_red1 = np.array([{r_h1_max}, 255, 255])
self.lower_red2 = np.array([{r_h2_min}, {r_s_min}, {r_v_min}])
self.upper_red2 = np.array([{r_h2_max}, 255, 255])

self.lower_green = np.array([{g_h_min}, {g_s_min}, {g_v_min}])
self.upper_green = np.array([{g_h_max}, 255, 255])

self.lower_yellow = np.array([{y_h_min}, {y_s_min}, {y_v_min}])
self.upper_yellow = np.array([{y_h_max}, 255, 255])

self.lower_blue = np.array([{b_h_min}, {b_s_min}, {b_v_min}])
self.upper_blue = np.array([{b_h_max}, 255, 255])
############################################################
"""
        # flush=True force l'apparition dans le terminal immédiatement
        print(output, flush=True)

    def run(self):
        self.get_logger().info("Calibration active. CLIQUEZ SUR LA FENETRE IMAGE puis appuyez sur 'P'")
        rate = self.create_rate(20)

        while rclpy.ok():
            if self.image is not None:
                image = self.image.copy()
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                v = self.get_trackbar_values()

                # --- Masques ---
                m_r1 = cv2.inRange(hsv, np.array([v['red'][0], v['red'][4], v['red'][6]]), np.array([v['red'][1], v['red'][5], v['red'][7]]))
                m_r2 = cv2.inRange(hsv, np.array([v['red'][2], v['red'][4], v['red'][6]]), np.array([v['red'][3], v['red'][5], v['red'][7]]))
                mask_red = m_r1 | m_r2
                mask_green = cv2.inRange(hsv, np.array([v['green'][0], v['green'][2], v['green'][4]]), np.array([v['green'][1], v['green'][3], v['green'][5]]))
                mask_yellow = cv2.inRange(hsv, np.array([v['yellow'][0], v['yellow'][2], v['yellow'][4]]), np.array([v['yellow'][1], v['yellow'][3], v['yellow'][5]]))

                # Bleu (on récupère les valeurs directement)
                b_h_min = cv2.getTrackbarPos("Blue H min", "HSV Calibration")
                b_h_max = cv2.getTrackbarPos("Blue H max", "HSV Calibration")
                b_s_min = cv2.getTrackbarPos("Blue S min", "HSV Calibration")
                b_v_min = cv2.getTrackbarPos("Blue V min", "HSV Calibration")
                mask_blue = cv2.inRange(hsv, np.array([b_h_min, b_s_min, b_v_min]), np.array([b_h_max, 255, 255]))

                # Visualisation bitwise_or
                res_rg = cv2.bitwise_or(mask_red, mask_green)
                res_yb = cv2.bitwise_or(mask_yellow, mask_blue)
                result = cv2.bitwise_or(res_rg, res_yb)
                
                # Dessin horizon
                cv2.line(image, (0, self.horizon_y), (image.shape[1], self.horizon_y), (255, 0, 0), 2)

                cv2.imshow("Original", image)
                cv2.imshow("Mask", result)

            # --- GESTION DU CLAVIER (C'est ici que ça se joue) ---
            key = cv2.waitKey(1) & 0xFF
            
            if key != 255: # Si une touche est pressée (255 = rien)
                self.get_logger().info(f"Touche détectée : {key} (caractère : {chr(key) if key < 256 else '?'})")

            if key == ord('p') or key == ord('P'):
                self.print_values(v)
            elif key == ord('q') or key == ord('Q'):
                self.get_logger().info("Fermeture...")
                break

            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    node = HSVCalibration()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()