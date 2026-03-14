import cv2
import mediapipe as mp
import numpy as np
import math
import time
import serial
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

L1 = 4.0
L2 = 12.0
X_MIN, X_MAX = -16, 16
Y_MIN, Y_MAX = 0, 16

COM_PORT = "COM9"
BAUD_RATE = 115200
SEND_INTERVAL = 0.03  
ANGLE_THRESHOLD = 0.5 


def inverse_kinematics(x, y):
    if y < 0: return None
    r2 = x*x + y*y
    r = math.sqrt(r2)
    if r > L1 + L2 or r < abs(L2 - L1): return None

    cos_t2 = (r2 - L1*L1 - L2*L2) / (2 * L1 * L2)
    cos_t2 = max(-1, min(1, cos_t2))
    sin_t2 = math.sqrt(1 - cos_t2*cos_t2)

    t2 = math.atan2(sin_t2, cos_t2)
    k1, k2 = L1 + L2 * math.cos(t2), L2 * math.sin(t2)
    t1 = math.atan2(y, x) - math.atan2(k2, k1)
    
    if 0 <= math.degrees(t1) <= 180 and -math.pi/2 <= t2 <= math.pi/2:
        return t1, t2
    return None


class VisionArmApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Vision-Controlled Planar Arm")
        
        try:
            self.ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0, write_timeout=0)
            time.sleep(1)
        except:
            self.ser = None
            print("Serial not connected.")

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7)
        self.cap = cv2.VideoCapture(0)

        self.last_send_time = 0
        self.last_angles = (0, 0)

        # UI Layout
        self.info = tk.Label(root, text="Finger Tracking Active", font=("Consolas", 11), pady=10)
        self.info.pack()

        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_aspect("equal")
        self.ax.set_xlim(X_MIN, X_MAX)
        self.ax.set_ylim(Y_MIN - 1, Y_MAX + 1)
        
        self.draw_workspace_plot()
        self.arm_line, = self.ax.plot([], [], lw=5, color="#2c7be5", solid_capstyle='round')
        self.arm_joints = self.ax.scatter([], [], s=60, color="#333", zorder=5)

        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

        self.update_loop()

    def draw_workspace_plot(self):
        xs = np.linspace(X_MIN, X_MAX, 40)
        ys = np.linspace(0, Y_MAX, 40)
        pts_x, pts_y = [], []
        for x in xs:
            for y in ys:
                if inverse_kinematics(x, y):
                    pts_x.append(x); pts_y.append(y)
        self.ax.scatter(pts_x, pts_y, s=1, color="#ddd")

    def draw_webcam_grid(self, frame):
        """Draws the coordinate grid overlay on the camera frame."""
        h, w, _ = frame.shape
        # vertical grid lines (X)
        for gx in range(X_MIN, X_MAX + 1, 4):
            # Map robot X to normalized 0-1, then to pixels
            norm_x = np.interp(gx, [X_MIN, X_MAX], [0, 1])
            px = int(norm_x * w)
            cv2.line(frame, (px, 0), (px, h), (100, 100, 100), 1)
            cv2.putText(frame, str(gx), (px + 5, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        # horizontal grid lines (Y)
        for gy in range(Y_MIN, Y_MAX + 1, 4):
            # Map robot Y to normalized 0-1, then to pixels
            norm_y = np.interp(gy, [Y_MIN, Y_MAX], [1, 0])
            py = int(norm_y * h)
            cv2.line(frame, (0, py), (w, py), (100, 100, 100), 1)
            cv2.putText(frame, f"Y:{gy}", (10, py - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    def update_loop(self):
        ret, frame = self.cap.read()
        if not ret: return

        frame = cv2.flip(frame, 1) 
        h, w, c = frame.shape
        
        # 1. Reference Grid on Frame
        self.draw_webcam_grid(frame)

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)

        if results.multi_hand_landmarks:
            for hand_lms in results.multi_hand_landmarks:
                tip = hand_lms.landmark[8] # Index tip
                
                target_x = np.interp(tip.x, [0, 1], [X_MIN, X_MAX])
                target_y = np.interp(tip.y, [0, 1], [Y_MAX, Y_MIN]) 

                res = inverse_kinematics(target_x, target_y)
                if res:
                    t1, t2 = res
                    t1_deg = 180 - math.degrees(t1)
                    t2_deg = 90 - math.degrees(t2)

                    self.update_arm_viz(t1, t2, target_x, target_y, t1_deg, t2_deg)

                    now = time.time()
                    if now - self.last_send_time > SEND_INTERVAL:
                        if abs(t1_deg - self.last_angles[0]) > ANGLE_THRESHOLD:
                            self.send_to_esp(t1_deg, t2_deg)
                            self.last_send_time = now
                            self.last_angles = (t1_deg, t2_deg)
                    
                    color = (0, 255, 0) # Green for valid
                else:
                    color = (0, 0, 255) # Red for out of bounds

                #finger tip tracking circle
                cv2.circle(frame, (int(tip.x * w), int(tip.y * h)), 8, color, -1)

        cv2.imshow("Arm Control Feed", frame)
        cv2.waitKey(1)
        self.root.after(10, self.update_loop)

    def update_arm_viz(self, t1, t2, x, y, d1, d2):
        x1, y1 = L1 * math.cos(t1), L1 * math.sin(t1)
        x2, y2 = x1 + L2 * math.cos(t1 + t2), y1 + L2 * math.sin(t1 + t2)
        self.arm_line.set_data([0, x1, x2], [0, y1, y2])
        self.arm_joints.set_offsets(np.c_[[0, x1, x2], [0, y1, y2]])
        self.info.config(text=f"X={x:.1f} Y={y:.1f} | θ1={d1:.1f}° θ2={d2:.1f}°")
        self.canvas.draw_idle()

    def send_to_esp(self, t1, t2):
        if self.ser and self.ser.is_open:
            cmd = f"M3 {t2:.1f}; M2 {t1:.1f};\n"
            try:
                self.ser.write(cmd.encode("ascii"))
                self.ser.flush()
            except: pass

    def on_close(self):
        self.cap.release()
        cv2.destroyAllWindows()
        if self.ser: self.ser.close()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = VisionArmApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()