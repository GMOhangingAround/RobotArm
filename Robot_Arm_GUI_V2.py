import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QSlider, QLabel, QPushButton
from PyQt5.QtCore import Qt
import numpy as np
import sys
import time
import serial



class RobotArmGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Arm Control Panel")
        self.setGeometry(400, 400, 1600, 1200)  # x, y, width, height


        # Set current servo angles
        self.servo_angles = [90, 90, 90, 90, 90]  # Initial angles for 5 servos

        # Set robot dimensions
        self.base_height = 5
        self.arm1_length = 5
        self.arm2_length = 5
        self.gripper_length = 5 

        # Set cube parameters
        self.cube_pos = np.array([15.0, -5.0, 0.0])  
        self.cube_size = 3.0
        self.holding = False
        self.grab_distance = 10.0

        self.setup_layout()

    def setup_layout(self):

        widget = QWidget()
        self.setCentralWidget(widget)

        main_layout = QHBoxLayout(widget)

        left_panel = self.create_left_panel()
        main_layout.addWidget(left_panel)

        right_panel = self.create_right_panel()
        main_layout.addWidget(right_panel, stretch=2)  # Make right panel take more space


        self.ser = None # serial.Serial('COM3', baudrate =115200 , timeout=1 )
        self.last_command_time = 0
        self.command_interval = 100  # milliseconds


        self.draw_robot_arm()


    def create_left_panel(self):
        left_panel = QWidget()
        left_panel.setStyleSheet("background-color: lightgray;")

        layout = QVBoxLayout(left_panel)

        left_label = QLabel("Control Panel")
        layout.addWidget(left_label)

        layout.setSpacing(40)
        layout.setContentsMargins(10, 10, 10, 10)

        servos = [
            ("Base Rotation", 0, 180, 90),
            ("Swing", 0, 180, 90),
            ("Arm 1", 0, 180, 90),
            ("Arm 2", 0, 180, 90),
            ("Gripper", 0, 180, 90),
        ]

        self.sliders = []
        self.labels = []

        for i, (name, min_val, max_val, init_val) in enumerate(servos):
            label = QLabel(f"{name}: {init_val}°")
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(min_val)
            slider.setMaximum(max_val)
            slider.setValue(init_val)
            slider.valueChanged.connect(lambda value, idx=i: self.update_servo_angle(idx, value))

            layout.addWidget(label)
            layout.addWidget(slider)

            self.sliders.append(slider)
            self.labels.append(label)


        btn_row = QHBoxLayout()
        pick_button = QPushButton("Pick")
        place_button = QPushButton("Place")
        pick_button.clicked.connect(self.pick_object)
        place_button.clicked.connect(self.place_object)
        btn_row.addWidget(pick_button)
        btn_row.addWidget(place_button)
        layout.addLayout(btn_row)

        layout.addStretch(1)

        return left_panel
    

    def update_servo_angle(self, index, value):
        self.servo_angles[index] = value
        servo_names = ["Base Rotation", "Swing", "Arm 1", "Arm 2", "Gripper"]
        self.labels[index].setText(f"{servo_names[index]}: {value}°")
        self.draw_robot_arm()
        
        self.send_servo_commands(index, value) # Send command to hardware


    def create_right_panel(self):
        right_panel = QWidget()
        right_panel.setStyleSheet("background-color: white;")

        layout = QVBoxLayout(right_panel) 

        self.fig = plt.Figure(figsize=(12, 9))
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111, projection='3d')



        (self.arm_line,) = self.ax.plot([], [], [],'o-', markersize=10, markerfacecolor='red', color='blue')

        self.end_point = self.ax.scatter([], [], [], s=100, c='green')
       
        self.cube_edges = [self.ax.plot([], [], [], color='red', lw=2, alpha=0.9)[0] for _ in range(12)]
    
        layout.addWidget(self.canvas)

        return right_panel
    
    def cube_vertices(self, center, size):
        x, y, z = center
        s2 = size / 2.0
        # 8 vertices
        v = np.array([
            [x-s2, y-s2, z-s2], [x+s2, y-s2, z-s2],
            [x+s2, y+s2, z-s2], [x-s2, y+s2, z-s2],
            [x-s2, y-s2, z+s2], [x+s2, y-s2, z+s2],
            [x+s2, y+s2, z+s2], [x-s2, y+s2, z+s2],
        ])
        return v

    def cube_edges_idx(self):
        # 12 edges as pairs of vertex indices to connect
        return [
            (0,1),(1,2),(2,3),(3,0),   # bottom
            (4,5),(5,6),(6,7),(7,4),   # top
            (0,4),(1,5),(2,6),(3,7)    # verticals
        ]


    def update_cube(self):
        v = self.cube_vertices(self.cube_pos, self.cube_size)
        edges = self.cube_edges_idx()
        for line, (i,j) in zip(self.cube_edges, edges):
            xi, yi, zi = v[i]
            xj, yj, zj = v[j]
            line.set_data_3d([xi, xj], [yi, yj], [zi, zj])



    def pick_object(self):
        if self.holding:
            return  # Already holding an object

        _, _, _, _, gripper = self.forward_kinematics(self.servo_angles)
        gripper_pos = gripper

        distance = np.linalg.norm(np.array(gripper_pos) - self.cube_pos)
        if distance <= self.grab_distance:
            self.holding = True
            self.cube_pos = np.array(gripper_pos)  # Move cube to gripper position
            print("Object picked up!")
            self.draw_robot_arm()
        else:
            print(f"Object is out of reach. Distance: {distance:.2f} > {self.grab_distance}")

    def place_object(self):
        if not self.holding:
            return  # Not holding any object

        self.holding = False

        x, y, _ = self.cube_pos
        self.cube_pos = np.array([x, y, 0.0])  # Place cube down on the ground
        print("Object placed down.")
        self.draw_robot_arm()


    def forward_kinematics(self, angles):

        base, swing, arm1, arm2, grip = angles
        base_rad = np.radians(base)

        # Vertical positions
        th1 = np.radians(90 - swing)       # Shoulder angle relative to the base
        th2 = th1 + np.radians(90 - arm1)  # Elbow angle
        th3 = th2 + np.radians(90 - arm2)  # Wrist angle

        base_pos = np.array([0, 0, 0])
        shoulder_pos = np.array([0, 0, self.base_height])

        x1 = self.arm1_length * np.cos(th1)
        z1 = self.arm1_length * np.sin(th1)
        e1 = np.array([x1, 0, z1])  # Elbow position in the arm plane

        x2 = self.arm2_length * np.cos(th2)
        z2 = self.arm2_length * np.sin(th2)
        e2 = e1 + np.array([x2, 0, z2])  # Wrist position in the arm plane

        x3 = self.gripper_length * np.cos(th3)
        z3 = self.gripper_length * np.sin(th3)
        e3 = e2 + np.array([x3, 0, z3])  # Gripper position in the arm plane
        # Rotate around base
        rotation_matrix = np.array([
            [np.cos(base_rad), -np.sin(base_rad), 0],
            [np.sin(base_rad),  np.cos(base_rad), 0],
            [0, 0, 1]
        ])

        elbow_pos = shoulder_pos + rotation_matrix.dot(e1)
        wrist_pos = shoulder_pos + rotation_matrix.dot(e2)
        gripper_pos = shoulder_pos + rotation_matrix.dot(e3)
        
        
        return base_pos, shoulder_pos, elbow_pos, wrist_pos, gripper_pos
    
    
    def draw_robot_arm(self):
       
        base, shoulder, elbow, wrist, gripper = self.forward_kinematics(self.servo_angles)

        # Plot the arm segments
        xs = [base[0], shoulder[0], elbow[0], wrist[0], gripper[0]]
        ys = [base[1], shoulder[1], elbow[1], wrist[1], gripper[1]]
        zs = [base[2], shoulder[2], elbow[2], wrist[2], gripper[2]]

        self.arm_line.set_data_3d(xs, ys, zs)
        self.end_point._offsets3d = ([gripper[0]], [gripper[1]], [gripper[2]])

        # Set plot limits
        self.ax.set_xlim([-30, 30])
        self.ax.set_ylim([-30, 30])
        self.ax.set_zlim([0, 40])

        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')
        self.ax.set_title('5DOF Robot Arm')

        if self.holding:
            self.cube_pos = np.array(gripper)  # Keep cube at gripper position
        self.update_cube()  # Update cube position in the plot

        self.canvas.draw_idle()


    def send_servo_commands(self, index, value):
        
        if not (self.ser and self.ser.is_open):
            return  # Serial port not open
        now =  int(time.time() * 1000) # Current time in milliseconds
        if now - self.last_command_time < self.command_interval:
            return  # Throttle commands
        try:
            command = f"S{index} {value}\n"
            self.ser.write(command.encode())
            self.last_command_time = now
        except Exception as e:
            print(f"Error sending command: {e}")


def main():
    app = QApplication(sys.argv)
    window = RobotArmGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
