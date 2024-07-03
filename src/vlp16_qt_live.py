# Simple live point cloud viewer for Velodyne VLP-16 LIDAR

import socket
import struct
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from math import sin, cos, radians
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import Qt
import matplotlib.pyplot as plt

# Constants
UDP_IP = ""  # Bind to all available interfaces
UDP_PORT = 2368  # Default data port for VLP-16
ROTATION_LIMIT = 40000  # Number of rotations to persist points

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# Initialize variables to store points and rotation count
all_points = []
rotation_count = 0

# Vertical angles for VLP-16 (in degrees)
vertical_angles = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]

def parse_packet(data):
    """ Parse a single Velodyne packet """
    PACKET_SIZE = 1206  # Size of a complete VLP-16 packet
    BLOCK_SIZE = 100  # Size of a single block in bytes
    NUM_BLOCKS = 12  # Number of blocks in a single packet
    CHANNELS_PER_BLOCK = 32  # Number of channels per block
    ROTATION_ANGLE_OFFSET = 2  # Offset for rotation angle in block

    if len(data) != PACKET_SIZE:
        print(f"Invalid packet size: {len(data)}")
        return None

    blocks = []
    for i in range(NUM_BLOCKS):
        offset = i * BLOCK_SIZE
        header = data[offset:offset + 2]
        if header != b'\xff\xee':
            print(f"Invalid block header: {header}")
            continue  # Invalid block header

        azimuth = struct.unpack_from('<H', data, offset + ROTATION_ANGLE_OFFSET)[0] / 100.0

        block = []
        for j in range(CHANNELS_PER_BLOCK):
            channel_offset = offset + 4 + j * 3
            distance, reflectivity = struct.unpack_from('<HB', data, channel_offset)
            block.append((distance, reflectivity, azimuth))

        blocks.append(block)
    
    return blocks

def polar_to_cartesian(distance, horizontal_angle, vertical_angle):
    """ Convert polar coordinates to Cartesian coordinates """
    horizontal_angle_rad = radians(horizontal_angle)
    vertical_angle_rad = radians(vertical_angle)
    scaled_distance = distance / 200.0  # Adjust this scaling factor if needed

    x = scaled_distance * cos(vertical_angle_rad) * cos(horizontal_angle_rad)
    y = scaled_distance * cos(vertical_angle_rad) * sin(horizontal_angle_rad)
    z = scaled_distance * sin(vertical_angle_rad)

    return x, y, z, scaled_distance  # Return the scaled distance for coloring

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Live LiDAR Data')
        self.setGeometry(100, 100, 2048, 1080)  # Set window size to 2048x1080
        self.gl_widget = gl.GLViewWidget()
        self.setCentralWidget(self.gl_widget)
        self.gl_widget.setCameraPosition(distance=30, elevation=8, azimuth=45)
        
        # Add axis grid
        grid = gl.GLGridItem()
        grid.scale(10, 10, 1)
        self.gl_widget.addItem(grid)

        # Create a scatter plot item
        self.scatter = gl.GLScatterPlotItem(size=2)
        self.gl_widget.addItem(self.scatter)

        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(1)  # Reduced timer interval for faster updates

        self.all_points = []
        self.rotation_count = 0

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_R:
            self.all_points = []
            print("Point cloud data flushed")
            self.scatter.setData(pos=np.array([]))  # Clear the scatter plot

    def update(self):
        data, addr = sock.recvfrom(2048)
        if len(data) >= 1206:
            parsed_data = parse_packet(data[:1206])
            if parsed_data:
                xs = []
                ys = []
                zs = []
                distances = []

                for block_index, block in enumerate(parsed_data):
                    for channel_index, (distance, reflectivity, azimuth) in enumerate(block):
                        vertical_angle = vertical_angles[channel_index % 16]
                        x, y, z, scaled_distance = polar_to_cartesian(distance, azimuth, vertical_angle)
                        xs.append(x)
                        ys.append(y)
                        zs.append(z)
                        distances.append(scaled_distance)

                points = np.vstack((xs, ys, zs)).T
                self.all_points.append((points, distances))
                if len(self.all_points) > ROTATION_LIMIT:
                    self.all_points.pop(0)

                combined_points = np.vstack([p[0] for p in self.all_points])
                combined_distances = np.concatenate([p[1] for p in self.all_points])

                norm_distances = (combined_distances - np.min(combined_distances)) / (np.max(combined_distances) - np.min(combined_distances))

                cmap = plt.get_cmap('coolwarm')
                colors = cmap(norm_distances)

                self.scatter.setData(pos=combined_points, color=colors[:, :4])

                self.rotation_count += 1
                print(f"Rotation count: {self.rotation_count}")
                print(f"Number of points: {len(combined_points)}")
                for i in range(min(1, len(xs))):
                    print(f"Point {i}: x={xs[i]}, y={ys[i]}, z={zs[i]}, color={colors[i]}")

app = QApplication([])
main_window = MainWindow()
main_window.show()
app.exec_()