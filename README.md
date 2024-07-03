# Simple Live Point Cloud Viewer for Velodyne VLP-16 LiDAR

This repository contains a simple Python-based live point cloud viewer for the Velodyne VLP-16 LiDAR sensor. 
The viewer uses PyQtGraph for visualization and can handle real-time data streaming from the sensor.

## Features

- **Real-time Visualization**: Visualize point cloud data in real-time.
- **Scatter Plot with Color Mapping**: Points are color-coded based on their distance.
- **Keyboard Interaction**: Press `R` to flush the point cloud data.

## Requirements

- Python 3.x
- NumPy
- PyQt5
- PyQtGraph
- Matplotlib