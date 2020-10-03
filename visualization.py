# Load and visualize the reconstructed point cloud

import open3d as o3d

# Load point cloud data
f = open("scene_pointcloud.txt", "r")

points = []

for point in f:
    point = point[1:-2]
    ar = point.split(',')
    points.append([ar[0], ar[1], ar[2]])

# Build point cloud
mycloud = o3d.geometry.PointCloud()
mycloud.points = o3d.utility.Vector3dVector(points)

# Visualization
o3d.visualization.draw_geometries([mycloud])