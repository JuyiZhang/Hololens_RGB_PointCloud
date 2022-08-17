import open3d as o3d
import  numpy

mesh = o3d.io.read_triangle_mesh("Segmentation_Skin_Reduced.stl")
pointcloud = mesh.sample_points_poisson_disk(10000)
pointcloud.scale(0.001,[0,0,0])
points=numpy.asarray(pointcloud.points)
# o3d.visualization.draw_geometries([mesh])
o3d.visualization.draw_geometries([pointcloud])