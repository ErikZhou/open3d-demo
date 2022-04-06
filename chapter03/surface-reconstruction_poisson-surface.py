"""
http://www.open3d.org/docs/release/tutorial/Advanced/surface_reconstruction.html#Poisson-surface-reconstruction

表面重建方法会产生不平滑的结果，因为PointCloud的点也是三角形网格的顶点，无需进行任何修改。
泊松曲面重构方法[Kazhdan2006]解决了正规化优化问题，以获得光滑表面。

Open3D实现了create_from_point_cloud_poisson方法，该方法基本上是Kazhdan代码的包装。
该函数的一个重要参数是深度，深度定义了用于表面重建的八叉树的深度，因此暗示了所得三角形网格的分辨率。 较高的深度值意味着具有更多细节的网格。

This algorithm assumes that the PointCloud has normals.
"""

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from common import open3d_tutorial as o3dtut

mesh = o3dtut.get_bunny_mesh()
pcd = mesh.sample_points_poisson_disk(3000)
#pcd = o3dtut.get_eagle_pcd()


print(pcd)
'''
pcd.estimate_normals()
# to obtain a consistent normal orientation
pcd.orient_normals_towards_camera_location(pcd.get_center())

# or you might want to flip the normals to make them point outward, not mandatory
pcd.normals = o3d.utility.Vector3dVector( - np.asarray(pcd.normals))

# surface reconstruction using Poisson reconstruction
mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)

# paint uniform color to better visualize, not mandatory
mesh.paint_uniform_color(np.array([0.7, 0.7, 0.7]))

o3d.io.write_triangle_mesh('a.ply', mesh)
o3d.visualization.draw_geometries([mesh])
'''


pcd.paint_uniform_color([1.0, 0.0, 1.0])
o3d.visualization.draw_geometries([pcd])
#o3d.visualization.draw_geometries([pcd], zoom=0.664, front=[-0.4761, -0.4698, -0.7434], lookat=[1.8900, 3.2596, 0.9284], up=[0.2304, -0.8825, 0.4101])

# Poisson surface reconstruction
radius = 0.1  # 搜索半径
max_nn = 30  # 邻域内用于估算法线的最大点数
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius, max_nn))  # 法线估计

print('run Poisson surface reconstruction')
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)#9
print(mesh)
o3d.visualization.draw_geometries([mesh],mesh_show_back_face=True)

'''
# 点云体素化
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=0.05)
# 可视化重建结果
o3d.visualization.draw_geometries([voxel_grid], window_name="点云重建",
                                  width=800,
                                  height=600,
                                  mesh_show_back_face=True)

print('visualize densities')
densities = np.asarray(densities)
density_colors = plt.get_cmap('plasma')(
    (densities - densities.min()) / (densities.max() - densities.min()))
density_colors = density_colors[:, :3]
density_mesh = o3d.geometry.TriangleMesh()
density_mesh.vertices = mesh.vertices
density_mesh.triangles = mesh.triangles
density_mesh.triangle_normals = mesh.triangle_normals
density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
o3d.visualization.draw_geometries([density_mesh])

print('remove low density vertices')
vertices_to_remove = densities < np.quantile(densities, 0.01)
mesh.remove_vertices_by_mask(vertices_to_remove)
print(mesh)
o3d.visualization.draw_geometries([mesh])
'''
