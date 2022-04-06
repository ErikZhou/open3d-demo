import open3d as o3d
import numpy as np
#import open3d_tutorial as o3dtut
from common import open3d_tutorial as o3dtut


gt_mesh = o3dtut.get_bunny_mesh()
pcd = gt_mesh.sample_points_poisson_disk(5000)
pcd.normals = o3d.utility.Vector3dVector(np.zeros(
    (1, 3)))  # invalidate existing normals

pcd.estimate_normals()
o3d.visualization.draw_geometries([pcd], point_show_normal=True)


pcd.orient_normals_consistent_tangent_plane(100)
o3d.visualization.draw_geometries([pcd], point_show_normal=True)

