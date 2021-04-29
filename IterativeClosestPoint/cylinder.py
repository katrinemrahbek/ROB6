import open3d as o3d


mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(0.376/2,height=5.0, resolution = 50, split = 25)

o3d.visualization.draw_geometries([mesh_cylinder])

o3d.io.write_triangle_mesh("cylinder190.ply", mesh_cylinder)
