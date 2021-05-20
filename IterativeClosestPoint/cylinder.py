import open3d as o3d


mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(0.468/2,height=4.0, resolution = 500, split = 100)
bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound = (-1.0, -1.0 , 0.0), max_bound = (1.0,1.0,3.5)) # bounding box that crops data
croppedpcd = mesh_cylinder.crop(bbox)

normal = o3d.geometry.TriangleMesh.compute_triangle_normals(croppedpcd)

o3d.visualization.draw_geometries([normal])

o3d.io.write_triangle_mesh("cylinder468.ply", croppedpcd)
