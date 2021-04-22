import numpy as np
import open3d as o3d
import time
import os
import matplotlib.pyplot as plt
import LeastSquareCylinderFit as LSC

rootdir = "data/wet_pvc_400/"

for subdir, dirs, files in os.walk(rootdir):
    radii = []
    times = []
    errors = []
    for k, file in enumerate(files):
        '''load ply file from directory'''
        pcd = o3d.io.read_point_cloud(os.path.join(subdir, file))

        print(file)

        '''crop points'''
        bbox1 = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-1, -1, 0.01), max_bound=(1, 1, 0.75))
        bbox2 = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-1, -1, -0.75), max_bound=(1, 1, -0.01))
        pcd1 = pcd.crop(bbox1)
        pcd2 = pcd.crop(bbox2)
        data1 = np.array(pcd1.points)
        data2 = np.array(pcd2.points)
        data = np.concatenate((data1, data2))


        time1 = time.time()

        #Call function
        r, error = LSC.CylinderFitting(data)

        time2 = time.time()
        time3 = time2-time1
        radii.append(r)
        errors.append(error)
        times.append(time3)

    if(len(radii) > 0):
        meanR = sum(r for r in radii)/len(radii)
        fps = 1/(sum(t for t in times)/len(times))
        print(subdir)

        temp = 0
        for r in radii:
            temp += (r-meanR)**2
        stddev = np.sqrt(temp/(len(radii)-1))

        f = open("results_wet_pvc_400.txt", "a")
        f.write(subdir + ", r: " + str(meanR) + ", std_dev: " + str(stddev) + ", fps: " + str(fps) + "\n")
        for k, r in enumerate(radii):
            f.write(str(r) + ", " + str(errors[k]) + "\n")

        f.close()