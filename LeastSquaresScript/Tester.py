import numpy as np
import open3d as o3d
import time
import os
import matplotlib.pyplot as plt
import LeastSquareCylinderFit as LSC


def changer():
    folder = "ResultsChanged/"
    for files in os.listdir(folder):

        file = open(folder+files, "r+")
        content = file.read()
        file.seek(0, 0)
        exposure = []
        linesToRead = [0, 51, 102, 153]
        for i, line in enumerate(file):
            if i in linesToRead:
                exposure.append(line)

        file.seek(0, 0)

        thingy = ""
        print(exposure)
        for i in exposure:
            thingy += str(i)

        file.write(thingy + content)

        file.close()

def tester():
    rootdir = "data/trash_datasets"

    for subdir, dirs, files in os.walk(rootdir):

        radii = []
        times = []
        errors = []
        if subdir.count(os.sep) == 1:
            folder = subdir
            folder = folder.replace(rootdir, "")
            folder = folder.replace("\\", "")

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
            try:
                r, error = LSC.CylinderFitting(data)
            except:
                continue
            time2 = time.time()
            time3 = time2-time1
            radii.append(r)
            errors.append(error)
            times.append(time3)

        if len(radii) > 0:
            meanR = sum(r for r in radii)/len(radii)
            fps = 1/(sum(t for t in times)/len(times))
            print(subdir)

            temp = 0
            for r in radii:
                temp += (r-meanR)**2
            stddev = np.sqrt(temp/(len(radii)-1))


            f = open("results/results_LSCF_" + folder + ".txt", "a")
            f.write(subdir + ", r: " + str(meanR) + ", std_dev: " + str(stddev) + ", fps: " + str(fps) + "\n")
            for k, r in enumerate(radii):
                f.write(str(r) + ", " + str(errors[k]) + "\n")

            f.close()


changer()
