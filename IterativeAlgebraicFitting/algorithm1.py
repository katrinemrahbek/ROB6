## https://www.researchgate.net/publication/266214504_Extraction_of_cylinders_and_estimation_of_their_parameters_from_point_clouds

import numpy as np
import open3d as o3d
from cv2 import cv2
import math
import os
import matplotlib.pyplot as plt
import time

outputFile = "plastic_bag_200_Result_Algorithm1.txt"

def main():
    rootdir = "pipes_ply"
    alpha = 50
    beta = 0.95
    f = open(outputFile, "w")
    f.close
    error_list = []
    for subdir, dirs, files in os.walk(rootdir):
        tstart = time.time_ns()
        radii = []
        MSE_list = []
        for i, file in enumerate(files):
            plyFile = os.path.join(subdir, file)
            try:
                radius, MSE = calc_radius_on_ply(plyFile, alpha, beta)
                radii.append(radius)
                MSE_list.append(MSE)
            except: 
                print(str(subdir)+str(file)+": failed")
                error_list.append(str(subdir)+str(file)+": failed")

            print("Cylinder",i+1,"of", len(files)," -> r =",radius)

        if len(radii) != 0:
            tend = time.time_ns()
            fps = 1/((tend-tstart)/1000000000)
            WriteToFile(radii,subdir,fps,MSE_list)
    print(error_list)



def WriteToFile(radii,path,Afps,MSE):

    meanR = sum(r for r in radii)/len(radii)
    sumr = 0
    for j in range(len(radii)):
        sumr += (radii[j]-meanR)**2
    if len(radii)-1 != 0:
        std = np.sqrt(sumr/(len(radii)-1))
    else:
        std = 0


    f = open(outputFile, "a")
    f.write(str(path)+", r: "+str(meanR)+", std_dev: "+str(std)+", fps: "+str(Afps)+"\n")

    for j in range(len(radii)):
        f.write(str(radii[j])+", "+str(MSE[j])+"\n")

    f.write("\n")
    f.close


def calc_radius_on_ply(plyFileName, alpha, beta):

    original_pcd = o3d.io.read_point_cloud(plyFileName)
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-1, -1, -0.75), max_bound=(1, 1, 0.75))
    croppedpcd = original_pcd.crop(bbox)


    pcd = o3d.geometry.PointCloud(croppedpcd)
    
    pts = np.asarray(pcd.points)
    pts_cropped = crop_function(pts)
    pcd.points = o3d.utility.Vector3dVector(pts_cropped)
    
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    old_r = []
    r = 0
    MSE = 0
    for counter in range(50):

        newpcd = o3d.geometry.PointCloud()
        C = 0


        for i in pcd.normals:
            n = np.asarray([i])
            nt = np.transpose(n)
            C += nt*n

        eigval,eigvec = np.linalg.eig(C)
        
        eig_array = []
        for i in range(len(eigval)):
            eig_array.append([eigval[i],eigvec[i]])

        eig_array.sort(key=lambda a: a[0])

        a = eig_array[0][1]
        Cx = eig_array[1][1]
        Cy = eig_array[2][1]

        A = []
        b = []
        for i in pcd.points:
            x = np.dot(i,Cx)
            y = np.dot(i,Cy)
            col0 = 2*x
            col1 = 2*y
            col2 = 1
            rowb = np.square(math.sqrt(np.square(x)+np.square(y)))
            A.append([col0,col1,col2])
            b.append(rowb)

        AT = np.transpose(A)

        cx,cy,x = np.linalg.inv(AT.dot(A)).dot(AT).dot(b)

        r = np.sqrt(np.square(cx)+ np.square(cy) + x)



        ## Filter and update inliners
        axis_p = (cx*Cx) + (cy*Cy)
        MSE = 0
        for i in range(len(pcd.points)):
            pi = pcd.points[i]
            ni = pcd.normals[i]
            
            pip_dot = np.subtract(pi, axis_p)
            distance = np.subtract(pip_dot, (pip_dot.dot(a))*a)
            deltaDist = (np.linalg.norm(distance) - r)
            deltaNormal = np.cos(distance.dot(ni))/(np.linalg.norm(distance))
            MSE = MSE + deltaDist**2

            if (np.linalg.norm(deltaDist) < r/alpha) and (np.linalg.norm(deltaNormal) > beta):
                newpcd.points.append(pcd.points[i])
                newpcd.normals.append(pcd.normals[i])


        MSE *= 1.0 / len(pcd.points)
        pcd = o3d.geometry.PointCloud(newpcd)
        pcd.normals = newpcd.normals

        print("Iteration",counter+1,"-> r =",r)
        if len(old_r) >= 5:
            if r == old_r[counter-1] and r == old_r[counter-2]:
                break

        old_r.append(r)


    return r, MSE


def draw_cylinder(axis, point_on_axis, radius, height):
        cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=height)
        Rz, Ry = calculate_zy_rotation_for_arrow(axis)
        cylinder.rotate(Ry, center=np.array([0, 0, 0]))
        cylinder.rotate(Rz, center=np.array([0, 0, 0]))
        if axis[2] < 0:
            cylinder.translate(np.subtract(point_on_axis, axis))
        else:
            cylinder.translate(np.add(point_on_axis, axis))

        return cylinder


def calculate_zy_rotation_for_arrow(dirVector):
    # Rotation over z axis of the FOR
    gamma = np.arctan(dirVector[1] / dirVector[0])
    Rz = np.array([[np.cos(gamma), -np.sin(gamma), 0],
                   [np.sin(gamma), np.cos(gamma), 0],
                   [0, 0, 1]])
    # Rotate vec to calculate next rotation
    vec = Rz.T @ dirVector.reshape(-1, 1)
    vec = vec.reshape(-1)
    # Rotation over y axis of the FOR
    beta = np.arctan(vec[0] / vec[2])
    Ry = np.array([[np.cos(beta), 0, np.sin(beta)],
                   [0, 1, 0],
                   [-np.sin(beta), 0, np.cos(beta)]])
    return Rz, Ry


def crop_function(points):
    return_list = []
    for point in points:
        distance = np.sqrt(np.power(point[0], 2) + np.power(point[1], 2) + np.power(point[2], 2))
        if distance > 0.01:
            return_list.append(point)
    return return_list


if __name__ == "__main__":
    main()