
from typing import List
import numpy as np
from numpy.core.fromnumeric import mean
import open3d as o3d
import copy
import os
import time


def draw_registration_result(source, target, transformation): # draws the two point clouds in blue and yellow
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


class dataContainer:
    radiusList  = []
    MSE = []
    times = []
    meanRadius = 0.0
    folderName = ""
    fps  =  -1.0
    std_dev  = -1.0
    

def save_data(data, pipe):
    # folder_name, mean radius, corrected sample standard deviation, fps
    # for example:
    # data/beton200/exp_200, r: 0.101436, std_dev: 0.000165287, fps: 3.86907
    # data/...

    # then all the radius' and MSE for each ply file:
    # data/beton200/exp_50
    # 0.1023, 0.012
    mean_time = 0
    for t in data.times:
        mean_time += t/len(data.times)

    data.fps = 1/mean_time
    #data.std_dev = np.sqrt(1/len(data.radiusList)*sum(1,len(data.radiusList),)

    f = open("data/" + pipe + "/result_ICP.txt", "w")
    print("jeg skriver til en fil nu")
    f.write("data/" + str(pipe_folder) + "/" + str(exp_folder) +", r: " + str(data.meanRadius) + ", std_dev: " +str(data.std_dev) + ", fps: " + str(data.fps)+ "\n")
    for i in range(0, len(data.radiusList)):
        f.write(str(data.radiusList[i])+ ", " + str(data.MSE) + "\n")

    pass

class cylinder(): #cylinder class
    a = []
    b = []
    radius = -1
    #normalVector = []

pipe_folders = os.listdir("data/")
vector_data = []
#print(pipe_folders)

for pipe_folder in pipe_folders:
    exp_folders = os.listdir("data/" + str(pipe_folder))
    
    #print(exp_folders)

    dataCollector = dataContainer()


    for exp_folder in exp_folders:

        if not os.path.isdir(exp_folder):
            pass
        print(exp_folder)


        cyl = cylinder() # instance of cylinder
        radius = np.arange(0.1, 0.25, 0.01).tolist()  #make radius array with increments of 0.01
        averageRadius = 0.0
        arr = os.listdir('data/' + pipe_folder+ "/" + exp_folder ) 
        

        print("data/" + pipe_folder + "/" + exp_folder)

        for i in range (0,len(arr)):
            results = [] 
            results_cylinder_points = []

            
            pcd = o3d.io.read_point_cloud("data/" + pipe_folder + "/" + exp_folder+ "/" +str(arr[i])) #read point cloud
            bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound = (-1.0, -1.0 , 0.05), max_bound = (1.0,1.0,0.75)) # bounding box that crops data
            croppedpcd = pcd.crop(bbox)
            #downpcd = croppedpcd.voxel_down_sample(voxel_size=0.005) #downsample point cloud
            time1 = time.time()
            for i, rad in enumerate(radius):
                
                cyl.a = [0,rad/2.0,0]
                cyl.b = [0,rad/2.0,2]

                cyl.radius = rad
                mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(rad,height=1.7, resolution = 100, split = 25)
                mesh_pcd = o3d.geometry.PointCloud()
                mesh_pcd.points = o3d.utility.Vector3dVector(np.array(mesh_cylinder.vertices))
                cylinder_points = o3d.geometry.PointCloud()
                cylinder_points = mesh_pcd.crop(bbox)

                
                threshold = 0.1 # distance threshold

                trans_init = np.asarray(np.asarray([[1.0, 0.0, 0.0, 0.0],
                                                    [0.0, 1.0, 0.0, 0.0],
                                                    [0.0, 0.0, 1.0, 0.0], 
                                                    [0.0, 0.0, 0.0, 1.0]]))
                

                #print("Apply point-to-point ICP") # apply ICP
                
                reg_p2p = o3d.pipelines.registration.registration_icp(cylinder_points, croppedpcd, threshold, trans_init, o3d.pipelines.registration.TransformationEstimationPointToPoint())
                

                results.append(reg_p2p)
                results_cylinder_points.append(cylinder_points)
            
        
            min_error = 20000000
            index = -1
            for i, res in enumerate(results): #figure out where minimum is and uses this as radius
                #print(i, res.inlier_rmse)
                if res.inlier_rmse < min_error:
                    min_error = res.inlier_rmse
                    index = i
            time2 = time.time()
            time3 = time2-time1

            #print("index: " , index, " radius: ", radius[index], "diameter is ", radius[index]*2) # prints this and draws it 

            dataCollector.radiusList.append(radius[index])
            dataCollector.MSE = res.inlier_rmse**2
            averageRadius += radius[index]/len(arr)

        

        #draw_registration_result(results_cylinder_points[index], croppedpcd, results[index].transformation)
        print("average is = " + str(averageRadius))
        dataCollector.meanRadius = averageRadius
        dataCollector.times.append(time3)

        save_data(dataCollector, str(pipe_folder))


