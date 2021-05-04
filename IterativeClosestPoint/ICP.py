
from typing import List
import numpy as np
from numpy.core.fromnumeric import mean
import open3d as o3d
import copy
import os
import time
import copy


def draw_registration_result(source, target, transformation): # draws the two point clouds in blue and yellow
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


class dataContainer:
    radiusList  = None
    MSE = None
    times = None
    meanRadius = 0.0
    folderName = ""
    fps  =  -1.0
    std_dev  = -1.0
    exp = ""
    

def save_data(data, pipe): #saves data
    
    open("data/" + pipe + "/result_ICP.txt", "w").close()
    f = open("data/" + pipe + "/result_ICP.txt", "a")
    print("jeg skriver til en fil nu")
    for i, d in enumerate(data):
        
        f.write("data/" + str(pipe_folder) + "/" + str(d.exp) +", r: " + str(d.meanRadius) + ", std_dev: " +str(d.std_dev) + ", fps: " + str(d.fps)+ "\n")
    for d in data:
        f.write("data/" + str(pipe_folder) + "/" + str(d.exp) + "\n")
        for i in range(0, len(d.radiusList)):
            f.write(str(d.radiusList[i])+ ", " + str(d.MSE[i]) + "\n")
    f.close()


class cylinder(): #cylinder class
    a = []
    b = []
    radius = -1
    #normalVector = []

pipe_folders = os.listdir("data/")


for pipe_folder in pipe_folders:
    exp_folders = os.listdir("data/" + str(pipe_folder))
    
    
    vector_data = []
    for exp_folder in exp_folders:
        
        dataCollector = dataContainer()
        dataCollector.times = []
        dataCollector.MSE = []
        dataCollector.radiusList = []
        if not os.path.isdir('data/' + pipe_folder + "/" +exp_folder):
            print(str(exp_folder) + "er ikke et dir")
            continue
        


        cyl = cylinder() # instance of cylinder
        radius = np.arange(0.07, 0.3, 0.01).tolist()  #make radius array with increments of 0.01
        averageRadius = 0.0
        arr = os.listdir('data/' + pipe_folder + "/" + exp_folder) 
        

        print("data/" + pipe_folder + "/" + exp_folder)
        time1 = time.time()
        for i in range (0,len(arr)):
            results = [] 
            results_cylinder_points = []

            
            pcd = o3d.io.read_point_cloud("data/" + pipe_folder + "/" + exp_folder+ "/" +str(arr[i])) #read point cloud
            bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound = (-1.0, -1.0 , 0.01), max_bound = (1.0,1.0,0.75)) # bounding box that crops data
            croppedpcd = pcd.crop(bbox)
           
            
            for i, rad in enumerate(radius):
                
                cyl.a = [0,0,0]
                cyl.b = [0,0,2]

                cyl.radius = rad
                mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(rad,height=1.7, resolution = 50, split = 25)
                mesh_pcd = o3d.geometry.PointCloud()
                mesh_pcd.points = o3d.utility.Vector3dVector(np.array(mesh_cylinder.vertices))
                cylinder_points = o3d.geometry.PointCloud()
                cylinder_points = mesh_pcd.crop(bbox)

                
                threshold = 0.07 # distance threshold  # 500 mm rør, 0.105

                trans_init = np.asarray(np.asarray([[1.0, 0.0, 0.0, 0.0],
                                                    [0.0, 1.0, 0.0, 0.0],
                                                    [0.0, 0.0, 1.0, 0.0], 
                                                    [0.0, 0.0, 0.0, 1.0]]))
                

                #print("Apply point-to-point ICP") # apply ICP
                
                reg_p2p = o3d.pipelines.registration.registration_icp(cylinder_points, croppedpcd, threshold, trans_init, o3d.pipelines.registration.TransformationEstimationPointToPoint())
                

                results.append(reg_p2p)
                results_cylinder_points.append(cylinder_points)
            
        
            min_error = 20000
            index = -1
            for i, res in enumerate(results): #figure out where minimum is and uses this as radius
                #print(i, res.inlier_rmse)
                if res.inlier_rmse < min_error and res.inlier_rmse > 0.0:
                    min_error = res.inlier_rmse
                    index = i
            
            dataCollector.radiusList.append(radius[index])
            dataCollector.MSE.append( results[index].inlier_rmse**2)
            dataCollector.exp = (exp_folder)
            averageRadius += radius[index]/len(arr)


            #print("index: " , index, " radius: ", radius[index], "diameter is ", radius[index]*2) # prints this and draws it 


        #draw_registration_result(results_cylinder_points[index], croppedpcd, results[index].transformation)
        print("average is = " + str(averageRadius))
        dataCollector.meanRadius = averageRadius
        time2 = time.time()
        time3 = time2-time1
        
        dataCollector.fps = len(arr)/time3

        temp = 0
        for r in dataCollector.radiusList:
            temp += (r-dataCollector.meanRadius)**2
        dataCollector.std_dev = np.sqrt(temp/(len(dataCollector.radiusList)-1))

        vector_data.append(copy.deepcopy(dataCollector))
        #print(len(vector_data[0].radiusList))

    #print(len(vector_data[0].radiusList))   
    #print("got ", len(vector_data) , " exposure folders")
    save_data(vector_data, str(pipe_folder))


