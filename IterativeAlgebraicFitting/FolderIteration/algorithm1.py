## https://www.researchgate.net/publication/266214504_Extraction_of_cylinders_and_estimation_of_their_parameters_from_point_clouds

import numpy as np
import open3d as o3d
import glob, os
import matplotlib.pyplot as plt
import time
from numba import njit, jit
import copy

outputFile = "_Result_Algorithm1.txt"


def main():
    rootdir = "/home/hax/Desktop/sync_folder/P6/testfit_dataset/"
    alpha = 50
    beta = 0.95
    error_list = []
    for datasets in os.listdir(rootdir):
        dataset_dir = os.path.join(rootdir, datasets)
        with open(os.path.join(dataset_dir, datasets + outputFile), 'w') as outputFileHandler:
            result_string = ""
            exposure_folders = os.listdir(dataset_dir)
            exposure_folders.sort()
            for exposure_folder in exposure_folders:
                exposure_dir = os.path.join(dataset_dir, exposure_folder)
                tstart = time.time_ns()
                radii = []
                MSE_list = []
                ply_files = glob.glob(os.path.join(exposure_dir, '*.ply'))
                ply_files.sort()
                radius = 0
                for i, ply_file in enumerate(ply_files):
                    plyFile = os.path.join(exposure_dir, ply_file)
                    print(plyFile)
                    original_pcd = o3d.io.read_point_cloud(plyFile)
                    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-1, -1, -0.75), max_bound=(1, 1, 0.75))
                    croppedpcd = original_pcd.crop(bbox)

                    pcd = o3d.geometry.PointCloud(croppedpcd)

                    pts = np.asarray(pcd.points)
                    pts_cropped = crop_function(pts)
                    pcd.points = o3d.utility.Vector3dVector(pts_cropped)

                    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

                    try:
                        radius, MSE = calc_radius_on_ply(np.asarray(pcd.points), np.asarray(pcd.normals), alpha, beta)
                        radii.append(radius)
                        MSE_list.append(MSE)

                        print("Cylinder", i + 1, "of", len(ply_files), " -> r =", radius)
                    except:
                        break

                if len(radii) != 0:
                    tend = time.time_ns()
                    fps = 1 / ((tend - tstart) / 1000000000)

                    meanR = sum(r for r in radii) / len(radii)
                    sumr = 0
                    for j in range(len(radii)):
                        sumr += (radii[j] - meanR) ** 2
                    if len(radii) - 1 != 0:
                        std = np.sqrt(sumr / (len(radii) - 1))
                    else:
                        std = 0

                    outputFileHandler.write(
                        str(exposure_folder) + ", r: " + str(meanR) + ", std_dev: " + str(std) + ", fps: " + str(
                            fps) + "\n")

                    result_string += str(exposure_folder) + "\n"
                    for j in range(len(radii)):
                        result_string += (str(radii[j]) + ", " + str(MSE_list[j]) + "\n")

                    outputFileHandler.flush()

            outputFileHandler.write(result_string)

            print(error_list)


@njit()
def norm(point1):
    abs_value = 0.0
    for i in point1:
        abs_value += i ** 2

    return np.sqrt(abs_value)

@njit()
def calc_radius_on_ply(pcd_points, pcd_normals, alpha, beta):
    old_r = 0.0
    old_r_2 = 0.0
    r = 0.0
    MSE = 0.0
    old_MSE = 0.0
    internal_pcd = np.copy(pcd_points)
    internal_normals = np.copy(pcd_normals)
    for counter in range(50):
        C = np.zeros(shape=(3, 3))
        for i in internal_normals:
            n = np.asarray([[i[0]], [i[1]], [i[2]]])
            nt = np.asarray([[i[0], i[1], i[2]]])
            C += nt * n


        try:
            eigval, eigvec = np.linalg.eig(C)
        except:
            return old_r, old_MSE

        eig_array = []
        for i in range(len(eigval)):
            eig_array.append((eigval[i], i))

        eig_array = sorted(eig_array)

        a = eigvec[eig_array[0][1]]
        Cx = eigvec[eig_array[1][1]]
        Cy = eigvec[eig_array[2][1]]

        A = []
        b = []
        for i in range(internal_pcd.shape[0]):
            x = internal_pcd[i][0] * Cx[0] + internal_pcd[i][1] * Cx[1] + internal_pcd[i][2] * Cx[2]
            y = internal_pcd[i][0] * Cy[0] + internal_pcd[i][1] * Cy[1] + internal_pcd[i][2] * Cy[2]
            col0 = 2 * x
            col1 = 2 * y
            col2 = 1
            rowb = np.square(np.sqrt(np.square(x) + np.square(y)))
            A.append([col0, col1, col2])
            b.append(rowb)

        AT = np.transpose(np.asarray(A))

        cx, cy, x = np.linalg.inv(np.asarray(AT).dot(np.asarray(A))).dot(np.asarray(AT)).dot(np.asarray(b))

        old_r_2 = old_r
        old_r = r

        r = np.sqrt(np.square(cx) + np.square(cy) + x)

        ## Filter and update inliners
        axis_p = (cx * Cx) + (cy * Cy)

        newArraySize = 0
        for i in range(pcd_points.shape[0]):
            pi = pcd_points[i]
            ni = pcd_normals[i]

            pip_dot = np.subtract(pi, axis_p)
            distance = np.subtract(pip_dot, (pip_dot.dot(a)) * a)
            deltaDist = (norm(distance) - r)
            deltaNormal = np.cos(distance.dot(ni)) / (norm(distance))
            MSE = MSE + deltaDist ** 2

            if (abs(deltaDist) < r / alpha) and (abs(deltaNormal) > beta):
                newArraySize += 1

        old_MSE = MSE
        MSE = 0
        new_index = 0
        newpcd = np.zeros(shape=(newArraySize, 3))
        newnormals = np.zeros(shape=(newArraySize, 3))
        for i in range(pcd_points.shape[0]):
            pi = pcd_points[i]
            ni = pcd_normals[i]

            pip_dot = np.subtract(pi, axis_p)
            distance = np.subtract(pip_dot, (pip_dot.dot(a)) * a)
            deltaDist = (norm(distance) - r)
            deltaNormal = np.cos(distance.dot(ni)) / (norm(distance))
            MSE = MSE + deltaDist ** 2

            if (abs(deltaDist) < r / alpha) and (abs(deltaNormal) > beta):
                newpcd[new_index] = pcd_points[i]
                newnormals[new_index] = pcd_normals[i]
                new_index += 1

        MSE *= 1.0 / len(internal_pcd)
        internal_pcd = newpcd
        internal_normals = newnormals

        print("Iteration", counter + 1, "-> r =", r)
        if counter >= 5:
            if old_r == r and old_r_2 == r:
                break

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


@njit()
def crop_function(points):
    return_list = []
    for point in points:
        distance = np.sqrt(np.power(point[0], 2) + np.power(point[1], 2) + np.power(point[2], 2))
        if distance > 0.01:
            return_list.append(point)
    return return_list


if __name__ == "__main__":
    main()
