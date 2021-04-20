import matplotlib.pyplot as plt
import open3d as o3d
import numpy as np
import sympy
import sympy as sym
import matplotlib.pyplot as plt
from os import listdir
from os.path import isfile, join
import logging
import queue
from multiprocessing import Pool
import time

path = "Dataset/dry_pvc_400_long_low/exp_500"
onlyfiles = [f for f in listdir(path) if isfile(join(path, f))]
'''Define task function'''
def is_prime(n):
    MyPointClass(n)
    print("Done")

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
        if distance < 1.5 and point[2] < 1.5:
            return_list.append(point)
    return return_list


def calculate_mean(the_points):
    mean = np.array([0, 0, 0])
    for i in range(len(the_points)):
        mean = np.add(mean, the_points[i])
    mean = np.divide(mean, len(the_points))
    return mean


def draw_arrow(dir, place=[0, 0, 0], farve=[1, 0, 0]):
    mesh_arrow3 = o3d.geometry.TriangleMesh.create_arrow(
        cone_height=0.02,
        cone_radius=0.003,
        cylinder_height=0.04,
        cylinder_radius=0.002
    )
    Rz, Ry = calculate_zy_rotation_for_arrow(dir)
    mesh_arrow3.rotate(Ry, center=np.array([0, 0, 0]))
    mesh_arrow3.rotate(Rz, center=np.array([0, 0, 0]))
    mesh_arrow3.translate(place)
    mesh_arrow3.paint_uniform_color(farve)

    return mesh_arrow3


class Plane:
    def __init__(self, normal, point):
        self.normal = normal / np.linalg.norm(normal)
        self.point = point
        self.rot_matrix = self.rot_matrix_cal()

    def rot_matrix_cal(self):
        ax, ay, az = self.get_normal_angles()
        ax, ay, az = (np.pi / 2) - ax, (np.pi / 2) - ay, (np.pi / 2) - az

        Rx = np.array([[1, 0, 0],
                       [0, np.cos(ax), -np.sin(ax)],
                       [0, np.sin(ax), np.cos(ax)]])

        Ry = np.array([[np.cos(ay), 0, np.sin(ay)],
                       [0, 1, 0],
                       [-np.sin(ay), 0, np.cos(ay)]])

        Rz = np.array([[np.cos(az), -np.sin(az), 0],
                       [np.sin(az), np.cos(az), 0],
                       [0, 0, 1]])

        RxRyRz = np.matmul(Rz, np.matmul(Ry, Rx))

        return RxRyRz

    def distance_to(self, point):
        return np.dot(self.normal, point)

    def project_onto(self, point):
        t = self.distance_to(point)
        return point - np.multiply(t, self.normal)

    def draw_plane(self):
        plane_points = []
        for x in range(-100, 100):
            for y in range(-100, 100):
                new_x = x / 1000.0
                new_y = y / 1000.0
                z = (self.normal[0] * (0 - new_x) + self.normal[1] * (0 - new_y) + self.normal[2] * 0) / self.normal[2]
                plane_points.append([new_x, new_y, z])

        return plane_points

    def draw_plane_o3d(self):
        plane_points = self.draw_plane()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(plane_points)
        return pcd

    def get_normal_angles(self):
        return np.arctan2(np.sqrt(self.normal[1] ** 2 + self.normal[2] ** 2), self.normal[0]), \
               np.arctan2(np.sqrt(self.normal[2] ** 2 + self.normal[0] ** 2), self.normal[1]), \
               np.arctan2(np.sqrt(self.normal[0] ** 2 + self.normal[1] ** 2), self.normal[2])

    def corrected_points(self, points):
        return_points = []
        for point in points:
            return_points.append(np.subtract(np.matmul(point, self.rot_matrix), self.point))

        return return_points


class MyPointClass:
    def __init__(self, files):
        #self.pcd = o3d.io.read_point_cloud("pico/royale_20191209_153736-57082-000001-2019-12-9-14-37-36.477000.ply")
        #self.pcd = o3d.io.read_point_cloud("pico/royale_20191209_153736-57082-000001-2019-12-9-14-37-36.477000.ply")
        print(files)
        self.pcd = o3d.io.read_point_cloud(path + "/" + files)
        print("Read")
        pts = np.asarray(self.pcd.points)
        pts_cropped = crop_function(pts)
        self.pcd.points = o3d.utility.Vector3dVector(pts_cropped)

        #o3d.visualization.draw_geometries([self.pcd])
        #self.pcd.voxel_down_sample(voxel_size=0.05)
        print("Cropped")
        self.pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1,
                                                              max_nn=30))
        print("Pipe")
        returnsValues = self.fit_cylinder(files)

        #mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.08)


    def fit_height_plane(self):
        the_points = np.asarray(self.pcd.points)
        mean = calculate_mean(the_points)

        xxSum, xySum, xhSum, yySum, yhSum = 0, 0, 0, 0, 0
        for i in range(len(the_points)):
            diff = np.subtract(the_points[i], mean)
            xxSum += diff[0] * diff[0]
            xySum += diff[0] * diff[1]
            xhSum += diff[0] * diff[2]
            yySum += diff[1] * diff[1]
            yhSum += diff[1] * diff[2]

        det = xxSum * yySum - xySum * xySum
        barX, barY, barH, barA0, barA1 = 0, 0, 0, 0, 0
        if det != 0:
            barX = mean[0]
            barY = mean[1]
            barH = mean[2]
            barA0 = (yySum * xhSum - xySum * yhSum) / det
            barA1 = (xxSum * yhSum - xySum * xhSum) / det

        plane_points = []
        for x in range(-100, 100):
            for y in range(-100, 100):
                newx = x / 100.0
                newy = y / 100.0
                plane_points.append([newx, newy, barA0 * newx + barA1 * newy + (barH - barA0 * newx - barA1 * newy)])

        # self.pcd.estimate_normals(search_param=geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
        pcd = o3d.geometry.PointCloud()
        new_array = np.concatenate((the_points, plane_points))

        pcd.points = o3d.utility.Vector3dVector(new_array)
        o3d.visualization.draw_geometries([pcd])

    def fit_line(self):
        the_points = np.asarray(self.pcd.points)
        mean = calculate_mean(the_points)
        n = len(the_points[0])

        C = np.zeros((n, n))

        for i in range(len(the_points)):
            diff = np.subtract(the_points[i], mean)
            C = np.add(C, np.outer(diff, diff))

        output = np.linalg.eig(C)
        eigenvalues = output[0]
        eigenvectors = output[1]

        origin = mean
        directionX = eigenvectors[0]
        directionY = eigenvectors[1]
        directionZ = eigenvectors[2]

        mesh_arrow = o3d.geometry.TriangleMesh.create_arrow(
            cone_height=0.02,
            cone_radius=0.003,
            cylinder_height=0.04,
            cylinder_radius=0.002
        )
        Rz, Ry = calculate_zy_rotation_for_arrow(directionX)
        mesh_arrow.rotate(Ry, center=np.array([0, 0, 0]))
        mesh_arrow.rotate(Rz, center=np.array([0, 0, 0]))
        mesh_arrow.translate(origin)
        mesh_arrow.paint_uniform_color([1, 0, 0])

        mesh_arrow2 = o3d.geometry.TriangleMesh.create_arrow(
            cone_height=0.02,
            cone_radius=0.003,
            cylinder_height=0.04,
            cylinder_radius=0.002
        )
        Rz, Ry = calculate_zy_rotation_for_arrow(directionY)
        mesh_arrow2.rotate(Ry, center=np.array([0, 0, 0]))
        mesh_arrow2.rotate(Rz, center=np.array([0, 0, 0]))
        mesh_arrow2.translate(origin)
        mesh_arrow2.paint_uniform_color([0, 1, 0])

        mesh_arrow3 = o3d.geometry.TriangleMesh.create_arrow(
            cone_height=0.02,
            cone_radius=0.003,
            cylinder_height=0.04,
            cylinder_radius=0.002
        )
        Rz, Ry = calculate_zy_rotation_for_arrow(directionZ)
        mesh_arrow3.rotate(Ry, center=np.array([0, 0, 0]))
        mesh_arrow3.rotate(Rz, center=np.array([0, 0, 0]))
        mesh_arrow3.translate(origin)
        mesh_arrow3.paint_uniform_color([0, 0, 1])

        return [mesh_arrow, mesh_arrow2, mesh_arrow3]


    def draw_cylinder(self, axis, point_on_axis, radius):

        cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=0.2)
        Rz, Ry = calculate_zy_rotation_for_arrow(axis)
        cylinder.rotate(Ry, center=np.array([0, 0, 0]))
        cylinder.rotate(Rz, center=np.array([0, 0, 0]))
        if axis[2] < 0:
            cylinder.translate(np.subtract(point_on_axis, axis))
        else:
            cylinder.translate(np.add(point_on_axis, axis))

        return cylinder


    def fit_cylinder(self, save_name):
        all_the_points = np.asarray(self.pcd.points)

        beta = 0.3
        alfa = 2

        inliers = all_the_points
        all_the_normals = np.asarray(self.pcd.normals)
        normals = all_the_normals
        MSE_data = []
        histogram_data = []
        radiuses = []
        iterations = 50
        for number in range(0, iterations):
            print("Iteration:", number + 1, " of ", iterations)

            C = 0
            for i in normals:
                n = np.asarray([i])
                nt = np.transpose(n)
                C += nt * n

            eigenvalues, eigenvectors = np.linalg.eig(C)

            eigen_array = [() for x in range(len(eigenvalues))]
            for i in range(len(eigenvalues)):
                eigen_array[i] = (eigenvalues[i], eigenvectors[i])

            eigen_array.sort(key=lambda a: a[0])

            normal_vec = eigen_array[0][1]
            Cx = eigen_array[1][1]
            Cy = eigen_array[2][1]

            A = np.zeros(shape=(len(inliers), 3))
            b = np.zeros(shape=(len(inliers), 1))
            for i in range(len(inliers)):
                point_work = inliers[i]
                p_line_x = np.dot(point_work, Cx)
                p_line_y = np.dot(point_work, Cy)
                A[i, 0] = 2 * p_line_x
                A[i, 1] = 2 * p_line_y
                A[i, 2] = 1
                b[i] = np.sqrt(p_line_x ** 2 + p_line_y ** 2) ** 2

            AT = A.transpose()
            C = np.linalg.inv(AT.dot(A)).dot(AT).dot(b)
            r = np.sqrt(np.linalg.norm(-(C[0] ** 2) - (C[1] ** 2) + C[2]))

            p_dot = C[0]*Cx + C[1]*Cy

            new_inliers = []
            new_normals = []
            MSE = 0
            histogram_data.append([])
            for i in range(len(all_the_points)):
                pi = all_the_points[i]
                normal = all_the_normals[i]

                pip_dot = np.subtract(pi, p_dot)
                distance = np.subtract(pip_dot, (pip_dot.dot(normal_vec))*normal_vec)
                ddist = (np.linalg.norm(distance) - r)
                dnormal = np.cos(distance.dot(normal))/(np.linalg.norm(distance))
                MSE = MSE + ddist**2

                histogram_data[number].append(ddist)

                if (np.linalg.norm(ddist) < r/alfa) and (np.linalg.norm(dnormal) > beta):
                    new_inliers.append([pi[0], pi[1], pi[2]])
                    new_normals.append(normal)

            MSE *= 1.0 / len(all_the_points)
            MSE_data.append(MSE)
            inliers = new_inliers
            normals = new_normals
            radiuses.append(r)
            if len(inliers) < 9:
                break


        #fig, ax = plt.subplots(int(np.ceil(iterations / 2)), int(np.ceil(iterations / 2)))
        #for i in range(len(histogram_data)):
        #    index_row = i%iterations
        #    index_col = int(i/iterations)
        #    working_ax = ax[index_col, index_row]
        #    working_ax.hist(histogram_data[i])
        #    working_ax.set_ylim([0, 22000])
        #    working_ax.set_xlim([-0.20, 0.05])

        fig2, ax2 = plt.subplots(1,2)
        ax2[0].plot(range(len(MSE_data)), MSE_data, '-')
        ax2[1].plot(range(len(MSE_data)), radiuses, '-')

        plt.draw()
        plt.pause(0.001)
        plt.savefig(path + "/theplots" + save_name[:-4] + ".png")
        plt.close(fig2)

        my_cyl = self.draw_cylinder(normal_vec, p_dot, r)
        mesh_arrow1 = draw_arrow(normal_vec, place=p_dot, farve=[1, 0, 0])
        mesh_arrow2 = draw_arrow(Cx, place=p_dot, farve=[0, 1, 0])
        mesh_arrow3 = draw_arrow(Cy, place=p_dot, farve=[0, 0, 1])

        return my_cyl, mesh_arrow3, mesh_arrow2, mesh_arrow1


if __name__ == '__main__':
    for file in onlyfiles:
        is_prime(file)
