import numpy as np
import open3d as o3d
import time
import os
import matplotlib.pyplot as plt


def preprocess(data):
    n = len(data)
    dataSum = sum(i for i in data)

    average = dataSum/n

    Xi = np.zeros((n, 3))
    for i in range(n):
        Xi[i] = data[i] - average

    mu = np.zeros(6)
    products = np.zeros((n, 6))
    for i in range(n):
        '''Calculate products of Xi*Xi'''
        products[i][0] = Xi[i][0] * Xi[i][0]
        products[i][1] = Xi[i][0] * Xi[i][1]
        products[i][2] = Xi[i][0] * Xi[i][2]
        products[i][3] = Xi[i][1] * Xi[i][1]
        products[i][4] = Xi[i][1] * Xi[i][2]
        products[i][5] = Xi[i][2] * Xi[i][2]
        '''Calc mu'''
        mu[0] += products[i][0]
        mu[1] += 2 * products[i][1]
        mu[2] += 2 * products[i][2]
        mu[3] += products[i][3]
        mu[4] += 2 * products[i][4]
        mu[5] += products[i][5]
    mu = mu/n

    F0 = np.zeros((3, 3))
    F1 = np.zeros((3, 6))
    F2 = np.zeros((6, 6))
    for i in range(n):
        '''Calc Delta'''
        delta = np.zeros(6)
        delta[0] = products[i][0]
        delta[1] = 2 * products[i][1]
        delta[2] = 2 * products[i][2]
        delta[3] = products[i][3]
        delta[4] = 2 * products[i][4]
        delta[5] = products[i][5]
        delta = np.subtract(delta, mu)

        F0[0][0] += products[i][0]
        F0[0][1] += products[i][1]
        F0[0][2] += products[i][2]
        F0[1][1] += products[i][3]
        F0[1][2] += products[i][4]
        F0[2][2] += products[i][5]
        F1 += np.outer(Xi[i], delta)
        F2 += np.outer(delta, delta)
    F0 = F0/n
    F0[1][0] = F0[0][1]
    F0[2][0] = F0[0][2]
    F0[2][1] = F0[1][2]
    F1 = F1/n
    F2 = F2/n
    return n, Xi, mu, F0, F1, F2, average

def G(W, mu, F0, F1, F2, n):
    '''Calc projection Matrix from current direction'''
    P = np.identity(3) - np.outer(W, W)
    '''Calc Skew matrix'''
    S = [[0, -W[2], W[1]], [W[2], 0,  -W[0]], [-W[1], W[0], 0]]

    A = np.matmul(P, np.matmul(F0, P))
    hatA = -(np.matmul(S, np.matmul(A, np.transpose(S))))
    hatAA = np.matmul(hatA, A)

    trace = np.trace(hatAA)

    Q = hatA / trace
    p = [P[0][0], P[0][1], P[0][2], P[1][1], P[1][2], P[2][2]]
    alpha = np.matmul(F1, p)
    beta = np.matmul(Q, alpha)

    error = (np.dot(p, np.matmul(F2, p)) - 4 * np.dot(alpha, beta) + 4 * np.dot(beta, np.matmul(F0, beta))) / n
    rSQR = np.dot(p, mu) + np.dot(beta, beta)
    PC = beta

    return error, rSQR, PC


def CylinderFitting(subdir, file):
    pcd = o3d.io.read_point_cloud(os.path.join(subdir, file))
    '''pcd = pcd.voxel_down_sample(voxel_size=0.005)'''
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-1, -1, 0), max_bound=(1, 1, 1))
    pcd = pcd.crop(bbox)
    '''o3d.visualization.draw_geometries([pcd])'''
    data = np.asarray(pcd.points)
    preTime0 = time.time()
    n, Xi, mu, F0, F1, F2, average = preprocess(data)
    preTime1 = time.time()

    print("processingtime:", preTime1-preTime0)
    minError = float("inf")
    Res1 = 50
    Res2 = 50

    for i in range(Res1):
        phi = np.pi/2 * i/Res1
        csphi = np.cos(phi)
        snphi = np.sin(phi)

        for j in range(Res2):
            theta = 2*np.pi*j/Res2
            cstheta = np.cos(theta)
            sntheta = np.sin(theta)
            currentW = [cstheta * snphi, sntheta*snphi, csphi]
            error, rSQR, PC = G(currentW, mu, F0, F1, F2, n)
            if error < minError:
                minError = error
                """Direction Vector"""
                W = currentW
                """Center (when sum(Xi) = 0)"""
                C = PC
                """best radius^2 fitted"""
                bestR = rSQR

    bestR = np.sqrt(bestR)
    C += average
    return bestR


if __name__ == '__main__':


    rootdir = "C:/Users/olive/Desktop/Project 6/TestData/OneDrive/beton200/beton200"
    fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(3, 2, sharex='all', sharey='all')
    plotData = []

    for subdir, dirs, files in os.walk(rootdir):
        radii = []
        for k, file in enumerate(files):
            print(k + 1, "out of", len(files) + 1)
            r = CylinderFitting(subdir, file)
            print("radius:", r)
            radii.append(r)

        if(len(radii) > 0):
            plotData.append(radii)

    ax1.plot(plotData[0])
    ax2.plot(plotData[1])
    ax3.plot(plotData[2])
    ax4.plot(plotData[3])
    ax5.plot(plotData[4])
    ax6.plot(plotData[5])

    ax1.set_title("exp 100")
    ax2.set_title("exp 200")
    ax3.set_title("exp 300")
    ax4.set_title("exp 400")
    ax5.set_title("exp 50")
    ax6.set_title("exp 500")
    plt.show()

