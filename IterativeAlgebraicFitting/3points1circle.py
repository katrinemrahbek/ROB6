import numpy as np
import math
import os
import matplotlib.pyplot as plt
import statistics as stat



def define_circle(p1, p2, p3, plot):
    """
    Returns the center and radius of the circle passing the given 3 points.
    In case the 3 points form a line, returns (None, infinity).
    """
    temp = p2[0] * p2[0] + p2[1] * p2[1]
    bc = (p1[0] * p1[0] + p1[1] * p1[1] - temp) / 2
    cd = (temp - p3[0] * p3[0] - p3[1] * p3[1]) / 2
    det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])

    if abs(det) < 1.0e-6:
        return (None, np.inf)

    # Center of circle
    cx = (bc*(p2[1] - p3[1]) - cd*(p1[1] - p2[1])) / det
    cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det

    radius = np.sqrt((cx - p1[0])**2 + (cy - p1[1])**2)

    if plot:
        plt.figure(figsize=(6, 6))
        plt.ylim(top=0.025,bottom=-0.075)
        plt.xlim(right=0.05,left=-0.05)
        plt.grid()
        plt.plot(p1[0],p1[1],'bo')
        plt.plot(p2[0],p2[1],'bo')
        plt.plot(p3[0],p3[1],'bo')
        circle = plt.Circle((cx,cy), radius)
        plt.gcf().gca().add_artist(circle)
        plt.show()

    return ((cx, cy), radius)


if __name__ == "__main__":

    rootdir = "sensor_readings"
    outputFile = "sonar_calculated_radius.txt"
    plot = False

    for datasets in os.listdir(rootdir):
        dataset_dir = os.path.join(rootdir, datasets)
        radii = []
        rightDepth = []
        middleDepth = []
        leftDepth = []
        with open(os.path.join(dataset_dir, outputFile), 'w') as outf:
            files = os.listdir(dataset_dir)
            for k, file in enumerate(files):
                with open(os.path.join(dataset_dir,file),'r') as f:
                    if not file.endswith(".txt") or file == outputFile:
                        if not file == outputFile:
                            print(file,"did not comply")
                        continue
                    lines = f.readlines()
                    #print("doing:",file)

                    sensorRight = []
                    sensorMiddle = []
                    sensorLeft = []
                    center = []
                    sensorData = []

                    cutendl = lines[3].split("\n")
                    sensorRightString = cutendl[0].split(",")
                    for i in range(3):
                        sensorRight.append(float(sensorRightString[i]))

                    cutendl = lines[5].split("\n")
                    sensorMiddleString = cutendl[0].split(",")
                    for i in range(3):
                        sensorMiddle.append(float(sensorMiddleString[i]))

                    cutendl = lines[7].split("\n")
                    sensorLeftString = cutendl[0].split(",")
                    for i in range(3):
                        sensorLeft.append(float(sensorLeftString[i]))

                    center, radius = define_circle(sensorRight,sensorMiddle,sensorLeft,plot)
                    radii.append(radius)

                    cutendl = lines[1].split("\n")
                    sensorDataString = cutendl[0].split(",")
                    for i in range(6):
                        sensorData.append(float(sensorDataString[i]))
                    rightDepth.append(sensorData[0])
                    middleDepth.append(sensorData[1])
                    leftDepth.append(sensorData[2])
                        

            meanR = sum(r for r in radii)/len(radii)
            sumr = 0
            for j in range(len(radii)):
                sumr += (radii[j]-meanR)**2
            if len(radii)-1 != 0:
                std = np.sqrt(sumr/(len(radii)-1))
            else:
                std = 0

            outf.write(str(rootdir)+", r: "+str(meanR)+", std_dev: "+str(std)+"\n")

            for j in range(len(radii)):
                outf.write(str(radii[j])+"\n")

        # visualization
        mean_right = stat.mean(rightDepth)
        mean_middle = stat.mean(middleDepth)
        mean_left = stat.mean(leftDepth)
        std_right = stat.stdev(rightDepth)
        std_middle = stat.stdev(middleDepth)
        std_left = stat.stdev(leftDepth)

        print("Mean Right:", mean_right)
        print("Mean middle:", mean_middle)
        print("Mean left:", mean_left)
        print("Standard deviation right:", std_right)
        print("Standard deviation middle:", std_middle)
        print("Standard deviation left:", std_left)

        fig, (ax1,ax2,ax3) = plt.subplots(3, 1, sharey=True)
        fig.suptitle(str(datasets))
        ax1.plot(rightDepth)
        ax1.set_title('Right Depth')
        ax2.plot(middleDepth)
        ax2.set_title('Middle Depth')
        ax3.plot(leftDepth)
        ax3.set_title('Left Depth')
        plt.show()
