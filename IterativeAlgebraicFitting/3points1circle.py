import numpy as np
import math
import os
import matplotlib.pyplot as plt



def define_circle(p1, p2, p3):
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
    return ((cx, cy), radius)


# def polarToCircle(distance,angles,plot):
    # points = []
    # for i in range(3):
    #     x = distance[i]*np.cos(angles[i]*math.pi/180)
    #     y = distance[i]*np.sin(angles[i]*math.pi/180)
    #     points.append([x, y])


    # center = []
    # center, radius = define_circle(points[0], points[1], points[2])

    # if center is not None and plot:
    #     plt.figure(figsize=(10, 10))
    #     plt.ylim(top=2,bottom=-2)
    #     plt.xlim(right=2,left=-2)
    #     plt.grid()
    #     for i in range(3):
    #         plt.plot(points[i][0],points[i][1],'bo')
    #     circle = plt.Circle(center, radius)
    #     plt.gcf().gca().add_artist(circle)
    #     plt.show()

    # elif center is None:
    #     print("ERROR! 3 points might be on line")

    # return (radius)



if __name__ == "__main__":

    rootdir = "sensor_readings"
    radii = []

    with open(rootdir+"/sonar_calculated_radius","w") as outf:
        for subdir, dirs, files in os.walk(rootdir):
            for k, file in enumerate(files):
                if len(files) != k+1:
                    with open(subdir+"/"+file) as f:
                        sensorRight = []
                        sensorMiddle = []
                        sensorLeft = []
                        center = []

                        lines = f.readlines()

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

                        center, radius = define_circle(sensorRight,sensorMiddle,sensorLeft)
                        radii.append(radius)

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







    # angles = [-30,0,30]


    # with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as serial_port:
    #     serial_port.reset_input_buffer()

    #     while True:
    #         read_line = serial_port.read()
    #         if read_line != b'\xff':
    #             continue

    #         d = []
    #         byte_array = serial_port.read(24)

    #         serial_port.reset_input_buffer()

    #         d.append(struct.unpack('<f', byte_array[0:4]))
    #         d.append(struct.unpack('<f', byte_array[4:8]))
    #         d,append(struct.unpack('<f', byte_array[8:12]))

    #         radius = polarToCircle(d,angles,False)
    #         print(radius)