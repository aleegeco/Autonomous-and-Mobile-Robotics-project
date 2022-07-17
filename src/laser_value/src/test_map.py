import numpy as np
import cv2
from matplotlib import pyplot as plt
from visualizer import visibility
# %matplotlib inline

fn = '/home/marco/Documents/AMR/turtlebot_project_ws/map_02.pgm'
#fn = 'virtual_map.pgm'
# img1 = cv2.imread(fn, cv2.COLOR_BGR2GRAY)

img = cv2.imread(fn, cv2.IMREAD_GRAYSCALE)
image = img.copy()
plot = 0
if plot:
    #plt.figure(figsize=(15,10))
    #plt.imshow(image, cmap='gray',vmin=0, vmax=255)
    #plt.show()
    cv2.imshow('house', image)
    cv2.waitKey(0)

centre = [67, 33]
# centre = [267, 135]
image[centre[1], centre[0]] = 1



# sensor_output = 3 * np.random.rand(360)
sensor_output = np.ones(360)
sensor_output[0:45] = 6
resolution = 0.05
sensor_output_discrete = sensor_output / resolution
sensor_output_discrete = sensor_output_discrete.astype(int)
'''
for l in range(10):
    for i in range(20):
            x = round(i * np.cos(l))
            y = round(i * np.sin(l))
            if l <= 90:
                image[centre[1]-y, centre[0]-x] = 30
            elif l > 90 and l<=180:
                image[centre[1]-y, centre[0]+x] = 30
            elif l > 180 and l<=270:
                image[centre[1]+y, centre[0]+x] = 30
            elif l > 270 and l<=360:
                image[centre[1]+y, centre[0]-x] = 30

'''
'''
for l in range(len(sensor_output_discrete)):
    if sensor_output_discrete[l] == 120:
        d = 1
        while True:
            x = round(d * np.cos(l))
            y = round(d * np.sin(l))
            if l <= 90 and image[centre[1]-y, centre[0]-x] > 0:
                image[centre[1]-y, centre[0]-x] = 30
            elif l > 90 and l<=180 and image[centre[1]-y, centre[0]-x] > 0:
                image[centre[1]-y, centre[0]+x] = 30
            elif l > 180 and l<=270 and image[centre[1]+y, centre[0]+x] > 0:
                image[centre[1]+y, centre[0]+x] = 30
            elif l > 270 and l<=360 and image[centre[1]+y, centre[0]+x] > 0:
                image[centre[1]+y, centre[0]-x] = 30
            else:
                break
            d += 1
    else:
        for i in range(1, sensor_output_discrete[l]):
            x = round(i * np.cos(l))
            y = round(i * np.sin(l))
            if l <= 90:
                image[centre[1]-y, centre[0]-x] = 30
            elif l > 90 and l<=180:
                image[centre[1]-y, centre[0]+x] = 30
            elif l > 180 and l<=270:
                image[centre[1]+y, centre[0]+x] = 30
            elif l > 270 and l<=360:
                image[centre[1]+y, centre[0]-x] = 30
'''

for l in range(360):
    d = 1
    while True:
        x = int(round(d * np.cos(l)))
        y = int(round(d * np.sin(l)))
        if l <= 90 and image[(centre[1] - y), (centre[0] - x)] > 0:
            image[centre[1] - y, centre[0] - x] = 80
        elif l > 90 and l <= 180 and image[centre[1] - y, centre[0] + x] > 0:
            image[centre[1] - y, centre[0] + x] = 80
        elif l > 180 and l <= 270 and image[centre[1] + y, centre[0] + x] > 0:
            image[centre[1] + y, centre[0] + x] = 80
        elif l > 270 and l <= 360 and image[centre[1] + y, centre[0] - x] > 0:
            image[centre[1] + y, centre[0] - x] = 80
        else:
            break
        d += 1


# image = visibility(image, centre)

plot = 1
if plot:
    plt.figure(figsize=(15,10))
    plt.imshow(image, cmap='gray',vmin=0, vmax=255)
    plt.show()
    # cv2.imshow('house', image)
    # cv2.waitKey(0)


