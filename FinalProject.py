import serial
import time
import numpy as np
import pygame
import math
from pygame.locals import (
    K_ESCAPE,
    KEYDOWN,
)

ser = serial.Serial(port='COM3',baudrate=115200,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=0)

c = 0

scale = 20

color = [0,0,255]
lightSource = np.matrix([4, 7, 10])  # x, y, z
cameraLocation = np.matrix([6, -10, 0])  # x, y, z
cameraPointing = np.matrix([0, 1, 0])  # x, y, z
objectLocation = [0, 0, 0]  # x, y, z
xRotation = 0  # degrees
yRotation = 0  # degrees
zRotation = 0  # degrees
fieldOfView = 120
nearFrustum = -50  # z
farFrustum = 80  # z
M_diff = [.259, .29, .322, 1]  # r g b a

up = np.matrix([0, -1, 0])
screenheight = 1000
screenwidth = 1000

lightSource = lightSource / (np.linalg.norm(lightSource))

# Rotation Matrices Creation
xRotMatrix = np.matrix([
    [1, 0, 0, 0],
    [0, round(math.cos(math.radians(xRotation)), 5), round(math.sin(math.radians(xRotation)), 5), 0],
    [0, round(-math.sin(math.radians(xRotation)), 5), round(math.cos(math.radians(xRotation)), 5), 0],
    [0, 0, 0, 1]
], dtype=float)

yRotMatrix = np.matrix([
    [round(math.cos(math.radians(yRotation)), 5), 0, round(-math.sin(math.radians(yRotation)), 5), 0],
    [0, 1, 0, 0],
    [round(math.sin(math.radians(yRotation)), 5), 0, round(math.cos(math.radians(yRotation)), 5), 0],
    [0, 0, 0, 1]
], dtype=float)

zRotMatrix = np.matrix([
    [round(math.cos(math.radians(zRotation)), 5), round(math.sin(math.radians(zRotation)), 5), 0, 0],
    [round(-math.sin(math.radians(zRotation)), 5), round(math.cos(math.radians(zRotation))), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
], dtype=float)
# End Rotation Matrices Creation


# Translation Matrix Creation
translateMatrix = np.matrix([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [int(objectLocation[0]), int(objectLocation[1]), int(objectLocation[2]), 1]
], dtype=float)
# End Translate Matrix Creation

zaxis = (cameraPointing - cameraLocation) / (np.linalg.norm((cameraPointing - cameraLocation)))
xaxis = (np.cross(up, zaxis)) / (np.linalg.norm(np.cross(up, zaxis)))
yaxis = np.cross(zaxis, xaxis)

viewMatrix = np.matrix([
    [xaxis.item(0), yaxis.item(0), zaxis.item(0), 0],
    [xaxis.item(1), yaxis.item(1), zaxis.item(1), 0],
    [xaxis.item(2), yaxis.item(2), zaxis.item(2), 0],
    [-np.dot(xaxis, cameraLocation.T).item(0), -np.dot(yaxis, cameraLocation.T).item(0), -np.dot(zaxis, cameraLocation.T).item(0), 1]
], dtype=float)
# End View Matrix Creation

# Perspective Matrix Creation
FOVy = math.radians(fieldOfView)
aspect = screenheight / screenwidth
zn = nearFrustum
zf = farFrustum
yScale = 1 / (math.tan(FOVy/2))
xScale = yScale

perspectiveMatrix = np.matrix([
    [xScale, 0, 0, 0],
    [0, yScale, 0, 0],
    [0, 0, zf/(zf-zn), 1],
    [0, 0, -zn*zf/(zf-zn), 0]
], dtype=float)

# End Perspective Matrix Creation




def updateCamera():
    cameraLocation = np.matrix([20*math.cos(c), 20*math.sin(c), 0])
    n = np.matrix([-math.cos(c), -math.sin(c), 0], dtype=float)
    cameraPointing = np.matrix([n.item(0), n.item(1), n.item(2)])

    zaxis = (cameraPointing - cameraLocation) / (np.linalg.norm((cameraPointing - cameraLocation)))
    xaxis = (np.cross(up, zaxis)) / (np.linalg.norm(np.cross(up, zaxis)))
    yaxis = np.cross(zaxis, xaxis)

    viewMatrix = np.matrix([
        [xaxis.item(0), yaxis.item(0), zaxis.item(0), 0],
        [xaxis.item(1), yaxis.item(1), zaxis.item(1), 0],
        [xaxis.item(2), yaxis.item(2), zaxis.item(2), 0],
        [-np.dot(xaxis, cameraLocation.T).item(0), -np.dot(yaxis, cameraLocation.T).item(0),
         -np.dot(zaxis, cameraLocation.T).item(0), 1]
    ], dtype=float)
    # End View Matrix Creation

    # Perspective Matrix Creation
    FOVy = math.radians(fieldOfView)
    aspect = screenheight / screenwidth
    zn = nearFrustum
    zf = farFrustum
    yScale = 1 / (math.tan(FOVy / 2))
    xScale = yScale

    perspectiveMatrix = np.matrix([
        [xScale, 0, 0, 0],
        [0, yScale, 0, 0],
        [0, 0, zf / (zf - zn), 1],
        [0, 0, -zn * zf / (zf - zn), 0]
    ], dtype=float)

    # End Perspective Matrix Creation




# pass in an array, and it returns an array of fully pipelined and Z-sorted points
def pipeline(tuples):
    preAnythingMatrices = [] # must be formatted as [[x, y, z], [x, y, z], [x, y, z], [x, y, z],...]
    i = 1
    if len(tuples) == 1:
        return -1
    while i < len(tuples):
        v1 = tuples[i - 1]
        v2 = tuples[i]
        v1 = np.matrix([v1[0], v1[1], v1[2]], dtype=float)
        v2 = np.matrix([v2[0], v2[1], v2[2]], dtype=float)
        cr = np.cross(v1, v2)
        unit = np.linalg.norm(cr) / 100.0
        vert3PreOrtho = np.matrix([
                            (tuples[i - 1][0] + tuples[i][0]) / 2,
                            (tuples[i - 1][1] + tuples[i][1]) / 2,
                            (tuples[i - 1][2] + tuples[i][2]) / 2])
        v3 = np.add(vert3PreOrtho, unit)

        v1 = np.matrix([v1.item(0), v1.item(1), v1.item(2), 1], dtype=float)
        v2 = np.matrix([v2.item(0), v2.item(1), v2.item(2), 1], dtype=float)

        v3 = np.matrix([
            v3.item(0),
            v3.item(1),
            v3.item(2),
            1
        ])
        preAnythingMatrices.append([v1, v2, v3])
        i += 1

    # World Transforms
    afterWorldMatrices = []
    for i in preAnythingMatrices:
        vert1 = i[0] @ xRotMatrix @ yRotMatrix @ zRotMatrix @ translateMatrix
        vert2 = i[1] @ xRotMatrix @ yRotMatrix @ zRotMatrix @ translateMatrix
        vert3 = i[2] @ xRotMatrix @ yRotMatrix @ zRotMatrix @ translateMatrix
        afterWorldMatrices.append([vert1, vert2, vert3])

    afterViewMatrices = []
    # View Transforms
    for i in afterWorldMatrices:
        vert1 = i[0] @ viewMatrix
        vert2 = i[1] @ viewMatrix
        vert3 = i[2] @ viewMatrix
        afterViewMatrices.append([vert1, vert2, vert3])

    afterPerspectiveMatrices = []
    # Perspective Transforms
    for i in afterViewMatrices:
        vert1 = i[0] @ perspectiveMatrix
        vert2 = i[1] @ perspectiveMatrix
        vert3 = i[2] @ perspectiveMatrix
        afterPerspectiveMatrices.append([vert1, vert2, vert3])

    # Z sorting the matrices
    zAverages = []
    for i in afterPerspectiveMatrices:
        vert1 = i[0].item(2)
        vert2 = i[1].item(2)
        vert3 = i[2].item(2)
        zAverages.append((vert1 + vert2 + vert3) / 3)

    zSorting = np.argsort(zAverages)
    zSorting = zSorting.tolist()
    afterPerspectiveMatrices = [afterPerspectiveMatrices[ind] for ind in zSorting]
    afterWorldMatrices = [afterWorldMatrices[ind] for ind in zSorting]

    # Divide by WZ
    for i in afterPerspectiveMatrices:
        if (i[0].item(2) != 0):
            i[0] = np.matrix([i[0].item(0) / i[0].item(2), i[0].item(1) / i[0].item(2), i[0].item(2) / i[0].item(2),
                              i[0].item(3) / i[0].item(2)])
        if (i[1].item(2) != 0):
            i[1] = np.matrix([i[1].item(0) / i[1].item(2), i[1].item(1) / i[1].item(2), i[1].item(2) / i[1].item(2),
                              i[1].item(3) / i[1].item(2)])
        if (i[2].item(2) != 0):
            i[2] = np.matrix([i[2].item(0) / i[2].item(2), i[2].item(1) / i[2].item(2), i[2].item(2) / i[2].item(2),
                              i[2].item(3) / i[2].item(2)])

    afterViewPortMatrices = []
    # ViewPort Transforms
    for i in afterPerspectiveMatrices:
        vert1 = np.matrix([(screenwidth * .5 * i[0].item(0) + (screenwidth * .5)),
                           (screenheight * .5 * i[0].item(1)) + (screenheight * .5), i[0].item(2), i[0].item(3)])

        vert2 = np.matrix([(screenwidth * .5 * i[1].item(0) + (screenwidth * .5)),
                           (screenheight * .5 * i[1].item(1)) + (screenheight * .5), i[1].item(2), i[1].item(3)])

        vert3 = np.matrix([(screenwidth * .5 * i[2].item(0) + (screenwidth * .5)),
                           (screenheight * .5 * i[2].item(1)) + (screenheight * .5), i[2].item(2), i[2].item(3)])

        afterViewPortMatrices.append([vert1, vert2, vert3])

    return afterViewPortMatrices


pointCache = {1:[], 2:[], 3:[]} # each array is a collection of points, that connect to eachother sequentially. Each new
                                # key/value is a break in the lines. Tuple of points.

colors = {1:(255, 0, 0), 2:(255, 0, 0)}

pygame.init()
screen = pygame.display.set_mode([1000, 1000])
running = True

counter = 1

while running:
    try:
        s = ser.readline().decode()
        #print(s)
        if 'X:' in s:
            print(s)
            #print(s[3:9])
            newarr = []
            #print(float(s[3:8]))
            a1 = s.split(": ")
            newarr.append(float(a1[1][0:6]))
            newarr.append(float(a1[2][0:6]))
            newarr.append(float(a1[3][0:6]))
            if counter in pointCache.keys():
                pointCache[counter].append(newarr)
            else:
                pointCache[counter] = [newarr]


        if "@" in s:
            print(s)
            color = (int(s.split("@")[1].split(",")[0]),
                int(s.split("@")[1].split(",")[1]),
                int(s.split("@")[1].split(",")[2]))

            colors[counter] = color

        if 'b' in s:
            counter += 1
        time.sleep(.1)
    except:
        print("fail")
        time.sleep(.1)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == KEYDOWN:
            if event.key == K_ESCAPE:
                running = False
    screen.fill((255, 255, 255))

    for k,v in pointCache.items():
        if len(v) == 0 or len(v) == 1:
            continue
        afterViewPortMatrices = pipeline(v)
        if afterViewPortMatrices == -1:
            continue
        for i in range(len(afterViewPortMatrices)):

            v1 = (afterViewPortMatrices[i][0].item(0), afterViewPortMatrices[i][0].item(1))
            v2 = (afterViewPortMatrices[i][1].item(0), afterViewPortMatrices[i][1].item(1))
            v3 = (afterViewPortMatrices[i][2].item(0), afterViewPortMatrices[i][2].item(1))
            if v1[0] < 0 or v1[1] < 0:
                continue
            if v2[0] < 0 or v2[0] < 0:
                continue
            if v3[0] < 0 or v3[0] < 0:
                continue

            pygame.draw.polygon(screen, colors[k], (v1, v2, v3))


    pygame.display.flip()
    #c = c + .05
    #updateCamera()

pygame.quit()
