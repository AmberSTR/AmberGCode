import csv
import math
import time
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np

UP_SAMPLE_RATIO = 200
DOWN_SAMPLE_RATIO = 20
INIT_POSITION_DELAY = 500
class SavedTrajectory:
    x = []
    y = []
    G00_x = []
    G00_y = []
    d = []
    Ux = []
    Uy = []
    Dx = []
    Dy = []


def diag_length(x1, y1, x2, y2):
    l = abs(x1 - x2)
    w = abs(y1 - y2)
    d = math.hypot(l, w)
    return d


def UP_Sample(x1, y1, x2, y2):
    SavedTrajectory.Ux.append(x1)
    SavedTrajectory.Uy.append(y1)
    d = diag_length(x1, y1, x2, y2)
    slice = int(d * UP_SAMPLE_RATIO)
    dx = x2 - x1
    dy = y2 - y1
    for i in range(slice):
        x1 += dx / slice
        y1 += dy / slice
        SavedTrajectory.Ux.append(x1)
        SavedTrajectory.Uy.append(y1)


def Down_Sample():
    for i in range(len(SavedTrajectory.Ux) - 1):
        if i % DOWN_SAMPLE_RATIO == 0:
            SavedTrajectory.Dx.append(SavedTrajectory.Ux[i])
            SavedTrajectory.Dy.append(SavedTrajectory.Uy[i])


def Sampler():
    print(f"Recalculating route.. ")
    for i in range(len(SavedTrajectory.x) - 1):
        UP_Sample(SavedTrajectory.x[i], SavedTrajectory.y[i], SavedTrajectory.x[i + 1], SavedTrajectory.y[i + 1])
        d = diag_length(SavedTrajectory.x[i], SavedTrajectory.y[i], SavedTrajectory.x[i + 1], SavedTrajectory.x[i + 1])
        SavedTrajectory.d.append(d)
    min_d = 1000
    max_d = 0
    for item in SavedTrajectory.d:
        if item < min_d:
            min_d = item
        if item > max_d:
            max_d = item
    print(f"MIN:{min_d}")
    print(f"MAX:{max_d}")
    max_y = 0
    max_x = 0

    Down_Sample()
    indexX = 0
    indexY=0
    for item in SavedTrajectory.Dx:
        SavedTrajectory.Dx[indexX]= item/100
        if SavedTrajectory.Dx[indexX] > max_x:
            max_x = SavedTrajectory.Dx[indexX]
        indexX+=1
    for item in SavedTrajectory.Dy:
        SavedTrajectory.Dy[indexY]= item/100
        if SavedTrajectory.Dy[indexY] > max_y:
            max_y = SavedTrajectory.Dy[indexY]
        indexY+=1
    print(f"MAX_X:{max_x}")
    print(f"MAX_Y:{max_y}")
    t = np.arange(1, len(SavedTrajectory.Dx) + 1, 1)
    # plt.scatter(SavedTrajectory.Ux, SavedTrajectory.Uy, s=1,c=t)
    plt.scatter(SavedTrajectory.Dx, SavedTrajectory.Dy, s=10, c=t)
    # plt.plot(SavedTrajectory.Ux, SavedTrajectory.Uy)
    print(f"Original:    {len(SavedTrajectory.x)}")
    print(f"Up sample:   {len(SavedTrajectory.Ux)}")
    print(f"Down sample: {len(SavedTrajectory.Dx)}")
    plt.show()


def SaveG01(x, y):
    SavedTrajectory.x.append(x)
    SavedTrajectory.y.append(y)


def SaveG00(x, y):
    SavedTrajectory.G00_x.append(x)
    SavedTrajectory.G00_y.append(y)
    SaveG01(x, y)


with open('gcode.gcode') as f:
    LineCount = 0
    lines = f.readlines()
    for operation in lines:
        # TODO if operation.find('G21') == 0:
        # TODO if operation.find('G90') == 0:
        if operation.find('G1') == 0:
            LineCount += 1
            if operation.find('S') == -1:
                x = operation[operation.find('X') + 1:operation.find('Y') - 1]
                y = operation[operation.find('Y') + 1:len(operation) - 1]
            else:
                x = operation[operation.find('X') + 1:operation.find('Y') - 1]
                y = operation[operation.find('Y') + 1:operation.find('S') - 1]
                s = operation[operation.find('S') + 1:operation.find('F') - 1]
                f = operation[operation.find('F') + 1:len(operation) - 1]
            SaveG01(float(x), float(y))
        if operation.find('G0') == 0:
            LineCount += 1
            x = operation[operation.find('X') + 1:operation.find('Y') - 1]
            y = operation[operation.find('Y') + 1:len(operation) - 1]
            SaveG00(float(x), float(y))
Sampler()
rowindex = 0
timenow = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
with open(f"XY.csv", 'w', encoding='utf-8', newline='') as csv_file_W:
    writer = csv.writer(csv_file_W)
    for i in range (INIT_POSITION_DELAY):
        writer.writerow([f"{SavedTrajectory.Dx[0]}",f"{SavedTrajectory.Dy[0]}"])
    for item in SavedTrajectory.Dx:

        writer.writerow([f"{SavedTrajectory.Dx[rowindex]}",f"{SavedTrajectory.Dy[rowindex]}"])
        rowindex+=1