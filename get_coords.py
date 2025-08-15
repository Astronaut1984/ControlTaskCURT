import numpy as np
radius = 5

points = []

for i in range(0, 361, 10):
    x = round(float(radius * np.cos(np.deg2rad(i))), 3)
    y = round(float(radius * np.sin(np.deg2rad(i))), 3)
    point = (x, y)
    points.append(point)

print(points)