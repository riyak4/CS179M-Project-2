import random
import math

num_points = 200
points = []

cx, cy = 1.0, 1.0   # center
r = 1.0             # radius

for _ in range(num_points):
    angle = random.uniform(0, 2 * math.pi)  # random angle 0–360°
    
    x = cx + r * math.cos(angle)
    y = cy + r * math.sin(angle)

    points.append((x, y))

# Save to file
with open("Circle1_1.txt", "w") as f:
    for x, y in points:
        f.write(f"{x} {y}\n")

print(f"{num_points} random circular perimeter points saved.")
