import random

num_points = 400
points = []

for _ in range(num_points):
    edge = random.choice(['bottom', 'top', 'left', 'right'])
    
    if edge == 'bottom':  # y = 0
        x = random.uniform(179, 180)
        y = 179.0
    elif edge == 'top':   # y = 1
        x = random.uniform(179, 180)
        y = 180.0
    elif edge == 'left':  # x = 0
        x = 179.0
        y = random.uniform(179, 180)
    else:                 # right, x = 1
        x = 180.0
        y = random.uniform(179, 180)
    
    points.append((x, y))

# Save to a text file
with open("400points.txt", "w") as f:
    for x, y in points:
        f.write(f"{x} {y}\n")

print(f"{num_points} random perimeter coordinates saved")
