import random

num_points = 64
points = []

for _ in range(num_points):
    edge = random.choice(['bottom', 'top', 'left', 'right'])
    
    if edge == 'bottom':  # y = 0
        x = random.uniform(10, 11)
        y = 10.0
    elif edge == 'top':   # y = 1
        x = random.uniform(10, 11)
        y = 11.0
    elif edge == 'left':  # x = 0
        x = 10.0
        y = random.uniform(10, 11)
    else:                 # right, x = 1
        x = 11.0
        y = random.uniform(10, 11)
    
    points.append((x, y))

# Save to a text file
with open("All3Squares.txt", "w") as f:
    for x, y in points:
        f.write(f"{x} {y}\n")

print(f"{num_points} random perimeter coordinates saved")
