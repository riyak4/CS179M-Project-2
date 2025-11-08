import random

num_points = 200
points = []

for _ in range(num_points):
    edge = random.choice(['bottom', 'top', 'left', 'right'])
    
    if edge == 'bottom':  # y = 0
        x = random.uniform(88, 89)
        y = 88.0
    elif edge == 'top':   # y = 1
        x = random.uniform(88, 89)
        y = 89.0
    elif edge == 'left':  # x = 0
        x = 88.0
        y = random.uniform(88, 89)
    else:                 # right, x = 1
        x = 89.0
        y = random.uniform(88, 89)
    
    points.append((x, y))

# Save to a text file
with open("5Squares.txt", "w") as f:
    for x, y in points:
        f.write(f"{x} {y}\n")

print(f"{num_points} random perimeter coordinates saved")
