import random

num_points = 64
points = []

for _ in range(num_points):
    edge = random.choice(['bottom', 'top', 'left', 'right'])
    
    if edge == 'bottom':  # y = 0
        x = random.random()
        y = 3.0
    elif edge == 'top':   # y = 1
        x = random.random()
        y = 4.0
    elif edge == 'left':  # x = 0
        x = 3.0
        y = random.random()
    else:                 # right, x = 1
        x = 4.0
        y = random.random()
    
    points.append((x, y))

# Save to a text file
with open("2ndSquare.txt", "w") as f:
    for x, y in points:
        f.write(f"{x} {y}\n")

print(f"{num_points} random perimeter coordinates saved")
