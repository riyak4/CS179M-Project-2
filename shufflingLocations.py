import random 

filename = input("Enter the filename to read locations from: ")

with open(filename, "r") as f:
    value = f.read().strip().split("\n")

random.shuffle(value)

with open("Shuffled_" + filename, "w") as f:
    for line in value:
        f.write(line + "\n")

print(f"Shuffled locations saved to Shuffled_{filename}")

