import matplotlib.pyplot as plt
import sys

def path_image():
  # goal : for loop going through every drone (grab this value from file )
  # until the file ends -
  # grab the first value -> that will be the length of the inner for loop
  # until you grab a value that is not a pair
  # grab the next, continue drawing
  # when you hit -1, end

  x_coords = []
  y_coords = []
  colors = ["red", "green", "blue", "purple"]
  colorsIndex = 0

  fileName = sys.argv[1]

  plt.figure(figsize=(12, 12))
  with open('coords.txt', 'r') as file:

    for line in file:
      coords = line.strip().split()
      if len(coords) == 2:
        x_coords.append(float(coords[0]))
        y_coords.append(float(coords[1]))
      else:

        # plotting the first point bigger
        plt.scatter(
            x_coords[0], y_coords[0],
            color='black',          
            edgecolors='black',
            s=200,               
            zorder=3,             
        )

        plt.plot(x_coords, y_coords, color=colors[colorsIndex], marker='o', linestyle='-', linewidth=0.8, alpha=0.7)
        x_coords = []
        y_coords = []
        colorsIndex += 1

      
  plt.xlabel("Distance (meters)")
  plt.ylabel("Distance (meters)")
  plt.savefig(fileName)



if __name__ == '__main__':
  path_image()