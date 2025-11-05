#include "hillClimbing.h"
//#include "Problem.h"
#include <iostream>
#include <algorithm>


vector<Coordinate> HillClimbing::restartPath(vector<Coordinate>& total) {
  vector<Coordinate> newpath;
  for ( int i = 0; i < total.size() ; i++) {
      newpath.push_back({total[i].x, total[i].y});

  }
  // return 0 if empty parameter
  if (newpath.size() == 0) {
    return newpath;
  }

  //srand(time(0));
  random_device rd;
  mt19937 g(rd());
  
  // don't shuffle the landing pad coord in
  shuffle(newpath.begin()+1, newpath.end(), g);
  // append the landing pad coord in to circle back
  newpath.push_back({total[0].x, total[0].y});

  return newpath;
}

// creating the path - includes swapping, restarting 
vector<Coordinate> HillClimbing::makingPath(vector<Coordinate> newpath, float& temp_distance) { 

  float distance = getTotalDistance(newpath);
  if (distance < temp_distance) {
    temp_distance = distance;
    bestPath.clear();
    bestPath = newpath;
  }

  
  // swapping 
  int pathSize = newpath.size();

  
    // 2 for loops to go through each pair for swapping, using 2-opt loop
    for (int i = 1; i < pathSize - 2; i = i + 2) {

      for (int j = i + 1; j < pathSize - 1; j= j+ 2) {

    
        reverse(newpath.begin() + i, newpath.begin() + j + 1);
        distance = getTotalDistance(newpath);
        if (distance < temp_distance) {
          temp_distance = distance;
            //updating new best path
          bestPath.clear();
          bestPath = newpath;
        } else {
          // swap back if no improvement
          reverse(newpath.begin() + i, newpath.begin() + j + 1);
        }

      }
      // cout << "done with i = " << i << endl;
    }

  
 
  return bestPath;
}


//returning total distance of path
float HillClimbing::getTotalDistance(vector<Coordinate>& final_path) {
    float final_dist = 0.0f;
    int row;
    int col;
    for (int i = 0; i < final_path.size()-1; i++) {
        row = coordinate_indexes[final_path[i]];
        col = coordinate_indexes[final_path[i+1]];
        final_dist += distance_matrix[row][col];  
    }
    return final_dist;
}

void HillClimbing::ComputeMatrix(vector<Coordinate>& locations){
  // resizing distance matrix to avoid memory access violations 
  distance_matrix.resize(locations.size(), vector<float>(locations.size(), 0));
  for(int i = 0; i < locations.size(); i++){
      for(int j = i+1; j < locations.size(); j++){
        distance_matrix[i][j] = computeEuclideanDistance(locations[i], locations[j]);
        distance_matrix[j][i] = distance_matrix[i][j];
      }
    
  }
}

void HillClimbing::LoadHashmap(vector<Coordinate>& locations){
    for(int i = 0; i < locations.size();i++){
        coordinate_indexes[locations.at(i)] = i;
    }
}


float HillClimbing::computeEuclideanDistance(Coordinate node1, Coordinate node2){
    float smth_x = abs(node1.x - node2.x);
    float smth_y = abs(node1.y - node2.y);
    float dist = pow(smth_x, 2) + pow(smth_y, 2);
    dist = sqrt(dist);

    return dist;


}