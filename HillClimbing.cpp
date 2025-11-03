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
  shuffle(newpath.begin(), newpath.end(), g);

  // // append the landing pad coord in to circle back
  // newpath.push_back({total[0].x, total[0].y});

  return newpath;
}

// creating the path - includes swapping, restarting 
vector<Coordinate> HillClimbing::makingPath(vector<Coordinate> newpath, vector<Coordinate>& finalPath) { 
  // creating distance matrix and hashmap
  ComputeMatrix(newpath);
  LoadHashmap(newpath);



  float distance = getTotalDistance(newpath);
  if (distance < bestDistance) {
    bestDistance = distance;
    bestPath.clear();
    for ( int i = 0; i < newpath.size() ; i++) {
      bestPath.push_back({newpath[i].x, newpath[i].y});
    }
    finalPath = bestPath; //saving to global variable for image 

    // cout << "          " << bestDistance << endl;
  }

  
  // swapping 
  int pathSize = newpath.size();

  bool improved = true;
  while (improved) {
    improved = false;
    // 2 for loops to go through each pair for swapping, using 2-opt loop
    for (int i = 0; i < pathSize - 1; i++) {
      for (int j = i + 1; j < pathSize; j++) {

        //Check if stop signal received
        // if(stop_loop.load()) {
        //   return newpath;
        // }
        reverse(newpath.begin() + i, newpath.begin() + j + 1);
        distance = getTotalDistance(newpath);
        if (distance < bestDistance) {
          bestDistance = distance;
          //cout << "          " << bestDistance << endl;
          //updating new best path
          bestPath.clear();
          for ( int k = 0; k < newpath.size() ; k++) {
            bestPath.push_back({newpath[k].x, newpath[k].y});
          }
          finalPath = bestPath; //saving to global variable for image 
          improved = true;
        } else {
          // swap back if no improvement
          reverse(newpath.begin() + i, newpath.begin() + j + 1);
        }
      }
    }
  }
  
  cout << "the total route will be " << bestDistance << endl;
  return newpath;
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