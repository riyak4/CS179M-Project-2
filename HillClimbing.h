#ifndef HILLCLIMBING_H
#define HILLCLIMBING_H

//#include "Problem.h"
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <chrono>
#include <atomic>
#include <unordered_map>


#include <fstream>
#include <sstream>

#include <string>

#include <algorithm>
#include <random>
#include <iostream>


using namespace std;

struct Coordinate {
    float x;
    float y;

    //overload operator for == to work for easier comparisons 
    bool operator==(const Coordinate& point) const {
        return x == point.x && y == point.y;
    }
};

struct CoordinateHash {
    size_t operator()(const Coordinate &c) const {
        size_t h1 = hash<double>{}(c.x);
        size_t h2 = hash<double>{}(c.y);
        return h1 ^ (h2 << 1); // combine hashes
    }
};

class HillClimbing {
    private:
        vector<vector<float>> distance_matrix;
        //hash map 
        unordered_map<Coordinate, int, CoordinateHash> coordinate_indexes; 
    public: 
        //testing distance of path and swapping 
        vector<Coordinate> makingPath(vector<Coordinate> newpath, vector<Coordinate>& finalPath);

        //creating a whole new random path 
        vector<Coordinate> restartPath(vector<Coordinate>& total);

        //returns total distance of given path 
        float getTotalDistance(vector<Coordinate>& final_path);

        //computing the distance matrix
        void ComputeMatrix(vector<Coordinate>& locations);

        //loading the distance hashmap
        void LoadHashmap(vector<Coordinate>& locations);

        //calculating euclidean distance between 2 coordinates
        float computeEuclideanDistance(Coordinate node1, Coordinate node2);

        //public variables to store best path and distance used in main 
        vector<Coordinate> bestPath;
        // vector bestPath1/2  1
        // vector bestPath2/2  2
        // vector bestpath1/3  3
        // vector bestpath2/3  4
        // vector bestpath3/3  5
        // vector bestpath1/4  6
        // vector bestpath2/4  7
        // vector bestpath3/4  8
        // vector bestpath4/4  9

        float bestDistance = numeric_limits<float>::max();
};

#endif