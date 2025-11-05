#include "hillClimbing.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <cstdlib> 


// //imported a thread unfortunately so that 'ENTER' key can stop while loop on windows & macOS
#include <thread>
#include <chrono>
#include <atomic>
#include <limits>

using namespace std;

vector<int> kmeans(vector<Coordinate>& points, int k, int max_iterations, vector<Coordinate>& centroids) {
    int n = points.size();
    centroids.clear();
    vector<int> labels(n, -1);
    HillClimbing cluster;

    for (int i = 0; i < k; ++i) {
        centroids.push_back(points[rand() % n]);
    }

    for (int iter = 0; iter < max_iterations; ++iter) {
        bool changed = false;

        for (int i = 0; i < n; ++i) {
            float min_dist = numeric_limits<float>::max();
            int best_cluster = -1;

            for (int j = 0; j < k; ++j) {
                float dist = cluster.computeEuclideanDistance(points[i], centroids[j]);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_cluster = j;
                }
            }

            if (labels[i] != best_cluster) {
                labels[i] = best_cluster;
                changed = true;
            }
        }

        if (!changed) break;

        vector<Coordinate> new_centroids(k, {0.0f, 0.0f});
        vector<int> counts(k, 0);

        for (int i = 0; i < n; ++i) {
            new_centroids[labels[i]].x += points[i].x;
            new_centroids[labels[i]].y += points[i].y;
            counts[labels[i]]++;
        }

        for (int j = 0; j < k; ++j) {
            if (counts[j] > 0) {
                new_centroids[j].x /= counts[j];
                new_centroids[j].y /= counts[j];
            }
        }

        centroids = new_centroids;
    }

    return labels;
}

vector<Drone> TotalDronePaths(int droneNum, vector<Coordinate>& total_coordinates, HillClimbing& hc, vector<Coordinate>& temp_centroids) {
    vector<Drone> finalAllDrones;
    temp_centroids.clear();
    vector<vector<Coordinate>> allPaths(droneNum);
    float temp_distance = numeric_limits<float>::max();
    vector<int> labels = kmeans(total_coordinates, droneNum, 100, temp_centroids);

    for (int i =0; i<labels.size(); i++) { 
        allPaths[labels[i]].push_back(total_coordinates[i]);
    }

    for (int j =0; j<allPaths.size(); j++){
        vector<Coordinate> current_path_coordinates = hc.restartPath(allPaths[j]);
        vector<Coordinate> temp_best_path = hc.makingPath(current_path_coordinates, temp_distance);

        Drone drone(temp_best_path, temp_centroids[j], temp_distance);
        finalAllDrones.push_back(drone);

        // resetting temp variables for next drone
        temp_distance = numeric_limits<float>::max();
    }

    return finalAllDrones;

}

int main(){
    /* Variable Declariables */
    //input the data into a vector 
    vector<Coordinate> total_coordinates;

    //vector storing the final path of coordinates
    vector<Coordinate> final_path_coordinates;

    //vector storing current working-on path of coordinates
    vector<Coordinate> current_path_coordinates;

    //directly put in the file (x y) coords into temp variables
    float temp_x_coord;
    float temp_y_coord;

    //variable for the inputting the file
    string input_file;
    


    /* Input File Data */

    //take in file of coordinates as input (use locations.txt as example input file)
    cout << "Enter the name of file: ";
    cin >> input_file;

    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    //set the file name from commandline to file name variable
    //filename = input_file;

    //open the file + error handling if file can't open
    ifstream file(input_file);

    if(!file.is_open()){
        cout << "Errors occured when opening the file: \"" << input_file << endl;
        
        //second attempt to get correct file name
        cout << "Enter the correct file name. Make sure it has '.txt' at the end." << endl;
        cin >> input_file;
        file.open(input_file);

        if(!file.is_open()){
            cout << "Second attempt to open file failed. Exiting program." << endl;
            return 1;
        }
    }

    //check if file is empty. If it is, exit program
    if (file.peek() == ifstream::traits_type::eof()) {
        cout << "The file is empty. Exiting program." << endl;
        return 1;
    }


    //each line in the file goes into a temp x & y coord
    while(file >> temp_x_coord >> temp_y_coord){
        //make a coordinate pair with the file coordinates
        Coordinate coord_pair = {temp_x_coord,temp_y_coord};
        total_coordinates.push_back(coord_pair);
    }

      // checks number of locations does not exceed 4026
    int num_locations = total_coordinates.size();
    if (num_locations > 4026) {
        cout << "Error. Number of locations exceeds 4026." << endl;
        return 1;
    }

    //close the file
    file.close();



    /* Run Main program */
    HillClimbing hc; //hill climbing object

    hc.ComputeMatrix(total_coordinates);
    hc.LoadHashmap(total_coordinates);



    // starting print statement 
    cout << "There are " << total_coordinates.size() << " nodes: Solutions will be available by DON'T FORGET TO FIGURE OUT" << endl;

    // temperary variables for drone object
    float temp_distance = numeric_limits<float>::max();

    Coordinate temp_centroid = {0.0, 0.0};
    vector<Coordinate> temp_centroids;
    vector<Coordinate> temp_best_path;

    //-------------doing drone 1 path outside for loop --------------------
    current_path_coordinates = hc.restartPath(total_coordinates);

    
    // testing path's total distance and trying to swap to find better path
    temp_best_path = hc.makingPath(current_path_coordinates, temp_distance);

    float sumX = 0.0;
    float sumY = 0.0;
    for (int i =0; i<temp_best_path.size(); i++) {
        sumX += temp_best_path[i].x;
        sumY += temp_best_path[i].y;
    }

    temp_centroid = {sumX / temp_best_path.size(), sumY / temp_best_path.size()};
    Drone drone1(temp_best_path, temp_centroid, temp_distance);
     
    // resetting temp variables for next drone
    temp_distance = numeric_limits<float>::max();
    temp_centroid = {0.0, 0.0};
    temp_best_path.clear();

    cout << "1) If you are using 1 drone(s), the total route will be " << (ceil(drone1.distance)) << " meters" << endl;
    cout << "     i. Landing pad 1 should be at [" << drone1.centroid.x << ", " << drone1.centroid.y << "], serving " << drone1.path.size() << " locations, route is " << (ceil(drone1.distance)) << " meters" << endl;
    //----------------------------------------------------------------------

    //-------------doing 2 drones --------------------
    vector<Drone> drone2 = TotalDronePaths(2, total_coordinates, hc, temp_centroids);
    float sumDistance = drone2[0].distance + drone2[1].distance;
    cout << "2) If you are using 2 drone(s), the total route will be " << (ceil(sumDistance)) << " meters" << endl;
    cout << "     i. Landing pad 1 should be at [" << drone2[0].centroid.x << ", " << drone2[0].centroid.y << "], serving " << drone2[0].path.size() << " locations, route is " << (ceil(drone2[0].distance)) << " meters" << endl;
    cout << "     ii. Landing pad 2 should be at [" << drone2[1].centroid.x << ", " << drone2[1].centroid.y << "], serving " << drone2[1].path.size() << " locations, route is " << (ceil(drone2[1].distance)) << " meters" << endl;
    //--------------------------------------

    //-------------doing 3 drones --------------------
    vector<Drone> drone3 = TotalDronePaths(3, total_coordinates, hc, temp_centroids);
    sumDistance = drone3[0].distance + drone3[1].distance + drone3[2].distance;
    cout << "3) If you are using 3 drone(s), the total route will be " << (ceil(sumDistance)) << " meters" << endl;
    cout << "     i. Landing pad 1 should be at [" << drone3[0].centroid.x << ", " << drone3[0].centroid.y << "], serving " << drone3[0].path.size() << " locations, route is " << (ceil(drone3[0].distance)) << " meters" << endl;
    cout << "     ii. Landing pad 2 should be at [" << drone3[1].centroid.x << ", " << drone3[1].centroid.y << "], serving " << drone3[1].path.size() << " locations, route is " << (ceil(drone3[1].distance)) << " meters" << endl;
    cout << "     iii. Landing pad 3 should be at [" << drone3[2].centroid.x << ", " << drone3[2].centroid.y << "], serving " << drone3[2].path.size() << " locations, route is " << (ceil(drone3[2].distance)) << " meters" << endl;
    //--------------------------------------

    //-------------doing 4 drones --------------------
    vector<Drone> drone4 = TotalDronePaths(4, total_coordinates, hc, temp_centroids);
    sumDistance = drone4[0].distance + drone4[1].distance + drone4[2].distance + drone4[3].distance;
    cout << "4) If you are using 4 drone(s), the total route will be " << (ceil(sumDistance)) << " meters" << endl;
    cout << "     i. Landing pad 1 should be at [" << drone4[0].centroid.x << ", " << drone4[0].centroid.y << "], serving " << drone4[0].path.size() << " locations, route is " << (ceil(drone4[0].distance)) << " meters" << endl;
    cout << "     ii. Landing pad 2 should be at [" << drone4[1].centroid.x << ", " << drone4[1].centroid.y << "], serving " << drone4[1].path.size() << " locations, route is " << (ceil(drone4[1].distance)) << " meters" << endl;
    cout << "     iii. Landing pad 3 should be at [" << drone4[2].centroid.x << ", " << drone4[2].centroid.y << "], serving " << drone4[2].path.size() << " locations, route is " << (ceil(drone4[2].distance)) << " meters" << endl;
    cout << "     iv. Landing pad 4 should be at [" << drone4[3].centroid.x << ", " << drone4[3].centroid.y << "], serving " << drone4[3].path.size() << " locations, route is " << (ceil(drone4[3].distance)) << " meters" << endl;
    //--------------------------------------
   
    cout << endl;

    /* WRITING TO COORDINATE FILE */

    // string fileNameCoords = "coords.txt";
    // string imageFileName = input_file.substr(0, input_file.length() - 4) + "_SOLUTION_" + to_string(static_cast<int>(ceil(hc.bestDistance))) + ".png";
    // ofstream outputCoords(fileNameCoords);

    // if (!outputCoords.is_open()) {
    //     cout << "Error opening output coordinate file." << endl;
    //     return 1;
    // }

    // // giving the filename 
    // outputCoords << imageFileName << endl;


    // for (int i = 0; i < final_path_coordinates.size(); i++) {
    //     outputCoords << final_path_coordinates[i].x << " " << final_path_coordinates[i].y << endl;
    // }

    // outputCoords.close();


    /* WRITING TO OUTPUT INDEX FILE */

    // string fileName = input_file.substr(0, input_file.length() - 4) + "_SOLUTION_" + to_string(static_cast<int>(ceil(hc.bestDistance))) + ".txt";
    // ofstream outputFile(fileName);


    // if (!outputFile.is_open()) {
    //     cout << "Error opening output file." << endl;
    //     return 1;
    // }

    // outputFile << "1 ";

    // for (int i = 1; i < total_coordinates.size(); i++) {
    //     auto it = find(total_coordinates.begin(), total_coordinates.end(), hc.bestPath[i]);
    //     if (it != total_coordinates.end()) {
    //         int index = distance(total_coordinates.begin(), it);
    //         outputFile << index + 1 << " ";
    //     }
       
    // }

    // outputFile << "1" << endl;

    // outputFile.close();

    

    // cout << "Route written to disk as " << fileName << endl;
    
    // cout << "Image of route written to disk as " << imageFileName << endl;

    // // command to run our python code
    // string command = "python3 path_image.py";

    // int result = system(command.c_str());



    return 0;


}