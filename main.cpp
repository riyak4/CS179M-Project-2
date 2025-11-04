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

vector<int> kmeans(vector<Coordinate>& points, int k, int max_iterations){
    int n = points.size();
    vector<Coordinate> centroids;
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



        // printing out initial statement only once
    cout << "There are " << total_coordinates.size() << " nodes: Solutions will be available by DON'T FORGET TO FIGURE OUT";

    float temp_distance = numeric_limits<float>::max();
    Coordinate temp_centroid = {0.0, 0.0};
    vector<Coordinate> temp_best_path;


    current_path_coordinates = hc.restartPath(total_coordinates);

    
    // testing path's total distance and trying to swap to find better path
    temp_best_path = hc.makingPath(current_path_coordinates, temp_distance);

    float sumX;
    float sumY;
    for (const auto& coord : hc.bestPath) {
        sumX += coord.x;
        sumY += coord.y;
    }

    temp_centroid = {sumX / hc.bestPath.size(), sumY / hc.bestPath.size()};
    Drone drone1(hc.bestPath, temp_centroid, temp_distance);
    cout << drone1.centroid.x << ", " << drone1.centroid.y << endl;

    this_thread::sleep_for(chrono::milliseconds(500));

   
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