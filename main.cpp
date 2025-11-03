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

vector<int> kmeans(vector<Coordinate>& coordinates, int k, int max_iterations){
    int n = coordinates.size(); //size of given vector 
    vector<Coordinate> centroids;  //vector to store centroids
    vector<int> droneAssignment(n, -1);  //vector to store drone # for each point
    HillClimbing cluster; //hill climbing object for distance calculation

    //randomly initializing centroids
    for (int i = 0; i < k; ++i) {
        centroids.push_back(coordinates[rand() % n]);
    }

    //trying to cluster for 100 iterations or until final cluster assignment found
    for (int iter = 0; iter < max_iterations; ++iter) {
        bool changed = false; //flag to check if any point changed its cluster, if it did not, we break and move on 

        //for looping each point and group them to the nearest centroid/drone
        for (int i = 0; i < n; ++i) {
            float min_dist = numeric_limits<float>::max();
            int best_cluster = -1;

            for (int j = 0; j < k; ++j) {
                float dist = cluster.computeEuclideanDistance(coordinates[i], centroids[j]);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_cluster = j;
                }
            }

            if (droneAssignment[i] != best_cluster) {
                droneAssignment[i] = best_cluster;
                changed = true;
            }
        }

        if (!changed) break; //if cluster did not change, break out of loop

        vector<Coordinate> new_centroids(k, {0.0f, 0.0f});
        vector<int> counts(k, 0);

        //this and next for loop to recalculate centroids using mean of assigned locations
        for (int i = 0; i < n; ++i) {
            new_centroids[droneAssignment[i]].x += coordinates[i].x;
            new_centroids[droneAssignment[i]].y += coordinates[i].y;
            counts[droneAssignment[i]]++;
        }

        for (int j = 0; j < k; ++j) {
            if (counts[j] > 0) {
                new_centroids[j].x /= counts[j];
                new_centroids[j].y /= counts[j];
            }
        }

        centroids = new_centroids;
    }

    return droneAssignment; //returning vector of assigned drones for each point
}

int main(){
    //variable for 'Enter' command to stop the while loop
    atomic<bool> stop_loop{false};

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

      // checks number of locations does not exceed 256
    int num_locations = total_coordinates.size();
    if (num_locations > 256) {
        cout << "Error. Number of locations exceeds 256." << endl;
        return 1;
    }

    //close the file
    file.close();



    /* Run Main program */
    HillClimbing hc; //hill climbing object



    // clear input buffer before starting thread to capture 'ENTER' key
    cin.clear();
    cin.sync();

    // thread tracker_thread([&]{
    //     string dummy;
    //     getline(cin, dummy);
    //     //tells the main loop to stop
    //     stop_loop.store(true);     
    // });

    //in the while loop, is where the search function will be called and output final distances
    int printInitialStatement = 0;
   
    //creating a path to start with 
    current_path_coordinates = hc.restartPath(total_coordinates);

    // printing out initial statement only once
    if (printInitialStatement == 0) {
        cout << "There are " << current_path_coordinates.size() << " nodes: Solutions will be available by DON'T FORGET TO FIGURE OUT";
        printInitialStatement = 1;
    }

    
    // testing path's total distance and trying to swap to find better path
    hc.makingPath(current_path_coordinates, final_path_coordinates);

    for (int i = 1; i < 4; i++) { //each loop = different number of drones 
        vector<int> assignments = kmeans(total_coordinates, i, 100);
        float total_distance = 0.0f;
        if (i == 1) {
            total_distance = hc.bestDistance;
        }
        cout << i << ") If you use " << i << " drone(s), the total route will be _______ meters" << endl; //GET TOTAL DISTANCE OF ALL DRONES' PATHS
        //calculate the distance of each group, sum it to printout total route 
        //printing final route stuff for each drone #
    }

    // this_thread::sleep_for(chrono::milliseconds(500));
    


    // if (tracker_thread.joinable()) tracker_thread.join();

   
    cout << endl;

    /* WRITING TO COORDINATE FILE */

    string fileNameCoords = "coords.txt";
    string imageFileName = input_file.substr(0, input_file.length() - 4) + "_SOLUTION_" + to_string(static_cast<int>(ceil(hc.bestDistance))) + ".png";
    ofstream outputCoords(fileNameCoords);

    if (!outputCoords.is_open()) {
        cout << "Error opening output coordinate file." << endl;
        return 1;
    }

    // giving the filename 
    outputCoords << imageFileName << endl;


    for (int i = 0; i < final_path_coordinates.size(); i++) {
        outputCoords << final_path_coordinates[i].x << " " << final_path_coordinates[i].y << endl;
    }

    outputCoords.close();


    /* WRITING TO OUTPUT INDEX FILE */

    string fileName = input_file.substr(0, input_file.length() - 4) + "_SOLUTION_" + to_string(static_cast<int>(ceil(hc.bestDistance))) + ".txt";
    ofstream outputFile(fileName);


    if (!outputFile.is_open()) {
        cout << "Error opening output file." << endl;
        return 1;
    }

    outputFile << "1 ";

    for (int i = 1; i < total_coordinates.size(); i++) {
        auto it = find(total_coordinates.begin(), total_coordinates.end(), hc.bestPath[i]);
        if (it != total_coordinates.end()) {
            int index = distance(total_coordinates.begin(), it);
            outputFile << index + 1 << " ";
        }
       
    }

    outputFile << "1" << endl;

    outputFile.close();

    

    cout << "Route written to disk as " << fileName << endl;
    
    cout << "Image of route written to disk as " << imageFileName << endl;

    // command to run our python code
    string command = "python3 path_image.py";

    int result = system(command.c_str());



    return 0;


}