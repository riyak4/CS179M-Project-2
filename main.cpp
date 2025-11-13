#include "HillClimbing.h"
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
#include <iomanip>
#include <atomic>
#include <limits>


// included to create our coordinate visualization

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

    auto now = chrono::system_clock::now();
    auto future = now + chrono::minutes(5);

    time_t future_time = chrono::system_clock::to_time_t(future);

    //each line in the file goes into a temp x & y coord
    while(file >> temp_x_coord >> temp_y_coord){
        //make a coordinate pair with the file coordinates
        Coordinate coord_pair = {temp_x_coord,temp_y_coord};
        total_coordinates.push_back(coord_pair);
    }

      // checks number of locations does not exceed 4026
    int num_locations = total_coordinates.size();
    if (num_locations > 4096) {
        cout << "Error. Number of locations exceeds 4096." << endl;
        return 1;
    }

    //close the file
    file.close();



    /* Run Main program */
    HillClimbing hc; //hill climbing object

    hc.ComputeMatrix(total_coordinates);
    hc.LoadHashmap(total_coordinates);



    // starting print statement 
    cout << "There are " << total_coordinates.size() << " nodes: Solutions will be available by " << put_time(localtime(&future_time), "%H:%M") << endl;

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

    int choice;
    bool continueAsking = true;
    string writtenConfirmation = "Writing ";
    string imageName = "";

    while (continueAsking) {
        cout << "Please select your choice 1 to 4: ";
        cin >> choice;
        if(cin.fail()) {
            cin.clear(); // clear the fail state
            cin.ignore(numeric_limits<streamsize>::max(), '\n'); // discard invalid input
            cout << "Invalid input. Please enter a number between 1 and 4." << endl;
            continue;
        }
        switch(choice) {
            case 1: {
                string fileName = input_file.substr(0, input_file.length() - 4) + "_1_SOLUTION_" + to_string(static_cast<int>(ceil(drone1.distance))) + ".txt";
                imageName = input_file.substr(0, input_file.length() - 4) + "_1_SOLUTION_" + to_string(static_cast<int>(ceil(drone1.distance))) + ".png";
                writtenConfirmation = writtenConfirmation + fileName + " to disk" + "\n" + writtenConfirmation + imageName + " to disk"; 
                ofstream outputFile(fileName);

                


                if (!outputFile.is_open()) {
                    cout << "Error opening output file." << endl;
                    return 1;
                }

                for (int i = 0; i < drone1.path.size(); i++) {
                    auto it = find(total_coordinates.begin(), total_coordinates.end(), drone1.path[i]);
                    if (it != total_coordinates.end()) {
                        int index = distance(total_coordinates.begin(), it);
                        outputFile << index + 1 << " ";
                    }
                }
                outputFile.close();


                
                /* PYTHON VISUALIZATION */
                string fileNameCoords = "coords.txt";
                ofstream outputCoords(fileNameCoords);

                outputCoords << drone1.centroid.x << " " << drone1.centroid.y << endl;

                for (int i = 0; i < drone1.path.size(); i++) {
                    outputCoords << drone1.path[i].x << " " << drone1.path[i].y << endl;
                }

                outputCoords << drone1.centroid.x << " " << drone1.centroid.y << endl;

                outputCoords << "-1" << endl;

                outputCoords.close();

                continueAsking = false;

                

                break;
            }
            case 2: {
                // first drone file
                string fileName = input_file.substr(0, input_file.length() - 4) + "_1_SOLUTION_" + to_string(static_cast<int>(ceil(drone2[0].distance))) + ".txt";
                imageName = input_file.substr(0, input_file.length() - 4) + "_1_SOLUTION_" + to_string(static_cast<int>(ceil(drone2[0].distance))) + ".png";
                writtenConfirmation = writtenConfirmation + fileName + " to disk" + "\n" + writtenConfirmation + imageName + " to disk"; 
                ofstream outputFile(fileName);


                if (!outputFile.is_open()) {
                    cout << "Error opening output file." << endl;
                    return 1;
                }

                for (int i = 0; i < drone2[0].path.size(); i++) {
                    auto it = find(total_coordinates.begin(), total_coordinates.end(), drone2[0].path[i]);
                    if (it != total_coordinates.end()) {
                        int index = distance(total_coordinates.begin(), it);
                        outputFile << index + 1 << " ";
                    }
                }
                outputFile.close();

                // second drone file
                fileName = input_file.substr(0, input_file.length() - 4) + "_2_SOLUTION_" + to_string(static_cast<int>(ceil(drone2[1].distance))) + ".txt";
                writtenConfirmation = writtenConfirmation + fileName + " to disk";
                ofstream outputFile2(fileName);


                if (!outputFile2.is_open()) {
                    cout << "Error opening output file." << endl;
                    return 1;
                }

                for (int i = 0; i < drone2[1].path.size(); i++) {
                    auto it = find(total_coordinates.begin(), total_coordinates.end(), drone2[1].path[i]);
                    if (it != total_coordinates.end()) {
                        int index = distance(total_coordinates.begin(), it);
                        outputFile2 << index + 1 << " ";
                    }
                }
                outputFile2.close();



                /* PYTHON VISUALIZATION */
                string fileNameCoords = "coords.txt";
                ofstream outputCoords(fileNameCoords);

                for (int j = 0; j < drone2.size(); j++) {
                    outputCoords << drone2[j].centroid.x << " " << drone2[j].centroid.y << endl;

                    for (int i = 0; i < drone2[j].path.size(); i++) {
                        outputCoords << drone2[j].path[i].x << " " << drone2[j].path[i].y << endl;
                    }

                    outputCoords << drone2[j].centroid.x << " " << drone2[j].centroid.y << endl;

                    outputCoords << "-1" << endl;
                    
                }

                

                outputCoords.close();


                
                continueAsking = false;
                break;
            }
            case 3: {
                // first drone file
                string fileName = input_file.substr(0, input_file.length() - 4) + "_1_SOLUTION_" + to_string(static_cast<int>(ceil(drone3[0].distance))) + ".txt";
                imageName = input_file.substr(0, input_file.length() - 4) + "_1_SOLUTION_" + to_string(static_cast<int>(ceil(drone3[0].distance))) + ".png";
                writtenConfirmation = writtenConfirmation + fileName + " to disk" + "\n" + writtenConfirmation + imageName + " to disk"; 
                ofstream outputFile(fileName);


                if (!outputFile.is_open()) {
                    cout << "Error opening output file." << endl;
                    return 1;
                }

                for (int i = 0; i < drone3[0].path.size(); i++) {
                    auto it = find(total_coordinates.begin(), total_coordinates.end(), drone3[0].path[i]);
                    if (it != total_coordinates.end()) {
                        int index = distance(total_coordinates.begin(), it);
                        outputFile << index + 1 << " ";
                    }
                }
                outputFile.close();

                // second drone file
                fileName = input_file.substr(0, input_file.length() - 4) + "_2_SOLUTION_" + to_string(static_cast<int>(ceil(drone3[1].distance))) + ".txt";
                writtenConfirmation = writtenConfirmation + fileName + ", ";
                ofstream outputFile2(fileName);


                if (!outputFile2.is_open()) {
                    cout << "Error opening output file." << endl;
                    return 1;
                }

                for (int i = 0; i < drone3[1].path.size(); i++) {
                    auto it = find(total_coordinates.begin(), total_coordinates.end(), drone3[1].path[i]);
                    if (it != total_coordinates.end()) {
                        int index = distance(total_coordinates.begin(), it);
                        outputFile2 << index + 1 << " ";
                    }
                }
                outputFile2.close();

                // third drone file
                fileName = input_file.substr(0, input_file.length() - 4) + "_3_SOLUTION_" + to_string(static_cast<int>(ceil(drone3[2].distance))) + ".txt";
                writtenConfirmation = writtenConfirmation + fileName + " to disk";
                ofstream outputFile3(fileName);


                if (!outputFile3.is_open()) {
                    cout << "Error opening output file." << endl;
                    return 1;
                }

                for (int i = 0; i < drone3[2].path.size(); i++) {
                    auto it = find(total_coordinates.begin(), total_coordinates.end(), drone3[2].path[i]);
                    if (it != total_coordinates.end()) {
                        int index = distance(total_coordinates.begin(), it);
                        outputFile3 << index + 1 << " ";
                    }
                }
                outputFile3.close();






                /* PYTHON VISUALIZATION */
                string fileNameCoords = "coords.txt";
                ofstream outputCoords(fileNameCoords);

                for (int j = 0; j < drone3.size(); j++) {
                    outputCoords << drone3[j].centroid.x << " " << drone3[j].centroid.y << endl;

                    for (int i = 0; i < drone3[j].path.size(); i++) {
                        outputCoords << drone3[j].path[i].x << " " << drone3[j].path[i].y << endl;
                    }

                    outputCoords << drone3[j].centroid.x << " " << drone3[j].centroid.y << endl;

                    outputCoords << "-1" << endl;
                    
                }

                

                outputCoords.close();



                continueAsking = false;
                break;
            }
            case 4: {
                // first drone file
                string fileName = input_file.substr(0, input_file.length() - 4) + "_1_SOLUTION_" + to_string(static_cast<int>(ceil(drone4[0].distance))) + ".txt";
                imageName = input_file.substr(0, input_file.length() - 4) + "_1_SOLUTION_" + to_string(static_cast<int>(ceil(drone4[0].distance))) + ".png";
                writtenConfirmation = writtenConfirmation + fileName + " to disk" + "\n" + writtenConfirmation + imageName + " to disk"; 
                ofstream outputFile(fileName);


                if (!outputFile.is_open()) {
                    cout << "Error opening output file." << endl;
                    return 1;
                }

                for (int i = 0; i < drone4[0].path.size(); i++) {
                    auto it = find(total_coordinates.begin(), total_coordinates.end(), drone4[0].path[i]);
                    if (it != total_coordinates.end()) {
                        int index = distance(total_coordinates.begin(), it);
                        outputFile << index + 1 << " ";
                    }
                }
                outputFile.close();

                // second drone file
                fileName = input_file.substr(0, input_file.length() - 4) + "_2_SOLUTION_" + to_string(static_cast<int>(ceil(drone4[1].distance))) + ".txt";
                writtenConfirmation = writtenConfirmation + fileName + ", ";
                ofstream outputFile2(fileName);


                if (!outputFile2.is_open()) {
                    cout << "Error opening output file." << endl;
                    return 1;
                }

                for (int i = 0; i < drone4[1].path.size(); i++) {
                    auto it = find(total_coordinates.begin(), total_coordinates.end(), drone4[1].path[i]);
                    if (it != total_coordinates.end()) {
                        int index = distance(total_coordinates.begin(), it);
                        outputFile2 << index + 1 << " ";
                    }
                }
                outputFile2.close();

                // third drone file
                fileName = input_file.substr(0, input_file.length() - 4) + "_3_SOLUTION_" + to_string(static_cast<int>(ceil(drone4[2].distance))) + ".txt";
                writtenConfirmation = writtenConfirmation + fileName + ", ";
                ofstream outputFile3(fileName);


                if (!outputFile3.is_open()) {
                    cout << "Error opening output file." << endl;
                    return 1;
                }

                for (int i = 0; i < drone4[2].path.size(); i++) {
                    auto it = find(total_coordinates.begin(), total_coordinates.end(), drone4[2].path[i]);
                    if (it != total_coordinates.end()) {
                        int index = distance(total_coordinates.begin(), it);
                        outputFile3 << index + 1 << " ";
                    }
                }
                outputFile3.close();

                // fourth drone file
                fileName = input_file.substr(0, input_file.length() - 4) + "_4_SOLUTION_" + to_string(static_cast<int>(ceil(drone4[3].distance))) + ".txt";
                writtenConfirmation = writtenConfirmation + fileName + " to disk";
                ofstream outputFile4(fileName);


                if (!outputFile4.is_open()) {
                    cout << "Error opening output file." << endl;
                    return 1;
                }

                for (int i = 0; i < drone4[3].path.size(); i++) {
                    auto it = find(total_coordinates.begin(), total_coordinates.end(), drone4[3].path[i]);
                    if (it != total_coordinates.end()) {
                        int index = distance(total_coordinates.begin(), it);
                        outputFile4 << index + 1 << " ";
                    }
                }
                outputFile4.close();


                /* PYTHON VISUALIZATION */
                string fileNameCoords = "coords.txt";
                ofstream outputCoords(fileNameCoords);

                for (int j = 0; j < drone4.size(); j++) {
                    outputCoords << drone4[j].centroid.x << " " << drone4[j].centroid.y << endl;

                    for (int i = 0; i < drone4[j].path.size(); i++) {
                        outputCoords << drone4[j].path[i].x << " " << drone4[j].path[i].y << endl;
                    }

                    outputCoords << drone4[j].centroid.x << " " << drone4[j].centroid.y << endl;

                    outputCoords << "-1" << endl;
                    
                }

                

                outputCoords.close();




                continueAsking = false;
                break;
            }
            default:
                cout << "Invalid choice. Please try again." << endl;
                continue;
        }
    }

    cout << writtenConfirmation << endl;


    
    // cout << "Image of route written to disk as " << imageFileName << endl;

    // // command to run our python code
    // string command = "python3 path_image.py";

    // int result = system(command.c_str());
    // python visualization
    string command = "python3 path_image.py " + imageName;

    int result = system(command.c_str());


    return 0;


}