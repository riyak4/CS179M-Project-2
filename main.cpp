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

    //variable storing the current position of the drone (inital position)
    //moved current position to Problem.h cause i highkey don't think it needs to be here
    //Coordinate curr_position = {0,0};

    //variable storing the final path distance 
    //float for easier calculation, output round up (ceiling) int for output files !!
    float final_distance = numeric_limits<float>::max();

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

    thread tracker_thread([&]{
        string dummy;
        getline(cin, dummy);
        //tells the main loop to stop
        stop_loop.store(true);     
    });
    //using count for the time being of testing the loop

    //in the while loop, is where the search function will be called and output final distances
    int count = 0;
    int printInitialStatement = 0;

    while (!stop_loop.load()) {
        //creating a path to start with 
        current_path_coordinates = hc.restartPath(total_coordinates);

        // printing out initial statement only once
        if (printInitialStatement == 0) {
            cout << "There are " << current_path_coordinates.size() << " nodes, computing route..\n";
            cout << "     Shortest Route Discovered So Far " << endl;

            printInitialStatement = 1;

        }

        
        // testing path's total distance and trying to swap to find better path
        hc.makingPath(current_path_coordinates, final_path_coordinates, stop_loop);
        // if (newDist < final_distance) {
        //     final_distance = newDist;
        //     cout << "Found new distance: " << final_distance << endl;
        // }
        this_thread::sleep_for(chrono::milliseconds(500));
    }


    if (tracker_thread.joinable()) tracker_thread.join();

   
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