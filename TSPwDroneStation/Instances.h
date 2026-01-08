#pragma once
#include <stdio.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
using namespace std;

class INSTANCE
{
public:
    struct Stations {
        int id;
        double x;
        double y;
        double max_flight;                // E/2
		vector<int> reachable_customers;  // customers trong t?m bay
        vector<int> drone_only_nodes;
		vector<double> flight_time;       // th?i gian bay ??n t?ng customer
		int num_drones = 0;               // số drone hiện có tại station
    };

    vector<vector<double>> tau;     // distance matrix (truck)
	vector<vector<double>> tauprime; // distance matrix (drone)
    vector<vector<double>> nodes;   // coordinates: index -> (x,y)
    vector<int> truck_only;          
	vector<int> drone_only;
    vector<int> C;                  
    vector<Stations> station_list;   // all stations
	int num_trucks;                  
    int active_stations;             
    int UAVs;                       
    int n;                          
    double E;                       // max endurance
    double alpha;                   // drone speed 

    bool loadFromFile(const std::string& filename = "");
    bool loadFromFile2(const std::string& filename = "");
    bool loadFromFile3(const std::string& filename = "");
    bool loadFromFile4(const std::string& filename = "");


    void processStations();        
    void displayData();
};

// helper function
double eucliddistance(const vector<vector<double>>& mat, int node1, int node2);
double manhatandistance(const vector<vector<double>>& mat, int node1, int node2);
