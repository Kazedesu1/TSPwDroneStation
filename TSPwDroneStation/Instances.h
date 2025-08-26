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
        double maxFlight;                // E/2
		vector<int> reachableCustomers;  // customers trong t?m bay
		vector<double> flightTime;       // th?i gian bay ??n t?ng customer
    };

    vector<vector<double>> tau;     // distance matrix (truck)
    vector<vector<double>> nodes;   // coordinates: index -> (x,y)
    vector<int> truckonly;          
    vector<int> C;                  
    vector<Stations> stationList;   // all stations
	int numtrucks;                  
    int activeStations;             
    int UAVs;                       
    int n;                          

    double E;                       // max endurance
    double alpha;                   // drone speed 

    bool loadFromFile(const std::string& filename = "");
    void processStations();        
    void displayData();
};

// helper function
double eucliddistance(const vector<vector<double>>& mat, int node1, int node2);
