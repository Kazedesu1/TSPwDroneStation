#pragma once
#include <vector>
#include <string>
#include <iostream>
#include "Instances.h"
using namespace std;

class Solution {
public:
    struct DroneRoute {
        int stationId;               // station xuất phát
        int droneId;                 // ID drone trong station
        vector<int> customers;       // danh sách khách hàng drone phục vụ
        double completionTime;       // thời gian drone hoàn thành route
    };

    struct TruckRoute {
        int truckId;
        vector<int> route = { 0,0 };           // các node truck đi qua
        double completionTime;       // thời gian truck hoàn thành
    };

    vector<TruckRoute> trucks;       // tất cả các truck routes
    vector<DroneRoute> drones;       // tất cả các drone routes
    double objective;                // makespan 

    Solution();

    void display() const;      // in lời giải
};
