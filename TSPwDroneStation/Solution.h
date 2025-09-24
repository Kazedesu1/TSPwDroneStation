#pragma once
#include <vector>
#include <string>
#include <iostream>
#include "Instances.h"
using namespace std;

class Solution {
public:
    struct DroneRoute {
        int station_id;               // station xuất phát
        int drone_id;                 // ID drone trong station
        vector<int> customers;        // danh sách khách hàng drone phục vụ
        double completion_time;       // thời gian drone hoàn thành route
    };

    struct TruckRoute {
        int truck_id;
        vector<int> route = { 0,0 };  // các node truck đi qua
        double completion_time;        // thời gian truck hoàn thành
    };
	vector<int> remaining_customers; // danh sách khách hàng chưa được phục vụ
	vector<int> remaining_stations; // danh sách station chưa được kích hoạt
    vector<double> station_time;           // thời gian truck đến station
    vector<int> vehicle_of_customer;       // lưu truck_id hoặc drone_id phục vụ customer
    vector<TruckRoute> trucks;             // tất cả các truck routes
    vector<DroneRoute> drones;             // tất cả các drone routes
    vector<int> activated_stations;        // danh sách station được kích hoạt
    double objective;                      // makespan 

    Solution(const INSTANCE& inst);        // constructor nhận INSTANCE
    const INSTANCE& instance;              // instance phải là const reference

    void display() const;                  // in lời giải
    double calculateMakespan() const;      // Hàm tính makespan
    void RemoveFromTruck(int node, int truckid);

    // Add copy constructor and copy assignment operator
    Solution(const Solution& other);
    Solution& operator=(const Solution& other);
    void feasiblecheck() const;
};
