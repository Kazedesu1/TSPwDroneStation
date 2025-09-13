#include "solution.h"
#include <algorithm>
#include <iostream>
#include <iomanip>
#include "Param.h"

Solution::Solution(const INSTANCE& inst) : instance(inst) {
    vehicle_of_customer.resize(instance.n);
    station_time.resize(instance.n);
    trucks.resize(instance.num_trucks);

    for (int t = 0; t < instance.num_trucks; ++t) {
        trucks[t].truck_id = t;
        trucks[t].completion_time = 0.0;
    }

    // Khởi tạo remaining customers & stations
    remaining_customers = inst.C;
    for (const auto& st : inst.station_list) {
        remaining_stations.push_back(st.id);
    }
}

Solution::Solution(const Solution& other)
    : instance(other.instance),
    remaining_customers(other.remaining_customers),
    remaining_stations(other.remaining_stations),
    station_time(other.station_time),
    vehicle_of_customer(other.vehicle_of_customer),
    trucks(other.trucks),
    drones(other.drones),
    activated_stations(other.activated_stations),
    objective(other.objective)
{
}

Solution& Solution::operator=(const Solution& other) {
    if (this != &other) {
        remaining_customers = other.remaining_customers;
        remaining_stations = other.remaining_stations;
        station_time = other.station_time;
        vehicle_of_customer = other.vehicle_of_customer;
        trucks = other.trucks;
        drones = other.drones;
        activated_stations = other.activated_stations;
        objective = other.objective;
    }
    return *this;
}


void Solution::display() const {
    std::cout << "===== Solution =====" << std::endl;
	std::cout << "seed: " << Param::seed << std::endl;
    // xác định độ rộng in (tùy chỉnh theo độ dài lớn nhất bạn có)
    const int width = 50;

    for (const auto& t : trucks) {
        std::ostringstream oss;
        oss << "Truck " << t.truck_id << ": ";
        for (int v : t.route) oss << v << " ";

        std::cout << std::left << std::setw(width) << oss.str()
            << "| Completion = " << t.completion_time << std::endl;
    }

    for (const auto& d : drones) {
        std::ostringstream oss;
        oss << "Station " << d.station_id
            << ", Drone " << d.drone_id << ": ";
        for (int v : d.customers) oss << v << " ";

        std::cout << std::left << std::setw(width) << oss.str()
            << "| Completion = " << d.completion_time << std::endl;
    }

    std::cout << "Objective (makespan) = " << objective << std::endl;
}

double Solution::calculateMakespan() const {
    double makespan = 0.0;
    for (const auto& t : trucks) {
        makespan = std::max(makespan, t.completion_time);
    }
    for (const auto& d : drones) {
        makespan = std::max(makespan, d.completion_time);
    }
    return makespan;
}

