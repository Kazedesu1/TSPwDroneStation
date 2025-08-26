#include "solution.h"
#include <algorithm>
#include <iostream>

Solution::Solution() {
}

void Solution::display() const {
    std::cout << "===== Solution =====" << std::endl;
    for (const auto& t : trucks) {
        std::cout << "Truck " << t.truckId << ": ";
        for (int v : t.route) std::cout << v << " ";
        std::cout << " | Completion = " << t.completionTime << std::endl;
    }

    for (const auto& d : drones) {
        std::cout << "Station " << d.stationId
            << ", Drone " << d.droneId << ": ";
        for (int v : d.customers) std::cout << v << " ";
        std::cout << " | Completion = " << d.completionTime << std::endl;
    }

    std::cout << "Objective (makespan) = " << objective << std::endl;
}
