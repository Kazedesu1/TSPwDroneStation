#include "Solution.h"
#include <algorithm>
#include <iostream>
#include <iomanip>
#include "Param.h"
#include <unordered_map>

Solution::Solution(const INSTANCE& inst) : instance(inst) {
    vehicle_of_customer.resize(instance.n);
    station_time.resize(instance.n);
    wait_time.resize(instance.n);
    trucks.resize(instance.num_trucks);
	station_truck.resize(instance.n, -1); // -1 nghĩa là chưa có truck phục vụ
    for (int t = 0; t < instance.num_trucks; ++t) {
        trucks[t].truck_id = t;
        trucks[t].completion_time = 0.0;
    }

    // Khởi tạo remaining customers & stations
    remaining_customers = inst.C;
    for (const auto& st : inst.station_list) {
        remaining_stations.push_back(st.id);
    }

    drones.resize(instance.station_list.size());
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
	wait_time(other.wait_time),
    objective(other.objective),
	station_truck(other.station_truck)
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
		wait_time = other.wait_time;
        objective = other.objective;
		station_truck = other.station_truck;
    }
    return *this;
}


void Solution::display() const {
    std::cout << "===== Solution =====" << std::endl;
    std::cout << "seed: " << Param::seed << std::endl;
    const int width = 50;

    for (const auto& t : trucks) {
        std::ostringstream oss;
        oss << "Truck " << t.truck_id << ": ";
        for (int v : t.route) oss << v << " ";

        std::cout << std::left << std::setw(width) << oss.str()
            << "| Completion = " << t.completion_time << std::endl;
    }

    for (const auto& stationDrones : drones) {
        for (const auto& d : stationDrones) {
            std::ostringstream oss;
            oss << "Station " << d.station_id
                << ", Drone " << d.drone_id << ": ";
            for (int v : d.customers) oss << v << " ";

            std::cout << std::left << std::setw(width) << oss.str()
                << "| Completion = " << d.completion_time << " | Arrival Time =" << station_time[d.station_id] << " | Wait Time =" << wait_time[d.station_id] << std::endl;
        }
    }

    std::cout << "Objective (makespan) = " << objective << std::endl;
}

double Solution::calculateMakespan() const {
    double makespan = 0.0;
    for (const auto& t : trucks) {
        makespan = std::max(makespan, t.completion_time);
    }
    for (const auto& stationDrones : drones) {
        for (const auto& d : stationDrones) {
            makespan = std::max(makespan, d.completion_time);
        }
    }
    return makespan;
}

void Solution::feasiblecheck() const {
    //Kiểm tra tất cả khách hàng đã được phục vụ bằng route của trucks và drones và lặp lại 2 lần không
    // Kiểm tra khách hàng được phục vụ đúng một lần
    std::unordered_map<int, int> servedCount;
    for (const auto& t : trucks) {
        for (int node : t.route) {
            if (node != 0) {
                servedCount[node]++;
            }
        }
    }
    for (const auto& stationDrones : drones) {
        for (const auto& d : stationDrones) {
            for (int c : d.customers) {
                servedCount[c]++;
            }
        }
    }

    for (int c : instance.C) {
        if (servedCount[c] == 0) {
            std::cout << "Customer " << c << " is not served!" << std::endl;
        }
        else if (servedCount[c] > 1) {
            std::cout << "Customer " << c << " is served multiple times ("
                << servedCount[c] << " times)!" << std::endl;
        }
    }

    if (servedCount.size() > instance.C.size()) {
        bool ok = true;
        for (auto& kv : servedCount) {
            if (kv.second != 1) { ok = false; break; }
        }
        if (ok) {
            std::cout << "All customers are served exactly once." << std::endl;
        }
    }

    // Kiểm tra thời gian hoàn thành của truck
    for (const auto& t : trucks) {
        double time = 0.0;
        for (size_t i = 1; i < t.route.size(); ++i) {
            int u = t.route[i - 1];
            int v = t.route[i];
            time += instance.tau[u][v];
        }
        if (std::abs(time - t.completion_time) > 1e-6) {
            std::cout << "Truck " << t.truck_id << " completion time mismatch!" << std::endl;
        }
        else
            std::cout << "Truck " << t.truck_id << " completion time verified." << std::endl;
    }
    // Kiểm tra thời gian hoàn thành của drone
    for (const auto& stationDrones : drones) {
        for (const auto& d : stationDrones) {
            int station_id = d.station_id;
            double time = 0.0;
            if (station_id >= 0 && station_id < (int)station_time.size())
                time = station_time[station_id];
            for (int c : d.customers) {
                time += instance.tau[station_id][c] * 2 / instance.alpha;
            }
            if (std::abs(time - d.completion_time) > 1e-6) {
                std::cout << "Drone at Station " << d.station_id << " completion time mismatch!" << std::endl;
            }
            else
                std::cout << "Drone at Station " << d.station_id << " completion time verified." << std::endl;
        }
    }
    // Kiểm tra range của drone 
    bool exceeds = false;
    for (const auto& stationDrones : drones) {
        for (const auto& d : stationDrones) {
            int station_id = d.station_id;
            double max_flight = instance.E;
            for (int c : d.customers) {
                double flight_dist = instance.tau[station_id][c] * 2;
                if (flight_dist > max_flight + 1e-6) {
                    std::cout << "Drone at Station " << d.station_id << " exceeds range to Customer " << c << "!" << std::endl;
                    exceeds = true;
                }
            }
        }
    }
    if (!exceeds)
        std::cout << "All drones are within their range." << std::endl;
    std::cout << "Feasibility check completed." << std::endl;
}
