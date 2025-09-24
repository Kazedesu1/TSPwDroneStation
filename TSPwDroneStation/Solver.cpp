#include "Solver.h"
#include "Param.h"
#include <iostream>
#include <algorithm>
#include <limits>
#include <cstdlib>
#include <unordered_map>
#include <ctime>
#include <unordered_set>
using namespace std;

Solver::Solver(const INSTANCE& inst) : instance(inst) {}

void Solver::solve() {
    Solution sol(instance);
    sol = greedyInsertion(sol);
    sol.objective = sol.calculateMakespan();
	firstObjective = sol.objective;

    Solution bestSol = sol;

    int maxIter = 100;   
    int noImprove = 0;   
    int maxNoImprove = 20;

    for (int iter = 0; iter < maxIter && noImprove < maxNoImprove; ++iter) {
        double r1 = Param::real_random_generator(0, 1);
		Solution destroyed(instance);
        destroyed = RandomRemoveStation(sol);
        if( r1 < 0.5)
             destroyed = WorstRemoval(destroyed);
        else
        {
             destroyed = RandomRemoval(destroyed);
        }
        
        Solution repaired = greedyInsertion(destroyed);
        repaired.objective = repaired.calculateMakespan();

        if (repaired.objective < sol.objective) {
            sol = repaired;
            if (sol.objective < bestSol.objective) {
                bestSol = sol;
                noImprove = 0;  
            }
        }
        else {
            noImprove++;
        }

        std::cout << "Iter " << iter
            << " Obj=" << sol.objective
            << " Best=" << bestSol.objective
            << std::endl;
    }

    std::cout << "Final best solution: " << std::endl;
    bestSol.display();
	bestSol.feasiblecheck();
	bestObjective = bestSol.objective;
}


Solution Solver::greedyInsertion(Solution solution) {
    Solution sol(instance);
    sol = solution;
    int num_trucks = instance.num_trucks;
    int UAVs = instance.UAVs;

    auto distanceToTruckRoute = [&](int node) {
        double best = 1e9;
        for (int i = 0; i < num_trucks; i++) {
            for (int tNode : sol.trucks[i].route) {
                best = std::min(best, instance.tau[node][tNode]);
            }
        }
        return best;
        };

    vector<pair<int, int>> stationScore; // (score, stationIndex)

    for (int s : sol.remaining_stations) {
        double dStationToTruck = distanceToTruckRoute(s);
        int score = 0;
        for (int c : instance.station_list[s - instance.C.size() - 1].reachable_customers) {
            double dCustomerToTruck = distanceToTruckRoute(c);
            if (dStationToTruck < dCustomerToTruck) {
                score++;
            }
        }
        stationScore.push_back({ score, s });
    }

    // sort stations theo score giảm dần
    std::sort(stationScore.begin(), stationScore.end(),
        [](auto& a, auto& b) {
            return a.first > b.first;
        });

    // chọn các station có score > 0
    vector<int> active_stations;
    for (auto& p : stationScore) {
        if (p.first > 0) active_stations.push_back(p.second);
    }
    if (active_stations.size() > instance.active_stations) {
        active_stations.resize(instance.active_stations);
    }

    // --- Hàng đợi khách hàng + station còn lại ---
    vector<int> customerQueue = sol.remaining_customers;
    for (int s : active_stations) {
        customerQueue.push_back(s);
    }

    // Shuffle queue
    Param::shuffle(customerQueue);

    // --- Tập khách hàng có thể đi drone ---
    vector<int> droneElit;
    for (int s : active_stations) {
        for (int c : instance.station_list[s - instance.C.size() - 1].reachable_customers) {
            droneElit.push_back(c);
        }
    }

    // --- Bắt đầu greedy insertion ---
    for (int customer : customerQueue) {
        double bestcost = numeric_limits<double>::max();
        int bestVehi = -1, bestPos = -1;
        int totruck = -1;

        // Truck insertion
        for (int t = 0; t < num_trucks; ++t) {
            for (size_t pos = 1; pos < sol.trucks[t].route.size(); ++pos) {
                int prevNode = sol.trucks[t].route[pos - 1];
                int nextNode = (pos < sol.trucks[t].route.size()) ? sol.trucks[t].route[pos] : 0;

                double trucktime = sol.trucks[t].completion_time
                    + instance.tau[prevNode][customer]
                    + instance.tau[customer][nextNode]
                    - instance.tau[prevNode][nextNode];

                if (trucktime < bestcost) {
                    bestcost = trucktime;
                    bestVehi = t;
                    bestPos = pos;
                    totruck = 1;
                }
            }
        }

        // Drone insertion (nếu customer nằm trong droneElit)
        if (std::find(droneElit.begin(), droneElit.end(), customer) != droneElit.end()) {
            for (int d = 0; d < sol.drones.size(); d++) {
                int station_id = sol.drones[d].station_id;
                auto& reachable = instance.station_list[station_id - instance.C.size() - 1].reachable_customers;
                if (std::find(reachable.begin(), reachable.end(), customer) != reachable.end()) {
                    double flight_time = sol.drones[d].completion_time
                        + instance.tau[station_id][customer] * 2 / instance.alpha;
                    if (flight_time < bestcost) {
                        bestcost = flight_time;
                        bestVehi = d;
                        totruck = 0;
                    }
                }
            }
        }

        // --- Thực hiện insertion ---
        if (totruck == 1) {
            sol.trucks[bestVehi].route.insert(sol.trucks[bestVehi].route.begin() + bestPos, customer);
            sol.trucks[bestVehi].completion_time = bestcost;
            sol.vehicle_of_customer[customer] = bestVehi;

            // Nếu là station mới
            if (isStation(customer)) {
                double truckArriveStation = 0.0;
                for (int i = 1; i <= bestPos; ++i) {
                    int u = sol.trucks[bestVehi].route[i - 1];
                    int v = sol.trucks[bestVehi].route[i];
                    truckArriveStation += instance.tau[u][v];
                }
                sol.station_time[customer] = truckArriveStation;

                for (int d = 0; d < UAVs; ++d) {
                    Solution::DroneRoute dr;
                    dr.station_id = customer;
                    dr.drone_id = d;
                    dr.completion_time = sol.station_time[customer];
                    sol.drones.push_back(dr);
                }
                sol.activated_stations.push_back(customer);

                for (int c : instance.station_list[customer - instance.C.size() - 1].reachable_customers) {
                    droneElit.push_back(c);
                }
                // Xóa station khỏi remaining_stations
                sol.remaining_stations.erase(
                    std::remove(sol.remaining_stations.begin(), sol.remaining_stations.end(), customer),
                    sol.remaining_stations.end()
                );
            }

            // Cập nhật thời gian cho station sau
            int u = sol.trucks[bestVehi].route[bestPos - 1];
            int v = sol.trucks[bestVehi].route[bestPos + 1];
            double delta = instance.tau[u][customer] + instance.tau[customer][v] - instance.tau[u][v];
            for (int i = bestPos + 1; i < (int)sol.trucks[bestVehi].route.size(); ++i) {
                int node = sol.trucks[bestVehi].route[i];
                if (isStation(node)) {
                    sol.station_time[node] += delta;
                    for (auto& dr : sol.drones) {
                        if (dr.station_id == node) {
                            dr.completion_time += delta;
                        }
                    }
                }
            }
        }
        else if (totruck == 0) {
            sol.drones[bestVehi].customers.push_back(customer);
            int station_id = sol.drones[bestVehi].station_id;
            sol.drones[bestVehi].completion_time += instance.tau[station_id][customer] * 2 / instance.alpha;
        }

        // Xóa customer khỏi remaining_customers
        sol.remaining_customers.erase(
            std::remove(sol.remaining_customers.begin(), sol.remaining_customers.end(), customer),
            sol.remaining_customers.end()
        );
    }

    removeUnusedStations(sol);
    sol.objective = sol.calculateMakespan();
    return sol;
}


// Ki?m tra node có ph?i là station không
bool Solver::isStation(int node) const {
    for (const auto& station : instance.station_list) {
        if (station.id == node) return true;
    }
    return false;
}

Solution Solver::removeStation(Solution &sol, int stationNode) {
    Solution newSol = sol;
    // Tìm và đưa lại các customer trong drone routes về remaining_customers
    for (auto it = newSol.drones.begin(); it != newSol.drones.end();) {
        if (it->station_id == stationNode) {
            for (int c : it->customers) {
                newSol.remaining_customers.push_back(c);
            }
            it = newSol.drones.erase(it); // xóa drone route
        }
        else {
            ++it;
        }
    }
    for (auto& truck : newSol.trucks) {
        auto it = std::find(truck.route.begin(), truck.route.end(), stationNode);
        if (it != truck.route.end()) {
            int pos = std::distance(truck.route.begin(), it);
            int prevNode = (pos > 0) ? truck.route[pos - 1] : 0;
            int nextNode = (pos < truck.route.size() - 1) ? truck.route[pos + 1] : 0;
            truck.completion_time -= instance.tau[prevNode][stationNode] + instance.tau[stationNode][nextNode] - instance.tau[prevNode][nextNode];
            truck.route.erase(it);
        }
	}
    newSol.activated_stations.erase(
        std::remove(newSol.activated_stations.begin(), newSol.activated_stations.end(), stationNode),
        newSol.activated_stations.end()
	);
	newSol.remaining_stations.push_back(stationNode);
    newSol.objective = newSol.calculateMakespan();
    return newSol;
}

void Solver::removeUnusedStations(Solution& sol) {
    Solution newSol = sol;

    unordered_set<int> usedStations;
    for (const auto& dr : newSol.drones) {
        if (!dr.customers.empty()) {
            usedStations.insert(dr.station_id);
        }
    }

    for (int stationNode : newSol.activated_stations) {
        if (usedStations.find(stationNode) == usedStations.end()) {
			newSol = removeStation(newSol, stationNode);
        }
    }
	sol = newSol;
}

Solution Solver::RandomRemoval(Solution& sol) {
    Solution newSol = sol;

    // 1. Gom tất cả khách hàng trong mọi truck routes (không tính station và depot)
    vector<pair<int, int>> all_customers; // (customer, truck_id)
    for (int t = 0; t < (int)newSol.trucks.size(); ++t) {
        for (int node : newSol.trucks[t].route) {
            if (!isStation(node) && node != 0) {
                all_customers.push_back({ node, t });
            }
        }
    }

    // Shuffle danh sách tất cả khách hàng
    std::shuffle(all_customers.begin(), all_customers.end(), Param::mt);

    // Số lượng cần xóa = 50% tổng khách hàng
    size_t num_to_remove = round(all_customers.size() * Param::real_random_generator(0.3,0.6));

    // Xóa lần lượt
    for (size_t k = 0; k < num_to_remove; ++k) {
        int customer = all_customers[k].first;
        int truck_id = all_customers[k].second;
        removeCustomerFromTruck(newSol, truck_id, customer);
    }


    newSol.objective = newSol.calculateMakespan();
    return newSol;
}

Solution Solver::RandomRemoveStation(Solution& sol) {
    Solution newSol = sol;
    // 2. Xóa ngẫu nhiên 1 nửa drone station bằng removeStation
    size_t num_station_to_remove = newSol.activated_stations.size() / 2;
    vector<int> stations_to_remove = newSol.activated_stations;
    std::shuffle(stations_to_remove.begin(), stations_to_remove.end(), Param::mt);
    stations_to_remove.resize(num_station_to_remove);

    for (int station : stations_to_remove) {
        newSol = removeStation(newSol, station);
    }

    return newSol;
}

Solution Solver::WorstRemoval(Solution& sol) {
	Solution newSol = sol;
    vector<int> truck_customer;
    for (auto& truck : newSol.trucks) {
        for (int node : truck.route) {
            if (!isStation(node) && node != 0) {
                truck_customer.push_back(node);
            }
        }
    }
	vector<int> customers_list = truck_customer;
    double delta = Param::real_random_generator(0.3, 0.6);
    while (newSol.remaining_customers.size() <= round(truck_customer.size() * delta))
    {
        double worst_cost = 0;
        int worst_customer = -1;
        int worst_truck = -1;
        for (int c : customers_list) {
            for (int t = 0; t < (int)newSol.trucks.size(); ++t) {
                auto& truck = newSol.trucks[t];
                auto it = std::find(truck.route.begin(), truck.route.end(), c);
                if (it != truck.route.end()) {
                    int pos = std::distance(truck.route.begin(), it);
                    int prevNode = truck.route[pos - 1];
                    int nextNode = truck.route[pos + 1];
                    double cost = instance.tau[prevNode][c] + instance.tau[c][nextNode] - instance.tau[prevNode][nextNode];
                    if (cost > worst_cost) {
                        worst_cost = cost;
                        worst_truck = t;
                        worst_customer = c;
                    }
                }
            }
        }
        if (worst_customer != -1) {
            removeCustomerFromTruck(newSol, worst_truck, worst_customer);
            customers_list.erase(std::remove(customers_list.begin(), customers_list.end(), worst_customer), customers_list.end());
        }
    }

	return newSol;
}


void Solver::removeCustomerFromTruck(Solution& sol, int truckId, int customer) {
    auto& truck = sol.trucks[truckId];
    auto it = std::find(truck.route.begin(), truck.route.end(), customer);
    if (it == truck.route.end()) return;

    int pos = std::distance(truck.route.begin(), it);
    int prevNode = truck.route[pos - 1];
    int nextNode = truck.route[pos + 1];

    double delta = instance.tau[prevNode][customer] + instance.tau[customer][nextNode] - instance.tau[prevNode][nextNode];

    truck.route.erase(it);

    truck.completion_time -= delta;

    for (int i = pos; i < (int)truck.route.size(); ++i) {
        int node = truck.route[i];
        if (isStation(node)) {
            sol.station_time[node] -= delta;
            for (auto& dr : sol.drones) {
                if (dr.station_id == node) {
                    dr.completion_time -= delta;
                }
            }
        }
    }
	sol.remaining_customers.push_back(customer);
}
