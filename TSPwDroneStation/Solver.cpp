#include "Solver.h"
#include <iostream>
#include <algorithm>
#include <limits>
#include <cstdlib>
#include <unordered_map>
#include <ctime>
using namespace std;

Solver::Solver(const INSTANCE& inst) : instance(inst) {}

void Solver::solve() {
    Solution sol1;
	while (sol1.objective == 0 || sol1.objective > 180)
    {
    sol1 = greedyInsertion();
    sol1.display();
    }
}

Solution Solver::greedyInsertion() {
    Solution sol;
    int n = instance.n;
    int numTrucks = instance.numtrucks;
    int numStations = instance.activeStations;
    int UAVs = instance.UAVs;

    sol.trucks.resize(numTrucks);
    for (int t = 0; t < numTrucks; ++t) {
        sol.trucks[t].truckId = t;
        sol.trucks[t].completionTime = 0.0;
    }

    //  Chọn station được sử dụng 
    auto distanceToTruckRoute = [&](int node) {
        double best = 1e9;
        for (int i = 0; i < instance.numtrucks; i++) {
            for (int tNode : sol.trucks[i].route) {
                best = min(best, instance.tau[node][tNode]);
            }
        }
        return best;
        };

    vector<pair<int, int>> stationScore; // (score, stationIndex)

    for (int s = 0; s < numStations; ++s) {
        int stationNode = instance.stationList[s].id;
        double dStationToTruck = distanceToTruckRoute(stationNode);

        int score = 0;
        for (int c : instance.stationList[s].reachableCustomers) {
            double dCustomerToTruck = distanceToTruckRoute(c);

            if (dStationToTruck < dCustomerToTruck) {
                score++;
            }
        }

        stationScore.push_back({ score, stationNode });
    }

    // sort stations theo thứ tự score giảm dần
    std::sort(stationScore.begin(), stationScore.end(),
        [](auto& a, auto& b) {
            return a.first > b.first;
        });

    // chọn các station có score > 0 và không vượt quá số station được phép active
    vector<int> activeStations;
    for (auto& p : stationScore) {
        if (p.first > 0) activeStations.push_back(p.second);
    }
    if (activeStations.size() > instance.activeStations) {
        activeStations.resize(instance.activeStations);
    }


    //  Cho station vào hàng đợi các customer 
    vector<int> customerQueue = instance.C;
    for (int s : activeStations) {
        customerQueue.push_back(s);
    }

    //  Shuffle queue 
    srand(time(NULL));
    random_shuffle(customerQueue.begin(), customerQueue.end());

    //  Best insertion  
    vector<int> activatedStations; // danh sách station đã active
    vector<int> droneElit;

    for (int customer : customerQueue) {
        double bestcost = numeric_limits<double>::max();
        int bestVehi = -1, bestPos = -1; int totruck = -1;
        for (int t = 0; t < numTrucks; ++t) {
            for (size_t pos = 1; pos < sol.trucks[t].route.size(); ++pos) {
                int prevNode = sol.trucks[t].route[pos - 1];
                int nextNode = (pos < sol.trucks[t].route.size()) ? sol.trucks[t].route[pos] : 0;

                double trucktime = sol.trucks[t].completionTime + instance.tau[prevNode][customer] + instance.tau[customer][nextNode]
                    - instance.tau[prevNode][nextNode];
                if (trucktime <= bestcost) {
                    bestcost = trucktime;
                    bestVehi = t;
                    bestPos = pos;
                    totruck = 1;
                }
            }
        }
        if (std::find(droneElit.begin(), droneElit.end(), customer) != droneElit.end()) {
            for (int d = 0; d < sol.drones.size(); d++) {
                int stationid = sol.drones[d].stationId;
                if (std::find(instance.stationList[stationid - instance.C.size() - 1].reachableCustomers.begin(), instance.stationList[stationid - instance.C.size() - 1].reachableCustomers.end(), customer) != instance.stationList[stationid - instance.C.size() - 1].reachableCustomers.end()) {
                    double flightTime = sol.drones[d].completionTime + instance.tau[stationid][customer]*2 / instance.alpha;
                    if (flightTime <= bestcost) {
                        bestcost = flightTime;
                        bestVehi = d;
                        totruck = 0;
                    }
                }

            }
        }

        if (totruck == 1) {
            sol.trucks[bestVehi].route.insert(sol.trucks[bestVehi].route.begin() + bestPos, customer);
            sol.trucks[bestVehi].completionTime = bestcost;
            if (isStation(customer)) {
                double truckArriveStation = 0.0;
                // Tính thời gian truck đến station (customer ở vị trí bestPos)
                for (int i = 1; i <= bestPos; ++i) {
                    int u = sol.trucks[bestVehi].route[i - 1];
                    int v = sol.trucks[bestVehi].route[i];
                    truckArriveStation += instance.tau[u][v];
                }

                // Tạo route cho mỗi drone xuất phát từ station này
                for (int d = 0; d < UAVs; ++d) {
                    Solution::DroneRoute dr;
                    dr.stationId = customer;
                    dr.droneId = d;
                    dr.completionTime = truckArriveStation; // gán thời điểm station được kích hoạt
                    sol.drones.push_back(dr);
                }
                activatedStations.push_back(customer);

                if (instance.stationList[customer - instance.C.size() - 1].id == customer) {
                    for (int c : instance.stationList[customer - instance.C.size() - 1].reachableCustomers) {
                        droneElit.push_back(c);
                    }
                }
            }
        }
        else {
            sol.drones[bestVehi].customers.push_back(customer);
            int stationid = sol.drones[bestVehi].stationId;
            sol.drones[bestVehi].completionTime += instance.tau[stationid][customer]*2 / instance.alpha;
            droneElit.erase(remove(droneElit.begin(), droneElit.end(), customer), droneElit.end());
        }
    }
    //  Tính makespan 
    double makespan = 0.0;
    for (const auto& t : sol.trucks) {
        makespan = max(makespan, t.completionTime);
    }
    for (const auto& d : sol.drones) {
        makespan = max(makespan, d.completionTime);
    }
    sol.objective = makespan;


    return sol;
}

// Ki?m tra node có ph?i là station không
bool Solver::isStation(int node) const {
    for (const auto& station : instance.stationList) {
        if (station.id == node) return true;
    }
    return false;
}
