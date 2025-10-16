#include "Solver.h"
#include "Param.h"
#include <iostream>
#include <algorithm>
#include <limits>
#include <cstdlib>
#include <unordered_map>
#include <ctime>
#include <bitset>
#include <unordered_set>
using namespace std;

Solver::Solver(INSTANCE& inst) : instance(inst) {}

void Solver::solve() {
    // --- 1. Khởi tạo lời giải ban đầu ---
    Solution s0(instance);
    s0 = greedyInsertion(s0);
    firstObjective = s0.objective;

    Solution s_star = s0;
    Solution s_best = s0;

    // --- Thông số điều khiển ---
    int maxIter = 10000;     // có thể chỉnh
    double Delta = 0.1;     // (1 + Δ) là ngưỡng chấp nhận
    double epsilon = 0.95;  // hệ số giảm Δ sau mỗi vòng
    int nonimproved_threshold = 1000; // số vòng không cải thiện liên tiếp thì dừng
    int nonimproved_count = 0;


    for (int iter = 0; iter < maxIter; iter++) {
        // --- 2. Ruin & Recreate từ s* ---
        Solution destroyed = RandomRemoveStation(s_star);

        double r = Param::real_random_generator(0, 1);
        if (r < 0.5) {
            destroyed = WorstRemoval(destroyed);
        }
        else {
            destroyed = RandomRemoval(destroyed);
        }

		destroyed = RandomRemoveDroneNode(destroyed);
        Solution s = greedyInsertion(destroyed);

        // --- 3. Quy tắc chấp nhận để cập nhật s* ---
        double threshold = s_star.objective * (1.0 + Delta);
        if (s.objective < threshold) {
            s_star = s;

            // --- 4. Có tốt hơn s_best không? ---
            if (s_star.objective < s_best.objective) {
                s_best = s_star;
                nonimproved_count = 0; // reset nếu có cải thiện
            }
            else {
                nonimproved_count++;
            }
        }
        else {
            nonimproved_count++;
        }

        // --- 5. Giảm Δ ---
        Delta *= epsilon;

        // (Tùy chọn) In log
        std::cout << "Iter " << iter
            << " s*=" << s_star.objective
            << " best=" << s_best.objective
            << " Delta=" << Delta
            << " nonimproved=" << nonimproved_count
            << std::endl;

        // --- Dừng sớm nếu không cải thiện đủ threshold ---
        if (nonimproved_count >= nonimproved_threshold) {
            std::cout << "Stop: nonimproved threshold reached (" << nonimproved_threshold << ")\n";
            break;
        }
    }

    // --- Kết thúc ---
    std::cout << "Final best solution: " << std::endl;
    s_best.display();
    s_best.feasiblecheck();
    bestObjective = s_best.objective;
}



Solution Solver::greedyInsertion(Solution solution) {
    Solution sol(instance);
    sol = solution;
    int num_trucks = instance.num_trucks;
    int UAVs = instance.UAVs;

    // --- Chọn station bằng hàm mới ---
    vector<int> active_stations = selectStations(sol);

    // --- Đưa các station này vào customerQueue ---
    vector<int> customerQueue = sol.remaining_customers;
    for (int s : active_stations) {
        customerQueue.push_back(s);
    }
   
    // Xóa các station được chọn khỏi remaining_stations (chỉ danh sách, actual activation khi chèn vào route)
    for (int s : active_stations) {
        sol.remaining_stations.erase(
            std::remove(sol.remaining_stations.begin(), sol.remaining_stations.end(), s),
            sol.remaining_stations.end()
        );
    }

    // Shuffle queue
    Param::shuffle(customerQueue);

    // --- Tập khách hàng có thể đi drone (ban đầu lấy reachable của activated stations) ---
    vector<int> droneElit;
    for (int s : sol.activated_stations) {
        int idx = getStationIndexById(s);
        if (idx >= 0) {
            for (int c : instance.station_list[idx].reachable_customers)
                droneElit.push_back(c);
        }
    }
    // loại trùng
    sort(droneElit.begin(), droneElit.end());
    droneElit.erase(unique(droneElit.begin(), droneElit.end()), droneElit.end());
    
    // --- Bắt đầu greedy insertion ---
    while(!customerQueue.empty()) {
        int customer = customerQueue.front();
        customerQueue.erase(customerQueue.begin());
        double bestcost = std::numeric_limits<double>::max();
        int bestTruck = -1, bestPos = -1;
        int bestStationIdx = -1, bestDroneIdx = -1;
        int totruck = -1;
        double best_wait_time = 0.0;

        // ====== TRUCK INSERTION ======
        if (std::find(instance.drone_only.begin(), instance.drone_only.end(), customer) == instance.drone_only.end()) {
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
                        bestTruck = t;
                        bestPos = pos;
                        totruck = 1;
                    }
                }
            }
        }

        // ====== DRONE INSERTION (chỉ nếu customer trong droneElit) ======
        if (std::find(droneElit.begin(), droneElit.end(), customer) != droneElit.end()) {
            // Duyệt theo station-level
            for (int s = 0; s < (int)sol.drones.size(); ++s) {
                if (std::find(instance.station_list[s].reachable_customers.begin(), instance.station_list[s].reachable_customers.end(), customer) != instance.station_list[s].reachable_customers.end()) {
                    if (instance.station_list[s].num_drones > 0) {
                        for (int d = 0; d < (int)sol.drones[s].size(); ++d) {
                            int station_id = sol.drones[s][d].station_id;
                            double flight_time = sol.drones[s][d].completion_time
                                + instance.tauprime[station_id][customer] * 2.0;

                            if (flight_time < bestcost) {
                                bestcost = flight_time;
                                bestStationIdx = s;
                                bestDroneIdx = d;
                                totruck = 0;

                            }
                        }
                    }
                    else {
                        int best_d = -1;
                        double bestcompletion = std::numeric_limits<double>::max();

                        // Tìm drone có completion_time nhỏ nhất
                        for (int d = 0; d < (int)sol.drones[s].size(); ++d) {
                            if (sol.drones[s][d].completion_time < bestcompletion) {
                                bestcompletion = sol.drones[s][d].completion_time;
                                best_d = d;
                            }
                        }

                        // Nếu không có drone nào hợp lệ thì bỏ qua
                        if (best_d == -1) continue;

                        int station_id = sol.drones[s][best_d].station_id;
                        double flight_time = sol.drones[s][best_d].completion_time
                            + instance.tauprime[station_id][customer] * 2.0 - sol.station_time[station_id];

                        double max_flight_time = 0.0;
                        for (int d = 0; d < (int)sol.drones[s].size(); ++d) {
                            max_flight_time = std::max(max_flight_time, sol.drones[s][d].completion_time - sol.station_time[station_id]);
                        }
						max_flight_time = std::max(max_flight_time, flight_time);
                        double delta_wait_time = max_flight_time - sol.wait_time[station_id];
                        double truckCompletion = sol.trucks[sol.station_truck[station_id]].completion_time + delta_wait_time;

                        if (truckCompletion < bestcost) {
                            bestcost = truckCompletion;
                            bestStationIdx = s;
                            bestDroneIdx = best_d;
                            totruck = 2;
                            best_wait_time = max_flight_time;
                            bestTruck = sol.station_truck[station_id];
                        }
                    }
                }
            }
        }

        // ====== THỰC HIỆN INSERTION ======
        if (totruck == 1) {
            // ---- Insert vào truck ----
            sol.trucks[bestTruck].route.insert(sol.trucks[bestTruck].route.begin() + bestPos, customer);
            sol.trucks[bestTruck].completion_time = bestcost;
            sol.vehicle_of_customer[customer] = bestTruck;

            // Nếu node này là 1 station
            if (isStation(customer)) {
                // Tính thời gian truck đến station, cộng thêm wait_time nếu station không có drone
                double truckArriveStation = 0.0;
                for (int i = 1; i <= bestPos; ++i) {
                    int u = sol.trucks[bestTruck].route[i - 1];
                    int v = sol.trucks[bestTruck].route[i];
                    truckArriveStation += instance.tau[u][v];
                    if (isStation(v) && instance.station_list[v-instance.C.size()-1].num_drones == 0) {
                        truckArriveStation += sol.wait_time[v];
					}
                }
                
                sol.station_time[customer] = truckArriveStation;
				int stIndex = customer - (int)instance.C.size() - 1;
                // Tạo vector drone mới cho station này
                if (instance.station_list[stIndex].num_drones > 0) {
                    for (int d = 0; d < instance.station_list[stIndex].num_drones; ++d) {
                        Solution::DroneRoute dr;
                        dr.station_id = customer;
                        dr.drone_id = d;
                        dr.completion_time = truckArriveStation;
                        sol.drones[customer - (int)instance.C.size() - 1].push_back(dr);
                        sol.station_truck[dr.station_id] = bestTruck;

                    }
                }
                else {
                    for (int d = 0; d < instance.UAVs; ++d) {
                        Solution::DroneRoute dr;
                        dr.station_id = customer;
                        dr.drone_id = d;
                        dr.completion_time = truckArriveStation;
                        sol.drones[customer - (int)instance.C.size() - 1].push_back(dr);
						sol.station_truck[dr.station_id] = bestTruck;
                    }
                }

                sol.activated_stations.push_back(customer);

                // Thêm các khách hàng có thể với droneElit
                for (int c : instance.station_list[stIndex].reachable_customers) {
                    droneElit.push_back(c);
                }

                // Xóa station khỏi remaining_stations
                sol.remaining_stations.erase(
                    std::remove(sol.remaining_stations.begin(), sol.remaining_stations.end(), customer),
                    sol.remaining_stations.end()
                );
            }

            // Cập nhật time cho station sau
            int u = sol.trucks[bestTruck].route[bestPos - 1];
            int v = sol.trucks[bestTruck].route[bestPos + 1];
            double delta = instance.tau[u][customer] + instance.tau[customer][v] - instance.tau[u][v];

            for (int i = bestPos + 1; i < (int)sol.trucks[bestTruck].route.size(); ++i) {
                int node = sol.trucks[bestTruck].route[i];
                if (isStation(node)) {
                    sol.station_time[node] += delta;
                    // Đồng thời cập nhật thời gian cho mọi drone của station đó
                    for (int j = 0; j < (int)sol.drones[node - (int)instance.C.size() - 1].size(); ++j) {
						sol.drones[node - (int)instance.C.size() - 1][j].completion_time += delta;
                    }
                }
            }
        }
        else if (totruck == 0) {
            // ===== Insert vào drone =====
            auto& drRoute = sol.drones[bestStationIdx][bestDroneIdx];
            drRoute.customers.push_back(customer);
            int station_id = drRoute.station_id;
            drRoute.completion_time += instance.tauprime[station_id][customer] * 2.0 ;

        }
        else if (totruck == 2) {
            // ===== Chờ truck tới station không có drone để deploy khách =====
            // bestStationIdx là chỉ số station trong drones & station_list
            int stationNode = instance.C.size() + 1 + bestStationIdx;
            int truck_id = sol.station_truck[stationNode] = bestTruck;
            ;

            // 1) Cập nhật completion_time của truck theo bestcost
            sol.trucks[truck_id].completion_time += best_wait_time - sol.wait_time[stationNode];

            

            // 3) Insert customer vào drone tốt nhất
            auto& drRoute = sol.drones[bestStationIdx][bestDroneIdx];
            drRoute.customers.push_back(customer);
            int station_id = drRoute.station_id;
            drRoute.completion_time += instance.tauprime[station_id][customer] * 2.0 ;

            // ====== Cập nhật thời gian các station sau đó (nếu truck bị delay) ======
            auto& truck = sol.trucks[truck_id];

            // Vì ở totruck = 2 không có bestPos, ta xác định vị trí station trong route
            auto itPos = std::find(truck.route.begin(), truck.route.end(), stationNode);

            // Nếu station này nằm trong route truck thì mới cập nhật
            if (itPos != truck.route.end()) {
                int pos = std::distance(truck.route.begin(), itPos);

                // Độ trễ thêm chính là delta_wait_time (hoặc best_wait_time - old_wait_time)
                double delta = best_wait_time - sol.wait_time[stationNode];

                // Cập nhật các station phía sau trên cùng truck
                for (int i = pos + 1; i < (int)truck.route.size(); ++i) {
                    int node = truck.route[i];
                    if (isStation(node)) {
                        sol.station_time[node] += delta;

                        // Đồng thời cập nhật cho drone tại station đó
                        int stIndex2 = node - (int)instance.C.size() - 1;
                        for (auto& dr : sol.drones[stIndex2]) {
                            dr.completion_time += delta;
                        }
                    }
                }
            }
            // 2) Gán wait_time mới cho station
            sol.wait_time[stationNode] = best_wait_time;
        }
        else if (totruck == -1)
        {
			customerQueue.push_back(customer);
            continue;
        }

        // Xóa customer khỏi remaining_customers
        sol.remaining_customers.erase(
            std::remove(sol.remaining_customers.begin(), sol.remaining_customers.end(), customer),
            sol.remaining_customers.end()
        );
    }

    // cleanup
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

Solution Solver::removeStation(Solution& sol, int stationNode) {
    Solution newSol = sol;

    int stationIndex = stationNode - instance.C.size() - 1;
    
    double removed_wait_time = 0;
    int truckID =-1;
	bool hasOwnDrones = (instance.station_list[stationIndex].num_drones > 0);

    //  TH1: Station có drone riêng → thu hồi toàn bộ drone & customer
    if (hasOwnDrones) {
        for (auto& dr : newSol.drones[stationIndex]) {
            for (int c : dr.customers) {
                newSol.remaining_customers.push_back(c);
            }
        }
        newSol.drones[stationIndex].clear();

        //  Lấy wait_time cũ
        removed_wait_time = newSol.wait_time[stationNode];
        newSol.wait_time[stationNode] = 0.0;

        truckID = newSol.station_truck[stationNode];
        newSol.station_truck[stationNode] = 0;
    }
    else {
        //  TH2: Station không có drone riêng


        // Giảm completion_time của truck
        newSol.trucks[truckID].completion_time -= removed_wait_time;

    }

    //  Xóa station khỏi truck route (dùng chung cho cả 2 TH)
    auto& truck = newSol.trucks[truckID];
    auto it = std::find(truck.route.begin(), truck.route.end(), stationNode);
    if (it != truck.route.end()) {
        int pos = std::distance(truck.route.begin(), it);
        int prevNode = truck.route[pos - 1];
        int nextNode = truck.route[pos + 1];

        double delta = instance.tau[prevNode][stationNode]
            + instance.tau[stationNode][nextNode]
            - instance.tau[prevNode][nextNode];

        truck.completion_time -= delta;
        truck.route.erase(it);

        // Cập nhật thời gian cho các station phía sau
        for (int i = pos; i < (int)truck.route.size(); ++i) {
            int node = truck.route[i];
            if (isStation(node)) {
                newSol.station_time[node] = newSol.station_time[node] - delta - removed_wait_time;
                int stIndex = getStationIndexById(node);
                if (stIndex >= 0) {
                    for (auto& dr : newSol.drones[stIndex]) {
                        dr.completion_time = dr.completion_time  - delta - removed_wait_time;
                    }
                }
            }
        }
    }
    //  Cập nhật danh sách activated / remaining
    newSol.activated_stations.erase(
        std::remove(newSol.activated_stations.begin(),
            newSol.activated_stations.end(),
            stationNode),
        newSol.activated_stations.end()
    );
    newSol.remaining_stations.push_back(stationNode);

    newSol.objective = newSol.calculateMakespan();
    return newSol;
}



void Solver::removeUnusedStations(Solution& sol) {
    Solution newSol = sol;

    // Tìm các station còn sử dụng ít nhất 1 drone
    unordered_set<int> usedStations;
    for (int s = 0; s < (int)newSol.drones.size(); ++s) {
        for (const auto& dr : newSol.drones[s]) {
            if (!dr.customers.empty()) {
                usedStations.insert(dr.station_id);
            }
        }
    }

    // Loại bỏ các station đã activate nhưng không dùng
    for (int stationNode : sol.activated_stations) {
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
    size_t num_to_remove = round(all_customers.size() * Param::real_random_generator(0.1,0.2));

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
    size_t num_station_to_remove = (int)newSol.activated_stations.size() / 2;
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
	if (instance.truck_only.size() == 0) return newSol;
    else { 
    vector<int> truck_customer;
        for (auto& truck : newSol.trucks) {
            for (int node : truck.route) {
                if (!isStation(node) && node != 0) {
                    truck_customer.push_back(node);
                }
            }
        }
	    vector<int> customers_list = truck_customer;
        double delta = Param::real_random_generator(0.1, 0.2);
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
}

void Solver::removeCustomerFromTruck(Solution& sol, int truckId, int customer) {
    auto& truck = sol.trucks[truckId];
    auto it = std::find(truck.route.begin(), truck.route.end(), customer);
    if (it == truck.route.end()) return;

    int pos = std::distance(truck.route.begin(), it);
    int prevNode = truck.route[pos - 1];
    int nextNode = truck.route[pos + 1];

    double delta = instance.tau[prevNode][customer]
        + instance.tau[customer][nextNode]
        - instance.tau[prevNode][nextNode];

    // Xóa customer khỏi route truck
    truck.route.erase(it);
    truck.completion_time -= delta;

    // Cập nhật thời gian cho các station phía sau
    for (int i = pos; i < (int)truck.route.size(); ++i) {
        int node = truck.route[i];
        if (isStation(node)) {
            sol.station_time[node] -= delta;

            //  Duyệt đúng theo cấu trúc mới: vector<vector<DroneRoute>>
            for (auto& droneGroup : sol.drones) {
                for (auto& dr : droneGroup) {
                    if (dr.station_id == node) {
                        dr.completion_time -= delta;
                    }
                }
            }
        }
    }

    // Đưa customer về remaining
    sol.remaining_customers.push_back(customer);
}



// trả về index trong instance.station_list cho station id (hoặc -1 nếu ko tìm)
int Solver::getStationIndexById(int id) const {
    for (size_t i = 0; i < instance.station_list.size(); ++i) {
        if (instance.station_list[i].id == id) return static_cast<int>(i);
    }
    return -1;
}

// Hàm chèn lần lượt các node vào các truck routes (best insertion) và trả về tổng delta cost
double Solver::tspCost(const Solution& sol_in, const vector<int>& nodes) {
    Solution temp = sol_in;
    double total_cost = 0.0;

    // Best Insertion: tại mỗi bước, chọn customer có chi phí chèn nhỏ nhất vào bất kỳ truck
    vector<int> to_insert = nodes;
    while (!to_insert.empty()) {
        double best_delta = std::numeric_limits<double>::infinity();
        int best_truck = -1;
        int best_pos = -1;
        int best_cus_idx = -1;

        // Duyệt tất cả customer còn lại
        for (size_t cus_idx = 0; cus_idx < to_insert.size(); ++cus_idx) {
            int x = to_insert[cus_idx];
            // Duyệt tất cả truck và vị trí chèn
            for (size_t t = 0; t < temp.trucks.size(); ++t) {
                const auto& route = temp.trucks[t].route;
                for (size_t pos = 1; pos < route.size(); ++pos) {
                    int u = route[pos - 1];
                    int v = route[pos];
                    double delta = instance.tau[u][x] + instance.tau[x][v] - instance.tau[u][v];
                    if (delta < best_delta) {
                        best_delta = delta;
                        best_truck = static_cast<int>(t);
                        best_pos = static_cast<int>(pos);
                        best_cus_idx = static_cast<int>(cus_idx);
                    }
                }
            }
        }

        if (best_truck != -1 && best_cus_idx != -1) {
            int x = to_insert[best_cus_idx];
            temp.trucks[best_truck].route.insert(temp.trucks[best_truck].route.begin() + best_pos, x);
            total_cost += best_delta;
            temp.trucks[best_truck].completion_time += best_delta;
            to_insert.erase(to_insert.begin() + best_cus_idx);
        } else {
            // Không tìm được vị trí hợp lệ, bỏ qua customer này
            to_insert.erase(to_insert.begin());
        }
    }

    return total_cost;
}

vector<int> Solver::selectStations(const Solution& sol) {
    vector<int> bestSet;
    double bestScore = -1e20;

    const auto& remaining_stations = sol.remaining_stations;
    int maxStations = instance.active_stations;

    vector<int> droneOnlyRemaining;
    for (int c : instance.drone_only) {
        bool served = false;
        for (int s : sol.activated_stations) {
            int stIndex = getStationIndexById(s);
            if (stIndex >= 0) {
                if (std::find(instance.station_list[stIndex].reachable_customers.begin(),
                    instance.station_list[stIndex].reachable_customers.end(), c) != instance.station_list[stIndex].reachable_customers.end()) {
                    served = true; // station đã phục vụ customer này
                    break;
                }
            }
        }
        if (served) continue;
        if (std::find(sol.remaining_customers.begin(), sol.remaining_customers.end(), c)
            != sol.remaining_customers.end())
        {
            droneOnlyRemaining.push_back(c);
        }
    }
    

    unordered_set<int> droneOnlySet(droneOnlyRemaining.begin(), droneOnlyRemaining.end());

    vector<StationScore> stationScores;
    stationScores.reserve(remaining_stations.size());

    for (int s : remaining_stations) {
        const auto& st = instance.station_list[s - instance.C.size() - 1];
        StationScore ss = computeStationScore(sol, st);
        stationScores.push_back(ss);
    }

    int n = stationScores.size();
    int totalMask = (1 << n);
    // Duyệt tất cả tổ hợp station bằng bitmask
    for (int mask = 1; mask < totalMask; ++mask) {

        // Bỏ qua nếu số station chọn vượt quá giới hạn
        if (std::bitset<32>(mask).count() > maxStations) continue;

        std::vector<int> currentSet;
        std::unordered_set<int> coveredDroneOnly;
        double sumScore = 0.0;
        
        // Thu thập thông tin của tổ hợp hiện tại
        for (int i = 0; i < n; i++) {
            if (mask & (1 << i)) {
                const StationScore& ss = stationScores[i];
                currentSet.push_back(ss.station_id);
                sumScore += ss.score;
                coveredDroneOnly.insert(ss.covered_drone_only.begin(), ss.covered_drone_only.end());
            }
        }

        // Kiểm tra xem đã phủ hết drone-only chưa
        bool coversAll = std::all_of(droneOnlySet.begin(), droneOnlySet.end(),
            [&](int dn) { return coveredDroneOnly.count(dn); });

        if (!coversAll) continue;

        // Cập nhật nghiệm tốt nhất
        if (sumScore > bestScore) {
            bestScore = sumScore;
            bestSet = currentSet;
        }
    }

    return bestSet;
}


StationScore Solver::computeStationScore(const Solution& sol, const INSTANCE::Stations& st) {
    StationScore ss;
    ss.station_id = st.id;
    ss.score = -1e9;

    // Lấy danh sách remaining customers mà station này có thể phục vụ
    vector<int> filtered_customers;
    for (int c : st.reachable_customers) {
        if (std::find(sol.remaining_customers.begin(), sol.remaining_customers.end(), c) != sol.remaining_customers.end()) {
            filtered_customers.push_back(c);
        }
    }

    if (filtered_customers.empty()) {
        // không còn khách cần phục vụ bởi station này
        ss.score = -1e9;
        return ss;
    }

    // cost nếu truck phải ghé tất cả các customer đó (chèn tuần tự bằng best insertion)
    double cost_customers = tspCost(sol, filtered_customers);

    // cost nếu truck chỉ chèn station (treat station.id as single node)
    double cost_station = tspCost(sol, vector<int>{ st.id });

    // penalty nếu station không có drone sẵn: theo cách B (tổng flight_time / (alpha * UAVs))
    double penalty_wait = 0.0;
    if (st.num_drones == 0) {
        double total_flight = 0.0;
        for (size_t i = 0; i < st.reachable_customers.size(); ++i) {
            int cust = st.reachable_customers[i];
            if (std::find(filtered_customers.begin(), filtered_customers.end(), cust) != filtered_customers.end()) {
                total_flight += instance.tauprime[ss.station_id][cust]*2;
            }
        }
        if (instance.UAVs > 0)
            penalty_wait = total_flight / instance.UAVs;
        else
            penalty_wait = total_flight; // fallback
    }

    ss.score = (cost_customers - cost_station) - penalty_wait;

    // covered drone-only nodes from remaining only
    for (int c : st.drone_only_nodes) {
        if (std::find(sol.remaining_customers.begin(), sol.remaining_customers.end(), c) != sol.remaining_customers.end()) {
            ss.covered_drone_only.push_back(c);
        }
    }

    return ss;
}

Solution Solver::RandomRemoveDroneNode(Solution& sol) {
    Solution newSol = sol;

    // Gom tất cả drone node đang phục vụ (không rỗng)
    vector<tuple<int, int, int>> all_drone_nodes; // (customer, station_idx, drone_idx)
    for (int s = 0; s < (int)newSol.drones.size(); ++s) {
        for (int d = 0; d < (int)newSol.drones[s].size(); ++d) {
            for (int c : newSol.drones[s][d].customers) {
                all_drone_nodes.push_back(make_tuple(c, s, d));
            }
        }
    }

    // Shuffle danh sách
    std::shuffle(all_drone_nodes.begin(), all_drone_nodes.end(), Param::mt);

    // Số lượng cần xóa = 30% tổng drone node đang phục vụ
    size_t num_to_remove = round(all_drone_nodes.size() * Param::real_random_generator(0.1, 0.2));

    // Xóa lần lượt
    for (size_t k = 0; k < num_to_remove; ++k) {
        int customer = std::get<0>(all_drone_nodes[k]);
        int station_idx = std::get<1>(all_drone_nodes[k]);
        int drone_idx = std::get<2>(all_drone_nodes[k]);

        // Xóa customer khỏi drone route
        auto& dr = newSol.drones[station_idx][drone_idx];
        auto it = std::find(dr.customers.begin(), dr.customers.end(), customer);
        if (it != dr.customers.end()) {
            dr.customers.erase(it);
        }
        // Đưa customer về remaining
        newSol.remaining_customers.push_back(customer);
    }

    // --- Tính lại thời gian cho truck, station, drone, wait_time ---
    // Reset
    for (auto& truck : newSol.trucks) truck.completion_time = 0.0;
    std::fill(newSol.station_time.begin(), newSol.station_time.end(), 0.0);
    std::fill(newSol.wait_time.begin(), newSol.wait_time.end(), 0.0);
    for (auto& droneGroup : newSol.drones)
        for (auto& dr : droneGroup) dr.completion_time = 0.0;

    // Duyệt từng truck
    for (auto& truck : newSol.trucks) {
        double time = 0.0;
        for (size_t i = 1; i < truck.route.size(); ++i) {
            int u = truck.route[i - 1];
            int v = truck.route[i];
            time += instance.tau[u][v];

            // Nếu v là station
            if (isStation(v)) {
                newSol.station_time[v] = time;

                // Tính thời gian drone phục vụ tại station v
                int stIdx = getStationIndexById(v);
                double max_drone_time = 0.0;
                for (auto& dr : newSol.drones[stIdx]) {
                    double drone_time = 0.0;
                    for (int c : dr.customers) {
                        drone_time += instance.tauprime[v][c] * 2.0 ;
                    }
                    dr.completion_time = newSol.station_time[v] + drone_time;
                    max_drone_time = std::max(max_drone_time, drone_time);
                }
                // Nếu station không có drone riêng thì cộng wait_time vào truck
                if (instance.station_list[stIdx].num_drones == 0) {
                    newSol.wait_time[v] = max_drone_time;
                    time += max_drone_time;
                }
            }
        }
        truck.completion_time = time;
    }
	removeUnusedStations(newSol);
    // Tính lại objective
    newSol.objective = newSol.calculateMakespan();
    return newSol;
}