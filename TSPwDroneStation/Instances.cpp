#include "Instances.h"
#include <regex>

bool INSTANCE::loadFromFile(const std::string& filename) {
    // Tự động lấy n, k, r từ tên file 
    std::smatch match;
    std::regex re("-n(\\d+)-k(\\d+)-r(\\d+)");
    if (std::regex_search(filename, match, re)) {
        if (match.size() == 4) {
            E = std::stod(match[3].str()) * 2; 
        }
    }
    ifstream fin(filename);
    if (!fin.is_open()) {
        cerr << "Error: cannot open file " << filename << endl;
        return false;
    }

    string line;
    enum Section { NONE, DEPOT, CUSTOMERS, DRONE_STATIONS };
    Section section = NONE;

    nodes.clear();
    station_list.clear();
    
    while (getline(fin, line)) {
        if (line.empty()) continue;

        if (line.find("*/ The Depot") != string::npos) {
            section = DEPOT;
            continue;
        }
        if (line.find("*/ The Demand Locations") != string::npos) {
            section = CUSTOMERS;
            continue;
        }
        if (line.find("*/ The Drone Stations") != string::npos) {
            section = DRONE_STATIONS;
            continue;
        }

        stringstream ss(line);
        double x, y;
        char comma;
        if (ss >> x >> comma >> y) {
            vector<double> point = { x, y };
            nodes.push_back(point);

            if (section == DRONE_STATIONS) {
                Stations st;
                st.id = nodes.size() - 1;
                st.x = x;
                st.y = y;
				st.num_drones = UAVs;
                station_list.push_back(st);
            }
            if (section == CUSTOMERS) {
				C.push_back(nodes.size() - 1); 
            }
        }
    }

    fin.close();
    active_stations = station_list.size();
    n = nodes.size();
	C.erase(remove(C.begin(), C.end(), 0), C.end()); 
    tau.assign(nodes.size(), vector<double>(nodes.size(), 0.0));
	tauprime.assign(nodes.size(), vector<double>(nodes.size(), 0.0));
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < nodes.size(); j++) {
            double dist = eucliddistance(nodes, i, j);
            tau[i][j] = tau[j][i] = dist;
            tauprime[i][j] = tauprime[j][i] = dist/alpha;
        }
    }
    processStations();
    return true;
}

bool INSTANCE::loadFromFile2(const std::string& filename) {
    std::ifstream fin(filename);
    if (!fin.is_open()) {
        std::cerr << "Error: cannot open file " << filename << std::endl;
        return false;
    }

    int n_station, n_customer;
    double speed_truck, speed_drone;
    fin >> n_station >> n_customer >> speed_truck >> speed_drone >> UAVs;
	n_station--; // trừ depot
    alpha = speed_drone / speed_truck;
	num_trucks = 1;
	UAVs = UAVs;
	active_stations = n_station;
    nodes.clear();
    station_list.clear();
    C.clear();
    truck_only.clear();
    drone_only.clear();
    vector<pair<double, double>> rawNodes;
    int total_nodes = 1 + n_customer + n_station;
    for (int i = 0; i < total_nodes; i++) {
        double x, y;
        fin >> x >> y;
        rawNodes.push_back({ x, y });
    }

    // Depot first
    nodes.push_back({ rawNodes[0].first , rawNodes[0].second }); // depot = 0

    // Customers next
    for (int i = 0; i < n_customer; i++) {
        double x = rawNodes[1 + n_station + i].first;
        double y = rawNodes[1 + n_station + i].second;
        nodes.push_back({ x, y });
        C.push_back(nodes.size() - 1);
        drone_only.push_back(nodes.size() - 1);
    }

    // Stations last
    for (int i = 0; i < n_station; i++) {
        double x = rawNodes[1 + i].first;
        double y = rawNodes[1 + i].second;
        nodes.push_back({ x, y });

        Stations st;
        st.id = nodes.size() - 1;
        st.x = x;
        st.y = y;
        st.num_drones = 0;
        station_list.push_back(st);
    }

    // --- Skip depot row ---
    for (int j = 0; j < n_customer + 1; j++) {
        int dummy;
        fin >> dummy; // bỏ qua hàng depot (node 0)
    }

    // --- Read station rows ---
    for (int i = 0; i < n_station; i++) {
        int stationIdx;
        fin >> stationIdx; // cột đầu tiên (số thứ tự station) -> bỏ
        for (int j = 0; j < n_customer; j++) {
            int r;
            fin >> r;
            if (r == 1) {
                int customerNodeId = 1 + j;  // customer bắt đầu từ node 1
                station_list[i].reachable_customers.push_back(customerNodeId);
                station_list[i].drone_only_nodes.push_back(customerNodeId);
            }
        }
    }


    // --- Distance matrix ---
    n = nodes.size();
    tau.assign(nodes.size(), vector<double>(nodes.size(), 0.0));
    tauprime.assign(nodes.size(), vector<double>(nodes.size(), 0.0));
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < nodes.size(); j++) {
            tau[i][j] = tau[j][i] = manhatandistance(nodes, i, j)/speed_truck;
            tauprime[i][j] = tauprime[j][i] = eucliddistance(nodes,i,j)/speed_drone;
        }
    }
    for (auto& st : station_list) {
        st.flight_time.clear();
        for (int c : st.reachable_customers) {
            double d1 = eucliddistance(nodes, st.id, c);
            double flight_time = d1 * 2 / speed_drone;
            st.flight_time.push_back(flight_time);
        }
    }

    fin.close();
    return true;
}



void INSTANCE::processStations() {
    for (auto& st : station_list) {
        st.max_flight = E / 2.0; 

        for (int c = 1; c < nodes.size(); c++) { 
			if (c == st.id) continue;
			// if c is a station, skip
			if (any_of(station_list.begin(), station_list.end(), [c](const Stations& s) { return s.id == c; })) {
				continue;
			}

            double d1 = eucliddistance(nodes, st.id, c);
            if (2 * d1 <= E) { 
                st.reachable_customers.push_back(c);
                st.flight_time.push_back(d1 *2/ alpha); // thời gian bay (tốc độ alpha)
            }
        }
    }
}

void INSTANCE::displayData() {
    cout << "Number of nodes: " << n << endl;
    for(auto& node : nodes) {
        cout <<  node[0] << ", " << node[1]  << endl;
	}
	cout << "Distance matrix (tau):" << endl;
    for (const auto& row : tau) {
        for (double val : row) {
            cout << val << " ";
        }
        cout << endl;
	}
	cout << "Distance matrix (tauprime):" << endl;
    for (const auto& row : tauprime) {
        for (double val : row) {
            cout << val << " ";
        }
        cout << endl;
    }
    cout << "Drone stations: " << station_list.size() << endl;
    for (auto& st : station_list) {
        cout << "Station " << st.id << " (" << st.x << "," << st.y << ") E/2=" << st.max_flight << endl;
        cout << "  Reachable customers: ";
        for (size_t i = 0; i < st.reachable_customers.size(); i++) {
            int cid = st.reachable_customers[i];
            double t = st.flight_time[i];
            cout << cid << "(time=" << t << ") ";
        }
        cout << endl;
    }
    for(int c : C) {
        cout << "Customer: " << c << " (" << nodes[c][0] << "," << nodes[c][1] << ")" << endl;
	}
	cout << "Active stations: " << active_stations << endl;
    cout << "UAVs per station: " << UAVs << endl;
    cout << "Max endurance (E): " << E << endl;
	cout << "Drone speed factor (alpha): " << alpha << endl;
    cout << "Number of trucks: " << num_trucks << endl;
    cout << "Truck-only nodes: ";
    for (int node : truck_only) {
        cout << node << " ";
    }
	cout << endl;
	cout << "Drone-only nodes: ";
    for (int node : drone_only) {
        cout << node << " ";
	}
	cout << endl;
	cout << "Customers: ";
    for (int c : C) {
        cout << c << " ";
	}
}

double eucliddistance(const vector<vector<double>>& mat, int node1, int node2) {
    double dx = mat[node1][0] - mat[node2][0];
    double dy = mat[node1][1] - mat[node2][1];
    return sqrt(dx * dx + dy * dy);
}

double manhatandistance(const vector<vector<double>>& mat, int node1, int node2) {
    double dx = fabs(mat[node1][0] - mat[node2][0]);
    double dy = fabs(mat[node1][1] - mat[node2][1]);
    return dx + dy;
}