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

bool INSTANCE::loadFromFile3(const std::string& filename) {
    // Tự động lấy n, k, r, numtruck, UAVs, alpha từ tên file 
    // Format: P-n65-k10-r16-4-2-0.5.txt
    // hoặc: A-n46-k7-r16-7-2-1.txt
    
    std::smatch match;
    
    // Pattern: {base}-r{r}-{truck}-{uav}-{alpha}.txt
    std::regex re1("([A-Z]-n\\d+-k\\d+)-r(\\d+)-(\\d+)-(\\d+)-([0-9.]+)\\.txt");
    
    // Pattern fallback: {base}-{truck}-{uav}-{alpha}.txt (without r)
    std::regex re2("([A-Z]-n\\d+-k\\d+)-(\\d+)-(\\d+)-([0-9.]+)\\.txt");
    
    // Pattern: {base}-n{n}-k{k}-r{r} (extract E from r)
    std::regex re_nkr("-n(\\d+)-k(\\d+)-r(\\d+)");
    
    // Try to extract E from n-k-r pattern
    if (std::regex_search(filename, match, re_nkr)) {
        if (match.size() == 4) {
            E = std::stod(match[3].str()) * 2;
        }
    }
    
    // Try to extract truck, uav, alpha from filename
    if (std::regex_search(filename, match, re1)) {
        // Format: {base}-r{r}-{truck}-{uav}-{alpha}.txt
        num_trucks = std::stoi(match[3].str());
        UAVs = std::stoi(match[4].str());
        alpha = std::stod(match[5].str());
    } else if (std::regex_search(filename, match, re2)) {
        // Format: {base}-{truck}-{uav}-{alpha}.txt
        num_trucks = std::stoi(match[2].str());
        UAVs = std::stoi(match[3].str());
        alpha = std::stod(match[4].str());
    }
    // If no match, use defaults already set in main.cpp
    
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
    C.clear();
    
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
            double a = (alpha > 0.0) ? alpha : 1.0;
            tauprime[i][j] = tauprime[j][i] = dist / a;
        }
    }
    
    processStations();
    return true;
}

bool INSTANCE::loadFromFile4(const std::string& filename) {
    
    std::ifstream fin(filename);
    if (!fin.is_open()) {
        std::cerr << "Error: cannot open file " << filename << std::endl;
        return false;
    }

    vector<tuple<int,double,double,int>> rows;
    string line;
    while (getline(fin, line)) {
        if (line.empty()) continue;
        // detect whether line looks like CSV (has commas and 4 columns)
        std::stringstream ss(line);
        string s_id, s_x, s_y, s_type;
        if (!getline(ss, s_id, ',')) continue;
        if (!getline(ss, s_x, ',')) continue;
        if (!getline(ss, s_y, ',')) continue;
        if (!getline(ss, s_type, ',')) {
            // last field may not be followed by comma; try to read remainder
            s_type.clear();
            std::string rem;
            if (ss >> rem) s_type = rem;
        }
        try {
            int id = stoi(s_id);
            double x = stod(s_x);
            double y = stod(s_y);
            int type = 0;
            if (!s_type.empty()) {
                // handle values like "0.4" (tolerance) -> interpret int by rounding
                try {
                    type = stoi(s_type);
                } catch(...) {
                    // try parse as double then round to int
                    try {
                        double td = stod(s_type);
                        type = static_cast<int>(round(td));
                    } catch(...) {
                        type = 0;
                    }
                }
            }
            rows.emplace_back(id, x, y, type);
        } catch (...) {
            // not a CSV line -> fallback to original parser: stop and use old loader
            fin.close();
            // fallback: call previous implementation by parsing file with sections
            // Re-open and run old logic: reuse existing code path above by reading file again
            // Reset file and parse with section-based parser already implemented below in file.
            // To avoid duplicating code here, attempt to parse with the original loader steps:
            // (seek to beginning and re-run original parsing)
            std::ifstream fin2(filename);
            if (!fin2.is_open()) {
                std::cerr << "Error: cannot reopen file " << filename << std::endl;
                return false;
            }
            // Use the original section-based parser (replicate minimal previous behavior)
            nodes.clear();
            station_list.clear();
            C.clear();
            enum Section { NONE, DEPOT, CUSTOMERS, DRONE_STATIONS };
            Section section = NONE;
            while (getline(fin2, line)) {
                if (line.empty()) continue;
                if (line.find("*/ The Depot") != string::npos) { section = DEPOT; continue; }
                if (line.find("*/ The Demand Locations") != string::npos) { section = CUSTOMERS; continue; }
                if (line.find("*/ The Drone Stations") != string::npos) { section = DRONE_STATIONS; continue; }
                stringstream ss2(line);
                double x2, y2; char comma2;
                if (ss2 >> x2 >> comma2 >> y2) {
                    nodes.push_back({ x2, y2 });
                    if (section == DRONE_STATIONS) {
                        Stations st; st.id = nodes.size() - 1; st.x = x2; st.y = y2; st.num_drones = UAVs;
                        station_list.push_back(st);
                    }
                    if (section == CUSTOMERS) {
                        C.push_back(nodes.size() - 1);
                    }
                }
            }
            fin2.close();
            active_stations = station_list.size();
            n = nodes.size();
            C.erase(remove(C.begin(), C.end(), 0), C.end());
            tau.assign(nodes.size(), vector<double>(nodes.size(), 0.0));
            tauprime.assign(nodes.size(), vector<double>(nodes.size(), 0.0));
            for (int i = 0; i < n; ++i) {
                for (int j = i + 1; j < n; ++j) {
                    double dist = eucliddistance(nodes, i, j);
                    tau[i][j] = tau[j][i] = dist;
                    tauprime[i][j] = tauprime[j][i] = dist / (alpha > 0.0 ? alpha : 1.0);
                }
            }
            processStations();
            return true;
        }
    }
    fin.close();

    
    rows.pop_back();
        
    if (rows.empty()) {
        std::cerr << "Error: no CSV rows parsed from " << filename << std::endl;
        return false;
    }

    // Build internal structures from CSV rows
    nodes.clear();
    station_list.clear();
    C.clear();
    truck_only.clear();
    drone_only.clear();

    // keep given order so indices match file order (depot should be first row)
    for (size_t i = 0; i < rows.size(); ++i) {
        int id; double x, y; int type;
        std::tie(id, x, y, type) = rows[i];
        nodes.push_back({ x, y });
    }

    // classify nodes (skip depot as customer)
    for (size_t idx = 0; idx < rows.size(); ++idx) {
        if (idx == 0) continue; // depot
        int type = get<3>(rows[idx]);
        if (type == 2) {
            // drone station
            Stations st;
            st.id = static_cast<int>(idx);
            st.x = nodes[idx][0];
            st.y = nodes[idx][1];
            st.num_drones = 1;
            station_list.push_back(st);
        } else {
            // customer (either normal or truck-only)
            C.push_back(static_cast<int>(idx));
            if (type == 1) truck_only.push_back(static_cast<int>(idx));
        }
    }

    // Set drone parameters: range 6.25 (one-way), so E = 2*6.25
    alpha = 25.0;      // divisor for tauprime (euclid / 25)
    E = 6.25 * 2.0;    // used by processStations (E/2 = 6.25)
    active_stations = station_list.size();
    n = nodes.size();
    UAVs =1;
    num_trucks = 1;

    // Distance matrices: tau = Manhattan, tauprime = Euclidean / 25
    tau.assign(n, vector<double>(n, 0.0));
    tauprime.assign(n, vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            double m = manhatandistance(nodes, i, j);
            double e = eucliddistance(nodes, i, j);
            tau[i][j] = tau[j][i] = m/ alpha;
            tauprime[i][j] = tauprime[j][i] = e / alpha;
        }
    }

    // fill stations reachable customers & flight times using processStations
    processStations();
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
    cout << "Customers: " << C.size() << endl;
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
