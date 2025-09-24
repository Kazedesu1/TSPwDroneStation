#include "Instances.h"
#include <regex>

bool INSTANCE::loadFromFile(const std::string& filename) {
    // --- Tự động lấy n, k, r từ tên file ---
    std::smatch match;
    std::regex re("-n(\\d+)-k(\\d+)-r(\\d+)");
    if (std::regex_search(filename, match, re)) {
        if (match.size() == 4) {
            num_trucks = std::stoi(match[2].str());
            E = std::stod(match[3].str()) * 2; // r là E/2, nên E = r*2
        }
    }
    if( num_trucks >=6) {
        active_stations = 6;
    }
	else active_stations = 4;
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
                station_list.push_back(st);
            }
            if (section == CUSTOMERS) {
				C.push_back(nodes.size() - 1); 
            }
        }
    }

    fin.close();

    n = nodes.size();
	C.erase(remove(C.begin(), C.end(), 0), C.end()); 
    tau.assign(nodes.size(), vector<double>(nodes.size(), 0.0));
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < nodes.size(); j++) {
            double dist = eucliddistance(nodes, i, j);
            tau[i][j] = tau[j][i] = dist;
        }
    }
    processStations();
    return true;
}

void INSTANCE::processStations() {
    for (auto& st : station_list) {
        st.max_flight = E / 2.0; 

        for (int c = 1; c < nodes.size(); c++) { 
			if (c == st.id) continue; 
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
}

double eucliddistance(const vector<vector<double>>& mat, int node1, int node2) {
    double dx = mat[node1][0] - mat[node2][0];
    double dy = mat[node1][1] - mat[node2][1];
    return sqrt(dx * dx + dy * dy);
}
