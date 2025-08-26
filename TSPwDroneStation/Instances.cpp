#include "Instances.h"

bool INSTANCE::loadFromFile(const std::string& filename) {
    ifstream fin(filename);
    if (!fin.is_open()) {
        cerr << "Error: cannot open file " << filename << endl;
        return false;
    }

    string line;
    enum Section { NONE, DEPOT, CUSTOMERS, DRONE_STATIONS };
    Section section = NONE;

    nodes.clear();
    stationList.clear();

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
                stationList.push_back(st);
            }
            if (section == CUSTOMERS) {
				C.push_back(nodes.size() - 1); 
            }
        }
    }

    fin.close();

    n = nodes.size();
	C.erase(remove(C.begin(), C.end(), 0), C.end()); 
    tau.assign(n, vector<double>(n, 0.0));
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            double dist = eucliddistance(nodes, i, j);
            tau[i][j] = tau[j][i] = dist;
        }
    }
    processStations();
    return true;
}

void INSTANCE::processStations() {
    for (auto& st : stationList) {
        st.maxFlight = E / 2.0; 

        for (int c = 1; c < n; c++) { 
			if (c == st.id) continue; 
            double d1 = eucliddistance(nodes, st.id, c);
            if (2 * d1 <= E) { 
                st.reachableCustomers.push_back(c);
                st.flightTime.push_back(d1 / alpha); // thời gian bay (tốc độ alpha)
            }
        }
    }
    for (int c = 1; c < n; c++) {
        bool reachable = false;
        for (auto& st : stationList) {
            for (int rc : st.reachableCustomers) {
                if (rc == c) {
                    reachable = true;
                    break;
                }
            }
            if (reachable) break;
        }
        if (!reachable) {
            truckonly.push_back(c);
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
    cout << "Drone stations: " << stationList.size() << endl;
    for (auto& st : stationList) {
        cout << "Station " << st.id << " (" << st.x << "," << st.y << ") E/2=" << st.maxFlight << endl;
        cout << "  Reachable customers: ";
        for (size_t i = 0; i < st.reachableCustomers.size(); i++) {
            int cid = st.reachableCustomers[i];
            double t = st.flightTime[i];
            cout << cid << "(time=" << t << ") ";
        }
        cout << endl;
    }
	cout << "Active stations: " << activeStations << endl;
    cout << "UAVs per station: " << UAVs << endl;
    cout << "Max endurance (E): " << E << endl;
	cout << "Drone speed factor (alpha): " << alpha << endl;
    cout << "Number of trucks: " << numtrucks << endl;
    cout << "Truck-only nodes: ";
    for (int node : truckonly) {
        cout << node << " ";
    }
	cout << endl;
}

double eucliddistance(const vector<vector<double>>& mat, int node1, int node2) {
    double dx = mat[node1][0] - mat[node2][0];
    double dy = mat[node1][1] - mat[node2][1];
    return sqrt(dx * dx + dy * dy);
}
