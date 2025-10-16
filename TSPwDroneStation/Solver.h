#pragma once
#ifndef SOLVER_H
#define SOLVER_H

#include <vector>
#include <set>
#include "Instances.h"
#include "Solution.h"
using namespace std;

// --- thêm struct phụ trợ cho score ---
struct StationScore {
	int station_id;
	double score;
	vector<int> covered_drone_only;
};

class Solver {
public:
	Solver(INSTANCE& instance);
	void solve();
	INSTANCE instance;
	Solution greedyInsertion(Solution sol);
	bool isStation(int node) const;
	Solution Ruin(Solution& sol);
	Solution removeStation(Solution& sol, int stationNode);
	void removeUnusedStations(Solution& sol);
	Solution RandomRemoval(Solution& sol);
	void removeCustomerFromTruck(Solution& sol, int truckId, int customer);
	Solution WorstRemoval(Solution& sol);
	Solution RandomRemoveStation(Solution& sol);
	double bestObjective = 1e9;
	double firstObjective = 1e9;
	int getStationIndexById(int id) const;
	double tspCost(const Solution& sol, const vector<int>& nodes);
	StationScore computeStationScore(const Solution& sol, const INSTANCE::Stations& st);
	vector<int> selectStations(const Solution& sol);
	Solution RandomRemoveDroneNode(Solution& sol);
};

#endif // SOLVER_H