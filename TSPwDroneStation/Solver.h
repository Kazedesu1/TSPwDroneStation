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
	void removeStation(Solution& sol, int stationNode);
	void removeUnusedStations(Solution& sol);
	Solution RandomRemoval(Solution& sol, double rate);
	void removeCustomerFromTruck(Solution& sol, int truckId, int customer);
	Solution WorstRemoval(Solution& sol, double rate);
	Solution RandomRemoveStation(Solution& sol, double rate);
	double bestObjective = 1e9;
	double firstObjective = 1e9;
	int getStationIndexById(int id) const;
	double tspCost(const Solution& sol, const vector<int>& nodes);
	StationScore computeStationScore(const Solution& sol, const INSTANCE::Stations& st);
	vector<int> selectStations(const Solution& sol);
	Solution RandomRemoveDroneNode(Solution& sol, double rate);
	int lastIter = 0;
	void select_insertion_criterion(vector<int>& customerQueue, const vector<int>& w, const Solution& sol);
	vector<int> w = { 5,1,1,2,2 };
	Solution s_best;
	int maxIter ;     
    double Delta ;     // (1 + Δ) là ngưỡng chấp nhận
    double epsilon ;  // hệ số giảm Δ sau mỗi vòng
    int nonimproved_threshold ; // số vòng không cải thiện liên tiếp thì dừng
    int nonimproved_count ;
    double min_rate ;
    double max_rate ;
};

#endif // SOLVER_H
