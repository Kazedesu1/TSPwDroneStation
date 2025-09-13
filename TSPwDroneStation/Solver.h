#pragma once
#ifndef SOLVER_H
#define SOLVER_H

#include <vector>
#include <set>
#include "Instances.h"
#include "Solution.h"
using namespace std;
class Solver {
public:
	Solver(const INSTANCE& instance);
	void solve();
	const INSTANCE& instance;
	Solution greedyInsertion(Solution sol);
	bool isStation(int node) const;
	Solution Ruin(Solution &sol);
	Solution removeStation(Solution& sol, int stationNode);
	void removeUnusedStations(Solution& sol); 
	Solution remove(Solution& sol); 
	void removeCustomerFromTruck(Solution& sol, int truckId, int customer);
};

#endif // SOLVER_H