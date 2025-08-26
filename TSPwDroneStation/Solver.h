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
	Solution greedyInsertion();
	bool isStation(int node) const;
};

#endif // SOLVER_H