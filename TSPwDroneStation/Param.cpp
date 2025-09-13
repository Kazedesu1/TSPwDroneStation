#include "Param.h"
#include <iostream>
#include "Solver.h"

int Param::seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();

// dynamic seed
std::mt19937 Param::mt = std::mt19937(Param::seed);

void Param::shuffle(std::vector<int>& vec) {
    std::shuffle(vec.begin(), vec.end(), Param::mt);
}

