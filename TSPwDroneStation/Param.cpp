#include "Param.h"
#include <iostream>
#include "Solver.h"

int Param::seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
// dynamic seed
std::mt19937 Param::mt = std::mt19937(Param::seed);

void Param::shuffle(std::vector<int>& vec) {
    std::shuffle(vec.begin(), vec.end(), Param::mt);
}

double Param::real_random_generator(const double& a, const double& b) {
    if (b <= a)
        throw std::string("ERROR | double random problem");
    return std::uniform_real_distribution<double>{a, b}(Param::mt);
}