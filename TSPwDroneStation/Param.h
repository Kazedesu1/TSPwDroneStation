#pragma once
#include <random>
#include <vector>
#include <chrono>
using namespace std;

class Param {
public:
    static std::mt19937 mt;
    static int seed;
    static void shuffle(std::vector<int>& vec);
    static double real_random_generator(const double& a, const double& b);
    void runFile(const std::string& filename = "Clustered/B-n31-k5-r8.mtspds");
    static std::vector<std::string> clusterd_filenames;
};
