#include "Param.h"
#include <iostream>
#include "Solver.h"

int Param::seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
// dynamic seed
std::mt19937 Param::mt = std::mt19937(Param::seed);

std::vector<std::string> Param::clusterd_filenames = {
        "Clustered/B-n31-k5-r8.mtspds",
        "Clustered/B-n31-k5-r12.mtspds",
        "Clustered/B-n31-k5-r16.mtspds",
        "Clustered/B-n34-k5-r8.mtspds",
        "Clustered/B-n34-k5-r12.mtspds",
        "Clustered/B-n34-k5-r16.mtspds",
        "Clustered/B-n35-k5-r8.mtspds",
        "Clustered/B-n35-k5-r12.mtspds",
        "Clustered/B-n35-k5-r16.mtspds",
        "Clustered/B-n38-k6-r8.mtspds",
        "Clustered/B-n38-k6-r12.mtspds",
        "Clustered/B-n38-k6-r16.mtspds",
        "Clustered/B-n39-k5-r8.mtspds",
        "Clustered/B-n39-k5-r12.mtspds",
        "Clustered/B-n39-k5-r16.mtspds",
        "Clustered/B-n41-k6-r8.mtspds",
        "Clustered/B-n41-k6-r12.mtspds",
        "Clustered/B-n41-k6-r16.mtspds",
        "Clustered/B-n43-k6-r8.mtspds",
        "Clustered/B-n43-k6-r12.mtspds",
        "Clustered/B-n43-k6-r16.mtspds",
        "Clustered/B-n44-k7-r8.mtspds",
        "Clustered/B-n44-k7-r12.mtspds",
        "Clustered/B-n44-k7-r16.mtspds",
        "Clustered/B-n45-k5-r8.mtspds",
        "Clustered/B-n45-k5-r12.mtspds",
        "Clustered/B-n45-k5-r16.mtspds",
        "Clustered/B-n45-k6-r8.mtspds",
        "Clustered/B-n45-k6-r12.mtspds",
        "Clustered/B-n45-k6-r16.mtspds",
        "Clustered/B-n50-k7-r8.mtspds",
        "Clustered/B-n50-k7-r12.mtspds",
        "Clustered/B-n50-k7-r16.mtspds",
        "Clustered/B-n50-k8-r8.mtspds",
        "Clustered/B-n50-k8-r12.mtspds",
        "Clustered/B-n50-k8-r16.mtspds",
        "Clustered/B-n51-k7-r8.mtspds",
        "Clustered/B-n51-k7-r12.mtspds",
        "Clustered/B-n51-k7-r16.mtspds",
        "Clustered/B-n52-k7-r8.mtspds",
        "Clustered/B-n52-k7-r12.mtspds",
        "Clustered/B-n52-k7-r16.mtspds",
        "Clustered/B-n56-k7-r8.mtspds",
        "Clustered/B-n56-k7-r12.mtspds",
        "Clustered/B-n56-k7-r16.mtspds",
        "Clustered/B-n57-k7-r8.mtspds",
        "Clustered/B-n57-k7-r12.mtspds",
        "Clustered/B-n57-k7-r16.mtspds",
        "Clustered/B-n57-k9-r8.mtspds",
        "Clustered/B-n57-k9-r12.mtspds",
        "Clustered/B-n57-k9-r16.mtspds",
        "Clustered/B-n63-k10-r8.mtspds",
        "Clustered/B-n63-k10-r12.mtspds",
        "Clustered/B-n63-k10-r16.mtspds",
        "Clustered/B-n64-k9-r8.mtspds",
        "Clustered/B-n64-k9-r12.mtspds",
        "Clustered/B-n64-k9-r16.mtspds",
        "Clustered/B-n66-k9-r8.mtspds",
        "Clustered/B-n66-k9-r12.mtspds",
        "Clustered/B-n66-k9-r16.mtspds",
        "Clustered/B-n67-k10-r8.mtspds",
        "Clustered/B-n67-k10-r12.mtspds",
        "Clustered/B-n67-k10-r16.mtspds",
        "Clustered/B-n68-k9-r8.mtspds",
        "Clustered/B-n68-k9-r12.mtspds",
        "Clustered/B-n68-k9-r16.mtspds",
        "Clustered/B-n78-k10-r8.mtspds",
        "Clustered/B-n78-k10-r12.mtspds",
        "Clustered/B-n78-k10-r16.mtspds",
        "Clustered/M-n101-k10-r8.mtspds",
        "Clustered/M-n101-k10-r12.mtspds",
        "Clustered/M-n101-k10-r16.mtspds",
        "Clustered/M-n121-k7-r8.mtspds",
        "Clustered/M-n121-k7-r12.mtspds",
        "Clustered/M-n121-k7-r16.mtspds"
};

void Param::shuffle(std::vector<int>& vec) {
    std::shuffle(vec.begin(), vec.end(), Param::mt);
}

double Param::real_random_generator(const double& a, const double& b) {
    if (b <= a)
        throw std::string("ERROR | double random problem");
    return std::uniform_real_distribution<double>{a, b}(Param::mt);
}