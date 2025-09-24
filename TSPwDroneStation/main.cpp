#include <fstream>
#include <vector>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <regex>
#include "Solver.h"

int main() {
    std::vector<std::string> files = {
        "Clustered/M-n121-k7-r8.mtspds",
        "Clustered/M-n121-k7-r12.mtspds",
        "Clustered/M-n121-k7-r16.mtspds"
    };

    std::vector<double> droneSpeeds = { 0.5, 1.0, 1.2 };

    std::ofstream csv("results.csv");
    csv << "Instance,firstObjective,finalObjective,time(ms)\n";

    for (const auto& file : files) {
        // Lấy tên instance (không có path + bỏ .mtspds)
        std::string filename = file.substr(file.find_last_of("/") + 1);
        std::string instName = filename.substr(0, filename.find("."));

        // Parse số trucks mặc định từ tên file: "-kX-"
        std::regex rgx("-k(\\d+)-");
        std::smatch match;
        int trucksFromFile = 0;
        if (std::regex_search(instName, match, rgx)) {
            trucksFromFile = std::stoi(match[1].str());
        }

        // danh sách cấu hình trucks: 2,3,4 + giá trị trong file (nếu khác)
        std::vector<int> truckConfigs = { 2, 3, 4 };
        if (std::find(truckConfigs.begin(), truckConfigs.end(), trucksFromFile) == truckConfigs.end()) {
            truckConfigs.push_back(trucksFromFile);
        }

        for (int numTrucks : truckConfigs) {
            for (int numDrones = 1; numDrones <= 3; ++numDrones) {
                for (double alpha : droneSpeeds) {
                    INSTANCE instance;
                    instance.UAVs = numDrones;
                    instance.alpha = alpha;
                    instance.num_trucks = numTrucks;
                    instance.loadFromFile("Intances/" + file);

                    Solver solver(instance);

                    auto start = std::chrono::high_resolution_clock::now();
                    solver.solve();
                    auto end = std::chrono::high_resolution_clock::now();
                    double time_ms = std::chrono::duration<double, std::milli>(end - start).count();

                    csv << instName
                        << "_" << numDrones
                        << "_" << numTrucks
                        << "_" << alpha
                        << "," << solver.firstObjective   // <-- thêm firstObjective
                        << "," << solver.bestObjective    // <-- final objective
                        << "," << std::fixed << std::setprecision(2) << time_ms
                        << "\n";

                    std::cout << "Done: " << instName
                        << " Drones=" << numDrones
                        << " Trucks=" << numTrucks
                        << " Speed=" << alpha
                        << " FirstObj=" << solver.firstObjective
                        << " FinalObj=" << solver.bestObjective
                        << " Time=" << time_ms << "ms\n";
                }
            }
        }
    }

    csv.close();
    return 0;
}






//int main() {
//    INSTANCE instance;
//    instance.alpha = 1;
//    instance.UAVs = 1;
//    instance.loadFromFile("Intances/Clustered/M-n121-k7-r8.mtspds");
//    instance.displayData();
//    Solver solver(instance);
//	solver.solve();
//    return 0;
//}