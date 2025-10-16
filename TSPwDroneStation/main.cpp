#include <fstream>
#include <vector>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <regex>
#include "Solver.h"
#include "Param.h"
#include <string>
#include <filesystem>

int main() {
    std::vector<std::string> files = Param::clusterd_filenames;
    std::vector<double> droneSpeeds = { 0.5, 1.0, 2 };

    std::ofstream csv("results.csv");
    csv << "Instance,firstObjective,finalObjective,time(s)\n";

    for (const auto& file : files) {
        // Lấy tên instance (bỏ path + extension)
        std::string filename = file.substr(file.find_last_of("/") + 1);
        std::string instName = filename.substr(0, filename.find("."));

        // Lấy số trucks từ tên file "-kX-"
        std::regex rgx("-k(\\d+)-");
        std::smatch match;
        int trucksFromFile = 0;
        if (std::regex_search(instName, match, rgx)) {
            trucksFromFile = std::stoi(match[1].str());
        }

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
                    double runtime = std::chrono::duration<double>(end - start).count();

                    // Ghi kết quả ra file ngay khi có
                    csv << instName << "_"
                        << numDrones << "_"
                        << numTrucks << "_"
                        << alpha << ","
                        << solver.firstObjective << ","
                        << solver.bestObjective << ","
                        << std::fixed << std::setprecision(2) << runtime << "\n";
                    csv.flush();

                    std::cout << "Done: " << instName
                        << " Drones=" << numDrones
                        << " Trucks=" << numTrucks
                        << " Speed=" << alpha
                        << " FirstObj=" << solver.firstObjective
                        << " FinalObj=" << solver.bestObjective
                        << " Time=" << runtime << "s\n";
                }
            }
        }
        std::cout << std::endl << std::endl;
    }

    csv.close();
    return 0;
}







//int main() {
//    INSTANCE instance;
//    instance.alpha = 0.5;
//    instance.UAVs = 1;
//	instance.num_trucks = 5;
//    instance.loadFromFile("Intances/Clustered/B-n31-k5-r12.mtspds");
//    instance.displayData();
//    Solver solver(instance);
//	std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
//	solver.solve();
//	std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
//	double time_s = std::chrono::duration<double>(end - start).count();
//	std::cout << "Time: " << std::fixed << std::setprecision(2) << time_s << "s" << std::endl;
//    return 0;
//}

//int main() {
//    INSTANCE instance;
//    
//    instance.loadFromFile2("Data/u_min/20-11-11-40-50-2.txt");
//    instance.displayData();
//    Solver solver(instance);
//    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
//    solver.solve();
//    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
//    double time_s = std::chrono::duration<double>(end - start).count();
//    std::cout << "Time: " << std::fixed << std::setprecision(2) << time_s << "s" << std::endl;
//    return 0;
//}

//int main() {
//    std::vector<std::string> u_min_files = { "u_min.txt", "u_min_1.txt", "u_min_2.txt", "u_min_3.txt" };
//    std::ofstream csv("2eche.csv");
//    csv << "filename,firstobj,bestobj,runtime(s)\n";
//
//    for (const auto& umin : u_min_files) {
//        std::ifstream fin("data/u_min/" + umin);
//        std::vector<std::string> filenames;
//        std::string line;
//        while (std::getline(fin, line)) {
//            if (!line.empty()) filenames.push_back(line);
//        }
//        fin.close();
//
//        for (const auto& fname : filenames) {
//            std::string fullpath = "data/u_min/" + fname;
//            INSTANCE instance;
//            if (!instance.loadFromFile2(fullpath)) continue;
//
//            Solver solver(instance);
//            auto start = std::chrono::high_resolution_clock::now();
//            solver.solve();
//            auto end = std::chrono::high_resolution_clock::now();
//            double runtime = std::chrono::duration<double>(end - start).count();
//
//            // Ghi kết quả ra file ngay khi có
//            csv << fname << ","
//                << solver.firstObjective << ","
//                << solver.bestObjective << ","
//                << std::fixed << std::setprecision(2) << runtime << "\n";
//            csv.flush(); // Đảm bảo ghi ra file ngay
//
//            std::cout << "Done: " << fname
//                << " 1stObj=" << solver.firstObjective
//                << " BestObj=" << solver.bestObjective
//                << " Time=" << runtime << "s\n";
//        }
//        std::cout << std::endl << std::endl;
//    }
//    csv.close();
//    return 0;
//}