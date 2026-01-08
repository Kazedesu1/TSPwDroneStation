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
#include <thread>
#include <mutex>
#include <unordered_map>

// void run_with_seed(int seed) {
//     Param::seed = seed;      // set seed cho solver  
//     std::srand(seed);        // C rand() if used elsewhere

//     // seed the thread-local C++ RNG so this thread is deterministic
//     Param::mt.seed(static_cast<std::uint32_t>(seed));

//     std::vector<std::string> files = Param::scatterd_filenames;
//     std::vector<double> droneSpeeds = { 0.5, 1.0, 2 };

//     // CSV cho từng thread
//     std::string csvName = "results_" + std::to_string(seed) + ".csv";
//     std::ofstream csv(csvName);
//     csv << "Instance,firstObjective,finalObjective,time(s),Iterations\n";

//     // Tạo thư mục runlog_x
//     std::string logFolder = "runlog_" + std::to_string(seed);
//     std::filesystem::create_directories(logFolder);

//     for (const auto& file : files) {

//         std::string filename = file.substr(file.find_last_of("/") + 1);
//         std::string instName = filename.substr(0, filename.find("."));

//         std::regex rgx("-k(\\d+)-");
//         std::smatch match;
//         int trucksFromFile = 0;
//         if (std::regex_search(instName, match, rgx))
//             trucksFromFile = std::stoi(match[1].str());

//         std::vector<int> truckConfigs = { 2, 3, 4 };
//         if (trucksFromFile > 4 &&
//             std::find(truckConfigs.begin(), truckConfigs.end(), trucksFromFile) == truckConfigs.end())
//             truckConfigs.push_back(trucksFromFile);

//         for (int numTrucks : truckConfigs) {
//             for (int numDrones = 1; numDrones <= 3; ++numDrones) {
//                 for (double alpha : droneSpeeds) {

//                     INSTANCE instance;
//                     instance.UAVs = numDrones;
//                     instance.alpha = alpha;
//                     instance.num_trucks = numTrucks;
//                     instance.loadFromFile("Intances/" + file);

//                     Solver solver(instance);

//                     solver.maxIter = 1228365;
//                     solver.Delta = 27.3101/100;
//                     solver.epsilon = 9848.123/10000;
//                     solver.nonimproved_threshold = 20442;   
//                     solver.min_rate = 15.4285/100;
//                     solver.max_rate = 55.7523/100;

                    
//                     solver.w = {18 ,5 ,15,1,12 };

//                     auto start = std::chrono::high_resolution_clock::now();
//                     solver.solve();
//                     auto end = std::chrono::high_resolution_clock::now();

//                     double runtime = std::chrono::duration<double>(end - start).count();
//                     int iterations = solver.lastIter;

//                     // CSV xuất kết quả
//                     csv << instName << "_"
//                         << numDrones << "_"
//                         << numTrucks << "_"
//                         << alpha << ","
//                         << solver.firstObjective << ","
//                         << solver.bestObjective << ","
//                         << std::fixed << std::setprecision(2) << runtime << ","
//                         << iterations << "\n";

//                     csv.flush();

//                     // LOG file cho từng seed
//                     {
//                         const Solution& s_best = solver.s_best;

//                         std::ostringstream aoss;
//                         aoss << alpha;
//                         std::string alphaStr = aoss.str();

//                         std::ostringstream fname;
//                         fname << instName << "_" << numDrones << "_" << numTrucks << "_" 
//                               << alphaStr << "_seed" << seed << ".txt";

//                         std::filesystem::path outPath = std::filesystem::path(logFolder) / fname.str();
//                         std::ofstream fout(outPath);

//                         if (fout.is_open()) {
//                             fout << "===== Solution =====\n";
//                             fout << "seed: " << seed << "\n";
//                             const int width = 50;

//                             for (const auto& t : s_best.trucks) {
//                                 std::ostringstream oss;
//                                 oss << "Truck " << t.truck_id << ": ";
//                                 for (int v : t.route) oss << v << " ";
//                                 fout << std::left << std::setw(width) << oss.str()
//                                      << "| Completion = " << t.completion_time << "\n";
//                             }

//                             for (const auto& stationDrones : s_best.drones) {
//                                 for (const auto& d : stationDrones) {
//                                     std::ostringstream oss;
//                                     oss << "Station " << d.station_id
//                                         << ", Drone " << d.drone_id << ": ";
//                                     for (int v : d.customers) oss << v << " ";

//                                     double arrival = 0.0, wait = 0.0;
//                                     if (d.station_id >= 0 && d.station_id < (int)s_best.station_time.size()) {
//                                         arrival = s_best.station_time[d.station_id];
//                                         wait = s_best.wait_time[d.station_id];
//                                     }

//                                     fout << std::left << std::setw(width) << oss.str()
//                                          << "| Completion = " << d.completion_time
//                                          << " | Arrival Time =" << arrival
//                                          << " | Wait Time =" << wait << "\n";
//                                 }
//                             }

//                             fout << "Objective (makespan) = " << s_best.objective << "\n";
//                         }
//                     }

//                     std::cout << "[Seed " << seed << "] Done: "
//                         << instName << " Drones=" << numDrones
//                         << " Trucks=" << numTrucks
//                         << " Speed=" << alpha
//                         << " FirstObj=" << solver.firstObjective
//                         << " FinalObj=" << solver.bestObjective
//                         << " Time=" << runtime << "s  Iter=" << iterations << "\n";
//                 }
//             }
//         }
//     }

//     csv.close();
// }

// int main() {
//     std::vector<std::thread> threads;

//     for (int seed = 1; seed <= 10; seed++)
//         threads.emplace_back(run_with_seed, seed);

//     for (auto &t : threads) t.join();

//     std::cout << "\n=== ALL THREADS DONE ===\n";
//     return 0;
// }



// int main() {
//    INSTANCE instance;
//    instance.alpha = 1;
//    instance.UAVs = 2;
// 	instance.num_trucks = 7;
//    instance.loadFromFile("Intances/Scattered/A-n46-k7-r16.mtspds");
//    instance.displayData();
//    Solver solver(instance);
// 	std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
// 	solver.solve();
// 	std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
// 	double time_s = std::chrono::duration<double>(end - start).count();
// 	std::cout << "Time: " << std::fixed << std::setprecision(2) << time_s << "s" << std::endl;
//    return 0;
// }


// std::unordered_map<std::string, std::string> parseArgs(int argc, char* argv[]) {
//     std::unordered_map<std::string, std::string> args;
//     for (int i = 1; i + 1 < argc; i += 2) {
//         args[argv[i]] = argv[i + 1];
//     }
//     return args;
// }

// int main(int argc, char* argv[]) {

//     auto args = parseArgs(argc, argv);

//     std::string instanceDir  = args["-instanceDir"];
//     std::string instanceName = args["-instanceName"];

//     Param::seed = 12345;
//     INSTANCE instance;


//     instance.loadFromFile3(instanceDir + "/" + instanceName);

//     Solver solver(instance);

//     solver.maxIter = std::stoi(args["-maxIter"]);
//     solver.Delta = std::stod(args["-Delta"])/100;
//     solver.epsilon = std::stod(args["-epsilon"])/10000;
//     solver.nonimproved_threshold =
//     std::stoi(args["-nonimproved_threshold"]);   
//     solver.min_rate = std::stod(args["-min_rate"])/100;
//     solver.max_rate = std::stod(args["-max_rate"])/100;

    
//     solver.w = {
//         std::stoi(args["-w1"]),
//         std::stoi(args["-w2"]),
//         std::stoi(args["-w3"]),
//         std::stoi(args["-w4"]),
//         std::stoi(args["-w5"])
//     };


//     solver.solve();

//     std::cout << solver.bestObjective << std::endl;
// }


// int main() {
//    INSTANCE instance;
   
//    instance.loadFromFile2("Data/u_min/20-11-11-40-50-2.txt");
//    instance.displayData();
//    Solver solver(instance);
//    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
//    solver.solve();
//    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
//    double time_s = std::chrono::duration<double>(end - start).count();
//    std::cout << "Time: " << std::fixed << std::setprecision(2) << time_s << "s" << std::endl;
//    return 0;
// }

// int main() {
//     namespace fs = std::filesystem;

//     std::vector<std::string> u_min_files = { "u_min" ,"u_min_1" , "u_min_2" , "u_min_3" , };
//     fs::path outputPath = "2eche.csv";

//     // --- Kiểm tra xem file kết quả đã tồn tại chưa ---
//     bool newFile = !fs::exists(outputPath);

//     // --- Mở file CSV ở chế độ ghi nối tiếp (append) ---
//     std::ofstream csv(outputPath, std::ios::out | std::ios::app);
//     if (!csv.is_open()) {
//         std::cerr << "❌ Không thể mở file kết quả: " << outputPath << std::endl;
//         return 1;
//     }

//     // --- Nếu là file mới, thêm header ---
//     if (newFile) {
//         csv << "filename,firstobj,bestobj,runtime(s)\n";
//     }

//     for (const auto& umin : u_min_files) {
//         std::ifstream fin("Data/"+ umin + "/" + umin +".txt");

//         if (!fin.is_open()) {
//             std::cerr << "❌ Không thể mở file danh sách: " << umin << std::endl;
//             continue;
//         }

//         std::vector<std::string> filenames;
//         std::string line;
//         while (std::getline(fin, line)) {
//             // Xóa ký tự \r nếu file tạo từ Windows
//             line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
//             if (!line.empty()) filenames.push_back(line);
//         }
//         fin.close();

//         for (const auto& fname : filenames) {
//             std::string fullpath = "Data/" + umin + "/" + fname;

//             INSTANCE instance;
//             if (!instance.loadFromFile2(fullpath)) {
//                 std::cerr << "⚠️ Bỏ qua file: " << fname << " (load thất bại)\n";
//                 continue;
//             }

//             Solver solver(instance);
//             auto start = std::chrono::high_resolution_clock::now();
//             solver.solve();
//             auto end = std::chrono::high_resolution_clock::now();
//             double runtime = std::chrono::duration<double>(end - start).count();

//             // --- Ghi kết quả ra file ---
//             csv << fname << ","
//                 << solver.firstObjective << ","
//                 << solver.bestObjective << ","
//                 << std::fixed << std::setprecision(2) << runtime << "\n";
//             csv.flush();

//             std::cout << "✅ Done: " << fname
//                       << " | 1stObj=" << solver.firstObjective
//                       << " | BestObj=" << solver.bestObjective
//                       << " | Time=" << runtime << "s\n";
//         }

//         std::cout << "\n";
//     }

//     csv.close();
//     std::cout << "📁 CSV saved to: " << fs::absolute(outputPath) << std::endl;

//     return 0;
// }

int main() {
   INSTANCE instance;
   instance.loadFromFile4("Instance sets/Murray_10/20140813T111613/nodes.csv");
   instance.displayData();
   Solver solver(instance);
   std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
   solver.solve();
   std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
   double time_s = std::chrono::duration<double>(end - start).count();
   std::cout << "Time: " << std::fixed << std::setprecision(2) << time_s << "s" << std::endl;
   return 0;
}
