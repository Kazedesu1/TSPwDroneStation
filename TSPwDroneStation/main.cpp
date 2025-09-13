#include <fstream>
#include <vector>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <map>
#include <limits>
#include "Solver.h"
int main() {
    INSTANCE instance;
    instance.alpha = 1;
    instance.UAVs = 2;
    instance.loadFromFile("Intances/Clustered/B-n31-k5-r8.mtspds");
    instance.displayData();
    Solver solver(instance);
	solver.solve();
    return 0;
}
