#include <fstream>
#include <vector>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <map>
#include <limits>
#include "Solver.h"

int main() {
    INSTANCE ins;
    ins.E = 24;
    ins.alpha = 1;
    ins.activeStations = 4;
    ins.UAVs = 2;
    ins.numtrucks = 3;

    ins.loadFromFile("Intances/Scattered/A-n34-k5-r8.mtspds");
	ins.displayData();
    Solver solver(ins);
	solver.solve();
    return 0;
}
