#pragma once
#include "grid.h"

class SimulatedAnnealing {
public:
    static std::vector<std::vector<Pos>> ejecutar(const Grid& grid, int k, int Tticks, int Kiter);
};
