#pragma once
#include "grid.h"
#include <vector>

using SecuenciaEventos = std::vector<std::vector<Pos>>;

class GeneradorSolucion {
public:
    static SecuenciaEventos generarRoundRobin(const Grid& grid, int k);
};
