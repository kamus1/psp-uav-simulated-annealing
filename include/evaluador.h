#pragma once
#include "grid.h"
#include "decodificador.h"

class Evaluador {
public:
    static int evaluar(const Grid& grid, const std::vector<std::vector<Pos>>& secuencias, int T, int penalizacionColision = 100000);
};
