#pragma once
#include "grid.h"
#include <vector>
#include <map>

using RutaTick = std::vector<std::vector<Pos>>;  // ruta[dron][tick] = Pos

class Decodificador {
public:
    static RutaTick generarRutaPorTick(const Grid& grid, const std::vector<std::vector<Pos>>& secuencias, int T);
    static bool hayColisiones(const RutaTick& rutas, const Grid& grid);
    static int contarColisiones(const RutaTick& rutas, const Grid& grid);
};
