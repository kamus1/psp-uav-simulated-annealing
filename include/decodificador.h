#pragma once
#include "grid.h"
#include "ruta_types.h"
#include <vector>
#include <map>

class Decodificador {
public:
    static RutaTick generarRutaPorTick(const Grid& grid, const std::vector<std::vector<Pos>>& secuencias, int T);
    static bool hayColisiones(const RutaTick& rutas, const Grid& grid);
    static int contarColisiones(const RutaTick& rutas, const Grid& grid);
};
