// evaluador: suma urgencia en cada tick, reiniciando cuando se visita
#include "evaluador.h"
#include <set>
#include <map>
#include <iostream>

int Evaluador::evaluar(const Grid& grid, const std::vector<std::vector<Pos>>& secuencias, int T, int penalizacionColision) {
    auto rutas = Decodificador::generarRutaPorTick(grid, secuencias, T);
    int urgenciaTotal = 0;

    // pesos fijos de cada celda urgente
    std::map<Pos, int> pesos;
    for (const auto& p : grid.urgencias)
        pesos[p] = grid.celdas[p.row][p.col].urgenciaInicial;

    // ticks sin visita (para resetear cuando haya visita)
    std::map<Pos, int> ticksSinVisita;
    for (const auto& p : grid.urgencias)
        ticksSinVisita[p] = 0;

    // recorrer todos los ticks
    for (int t = 0; t < T; ++t) {
        std::set<Pos> visitadasEnEsteTick;

        // registrar celdas visitadas por algún dron
        for (const auto& rutaDron : rutas) {
            if (static_cast<size_t>(t) < rutaDron.size())
                visitadasEnEsteTick.insert(rutaDron[t]);
        }

        // acumular urgencia de cada celda
        for (const auto& [pos, peso] : pesos) {
            ticksSinVisita[pos]++; // pasa un tick más
            urgenciaTotal += ticksSinVisita[pos] * peso; // acumula su nivel actual
            if (visitadasEnEsteTick.count(pos))
                ticksSinVisita[pos] = 0; // reinicia si fue visitada
        }
    }

    // penalizar colisiones
    if (Decodificador::hayColisiones(rutas, grid))
        urgenciaTotal += penalizacionColision;

    return urgenciaTotal;
}
