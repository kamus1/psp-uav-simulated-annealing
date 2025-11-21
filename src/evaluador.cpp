// evaluador: suma urgencia en cada tick, reiniciando cuando se visita
#include "evaluador.h"
#include "time_stats.h"
#include <set>
#include <map>
#include <iostream>
#include <chrono>

namespace {

int evaluarSobreRutas(const Grid& grid, const RutaTick& rutas, int T, int penalizacionColision) {
    int urgenciaTotal = 0;

    std::map<Pos, int> pesos;
    for (const auto& p : grid.urgencias)
        pesos[p] = grid.celdas[p.row][p.col].urgenciaInicial;

    std::map<Pos, int> ticksSinVisita;
    for (const auto& p : grid.urgencias)
        ticksSinVisita[p] = 0;

    for (int t = 0; t < T; ++t) {
        std::set<Pos> visitadasEnEsteTick;

        for (const auto& rutaDron : rutas) {
            if (static_cast<size_t>(t) < rutaDron.size())
                visitadasEnEsteTick.insert(rutaDron[t]);
        }

        for (const auto& [pos, peso] : pesos) {
            ticksSinVisita[pos]++;
            urgenciaTotal += ticksSinVisita[pos] * peso;
            if (visitadasEnEsteTick.count(pos))
                ticksSinVisita[pos] = 0;
        }
    }

    if (Decodificador::hayColisiones(rutas, grid))
        urgenciaTotal += penalizacionColision;

    return urgenciaTotal;
}

} // namespace

int Evaluador::evaluar(const Grid& grid, const std::vector<std::vector<Pos>>& secuencias, int T, int penalizacionColision) {
    auto start = std::chrono::steady_clock::now();
    auto rutas = Decodificador::generarRutaPorTick(grid, secuencias, T);
    int urgenciaTotal = evaluarSobreRutas(grid, rutas, T, penalizacionColision);
    auto end = std::chrono::steady_clock::now();
    TimeStats::addEvaluador(std::chrono::duration<double>(end - start).count());
    return urgenciaTotal;
}

int Evaluador::evaluarConRutas(const Grid& grid, const RutaTick& rutas, int T, int penalizacionColision) {
    auto start = std::chrono::steady_clock::now();
    int urgenciaTotal = evaluarSobreRutas(grid, rutas, T, penalizacionColision);
    auto end = std::chrono::steady_clock::now();
    TimeStats::addEvaluador(std::chrono::duration<double>(end - start).count());
    return urgenciaTotal;
}
