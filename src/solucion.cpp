#include "solucion.h"
#include <algorithm>
// codigo que genera la solución inicial para el algoritmo de Simulated Annealing.

SecuenciaEventos GeneradorSolucion::generarRoundRobin(const Grid& grid, int k) {
    SecuenciaEventos secuencias(k);  // una lista de celdas por cada dron

    std::vector<Pos> urgenciasOrdenadas = grid.urgencias;
    
    // ordena las celdas urgentes de mayor a menor prioridad (urgenciaInicial)
    std::sort(urgenciasOrdenadas.begin(), urgenciasOrdenadas.end(), [&](const Pos& a, const Pos& b) {
        return grid.celdas[a.row][a.col].urgenciaInicial > grid.celdas[b.row][b.col].urgenciaInicial;
    });

    //reparte las celdas urgentes de forma round-robin entre los drones
    for (size_t i = 0; i < urgenciasOrdenadas.size(); ++i) {
        int dronID = i % k;
        secuencias[dronID].push_back(urgenciasOrdenadas[i]);
    }

    return secuencias;
}
