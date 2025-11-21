#include "decodificador.h"
#include "time_stats.h"
#include "a_star.h"
#include <set>
#include <map>
#include <iostream>
#include <algorithm>
#include <chrono>


/* generación de rutas por tick (simulación con A*)
    input: secuencias de visitas objetivo (por dron).
    output: RutaTick: posición de cada dron en cada tick.
*/
RutaTick Decodificador::generarRutaPorTick(const Grid& grid, const std::vector<std::vector<Pos>>& secuencias, int T) {
    auto start = std::chrono::steady_clock::now();
    int nDrones = secuencias.size();
    RutaTick rutas(nDrones);
    std::vector<size_t> puntero(nDrones, 0); // índice del próximo objetivo en la secuencia del dron
    std::vector<Pos> actual(nDrones);     // posición actual de cada dron

    // inicializar desde base
    for (int d = 0; d < nDrones; ++d) {
        actual[d] = grid.bases[d % grid.bases.size()].posicion; // inicializa cada dron en su base (usa módulo por si hay más drones que bases)
        rutas[d].push_back(actual[d]);
    }

    // simula del tick 1 al T-1.
    for (int tick = 1; tick < T; ++tick) {
        for (int d = 0; d < nDrones; ++d) {

            //para cada dron
            if (puntero[d] < secuencias[d].size()) { // si aun tiene objetivos pendientes
                Pos destino = secuencias[d][puntero[d]]; // destino = prox celda de la secuencia
                auto camino = aStar(grid, actual[d], destino);
                if (camino.size() >= 2) {
                    actual[d] = camino[1]; // avanzar un paso
                } else {
                    // ya está en destino
                    puntero[d]++;
                }
            }
            // quedarse donde está si no hay más eventos
            rutas[d].push_back(actual[d]);
        }
    }

    auto end = std::chrono::steady_clock::now();
    TimeStats::addDecodificador(std::chrono::duration<double>(end - start).count());
    return rutas;
}


// detecta si hay colisiones de drones en las rutas
bool Decodificador::hayColisiones(const RutaTick& rutas, const Grid& grid) {
    auto start = std::chrono::steady_clock::now();
    int T = rutas[0].size(); //ticks simulados en cada ruta.
    int nDrones = rutas.size();
    bool hayConflictos = false;

    // iterar por tick y por dron
    for (int t = 0; t < T; ++t) {
        std::set<Pos> visitadas; //set de posiciones ocupadas en este tick.

        for (int d = 0; d < nDrones; ++d) {
            Pos pos = rutas[d][t];

            // permitir múltiples drones en la misma base en tick 0
            if (t == 0) {
                bool esBase = false;
                for (const auto& b : grid.bases) {
                    if (pos == b.posicion) {
                        esBase = true;
                        break;
                    }
                }
                if (esBase) continue;  // no se considera colisión
            }

            if (!visitadas.insert(pos).second) {
                hayConflictos = true;  // marcar conflicto pero seguir
            }
        }
    }

    auto end = std::chrono::steady_clock::now();
    TimeStats::addDecodificador(std::chrono::duration<double>(end - start).count());
    return hayConflictos;
}


// conteo de colisiones
int Decodificador::contarColisiones(const RutaTick& rutas, const Grid& grid) {
    auto start = std::chrono::steady_clock::now();
    int T = rutas[0].size();
    int nDrones = rutas.size();
    int colisiones = 0;

    // para cada tick contador[pos] dirá cuántos drones están en esa celda en este tick.
    for (int t = 0; t < T; ++t) {
        std::map<Pos, int> contador;
        for (int d = 0; d < nDrones; ++d) {
            Pos pos = rutas[d][t];

            //permitir múltiples drones en la misma base solo en tick 0
            if (t == 0) {
                bool esBase = false;
                for (const auto& b : grid.bases) {
                    if (pos == b.posicion) {
                        esBase = true;
                        break;
                    }
                }
                if (esBase) continue;  // pasa al siguiente dron
            }

            //si hay 3 drones en la misma celda se cuenta como 2 colisiones
            if (++contador[pos] > 1) ++colisiones;
        }
    }
    auto end = std::chrono::steady_clock::now();
    TimeStats::addDecodificador(std::chrono::duration<double>(end - start).count());
    return colisiones;
}
