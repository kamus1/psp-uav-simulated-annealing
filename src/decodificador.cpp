#include "decodificador.h"
#include <queue>
#include <set>
#include <map>
#include <cmath>
#include <iostream>
#include <algorithm>

// este codigo calcula los caminos reales con A* y detecta colisiones.

namespace {

// coste para moverse a vecino (siempre 1, es usado por A*)
int costo(const Pos&, const Pos&) {
    return 1;
}

// Heurística de A* (distancia Manhattan)
int heuristica(const Pos& a, const Pos& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

// obtener vecinos validos (8 direcciones + quieto)
//calcula movimientos posibles desde p
std::vector<Pos> vecinos(const Grid& grid, const Pos& p) {
    std::vector<Pos> res; // almacena vecinos validos

    // recorre delta de fila/columna en {-1,0,1}
    for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
            int nr = p.row + dr;
            int nc = p.col + dc;
            if (dr == 0 && dc == 0) continue;
            if (grid.esValida(nr, nc) && !grid.celdas[nr][nc].esObstaculo) {
                res.emplace_back(nr, nc);
            }
        }
    }
    res.push_back(p);  //opcion de quedarse quieto
    return res;
}

// A* para encontrar camino desde origen hasta destino
std::vector<Pos> aStar(const Grid& grid, const Pos& start, const Pos& goal) {
    std::priority_queue<std::pair<int, Pos>, std::vector<std::pair<int, Pos>>, std::greater<>> frontera; // min-heap por prioridad
    frontera.emplace(0, start);

    std::map<Pos, Pos> cameFrom;
    std::map<Pos, int> costoActual;
    cameFrom[start] = start;
    costoActual[start] = 0;

    while (!frontera.empty()) {
        Pos actual = frontera.top().second;
        frontera.pop();

        if (actual == goal) break;

        for (auto& next : vecinos(grid, actual)) {
            int nuevoCosto = costoActual[actual] + costo(actual, next);
            if (!costoActual.count(next) || nuevoCosto < costoActual[next]) {
                costoActual[next] = nuevoCosto;
                int prioridad = nuevoCosto + heuristica(next, goal);
                frontera.emplace(prioridad, next);
                cameFrom[next] = actual;
            }
        }
    }

    std::vector<Pos> path;
    if (!cameFrom.count(goal)) return path;

    for (Pos p = goal; p != start; p = cameFrom[p])
        path.push_back(p);
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace


/* generación de rutas por tick (simulación con A*)
    input: secuencias de visitas objetivo (por dron).
    output: RutaTick: posición de cada dron en cada tick.
*/
RutaTick Decodificador::generarRutaPorTick(const Grid& grid, const std::vector<std::vector<Pos>>& secuencias, int T) {
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

    return rutas;
}


// detecta si hay colisiones de drones en las rutas
bool Decodificador::hayColisiones(const RutaTick& rutas, const Grid& grid) {
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

    return hayConflictos;
}


// conteo de colisiones
int Decodificador::contarColisiones(const RutaTick& rutas, const Grid& grid) {
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
    return colisiones;
}
