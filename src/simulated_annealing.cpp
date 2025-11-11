#include "simulated_annealing.h"
#include "solucion.h"
#include "evaluador.h"
#include "decodificador.h"
#include <random>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>
#include <map>

static std::mt19937 rng(std::random_device{}()); // generador de números aleatorios Mersenne twister

//----------------- parámetros globales del algoritmo -----------------//
namespace {
    const double SA_TEMPERATURA_INICIAL = 1000.0;
    const double SA_FACTOR_ENFRIAMIENTO = 0.995;
    const double SA_SHIFT_BASE_WEIGHT = 5.0;
    const double SA_SHIFT_PROB_MAX = 0.65;
    const double SA_RESOLUCION_BASE = 2.0;     //prob movimiento resolución de colisiones
    const double SA_RESOLUCION_PROB_MAX = 0.5;
}

/* Función principal del algoritmo: ejecuta Simulated Annealing.

Parámetros:
    - grid: referencia constante a la grilla leída desde el archivo de instancia.
    - nDrones: cantidad de drones en operación.
    - Tticks: duración total de la simulación (ticks de tiempo).
    - Kiter: número máximo de iteraciones del algoritmo SA.

 */
std::vector<std::vector<Pos>> SimulatedAnnealing::ejecutar(const Grid &grid, int nDrones, int Tticks, int Kiter) {
    // generar solucion inicial
    auto actual = GeneradorSolucion::generarRoundRobin(grid, nDrones); 
    int scoreActual = Evaluador::evaluar(grid, actual, Tticks);
    auto mejor = actual;
    int mejorScore = scoreActual;

    // variables de la temperatura controladas por las constantes globales
    double T = SA_TEMPERATURA_INICIAL; 

    // struct para describir una colisión: en que tick, en que celda y que drones.
    struct ColisionDetalle {
        int tick;
        Pos pos;
        std::vector<int> drones;
    };

    std::vector<Pos> urgenciasOrdenadas = grid.urgencias; // copia las posiciones urgentes

    // ordena las urgencias por su valor inicial
    std::sort(urgenciasOrdenadas.begin(), urgenciasOrdenadas.end(), [&](const Pos& a, const Pos& b) {
        return grid.celdas[a.row][a.col].urgenciaInicial > grid.celdas[b.row][b.col].urgenciaInicial;
    });

    //----------------- funciones auxiliares, con lambdas --------------------//
    // distancia Manhattan entre dos posiciones
    auto distancia = [](const Pos& a, const Pos& b) {
        return std::abs(a.row - b.row) + std::abs(a.col - b.col);
    };

    // peso de una urgencia
    auto pesoUrgencia = [&](const Pos& p) {
        return grid.celdas[p.row][p.col].urgenciaInicial;
    };

    // posición de la base de un dron
    auto baseDeDron = [&](int d) {
        if (grid.bases.empty()) return Pos(0, 0);
        return grid.bases[d % grid.bases.size()].posicion;
    };

    // insertar una celda en la ruta de manera heurística
    auto insertarHeuristico = [&](std::vector<std::vector<Pos>>& rutas, int destino, const Pos& celda) {
        size_t mejorPos = 0;
        long long mejorScore = std::numeric_limits<long long>::max();
        Pos baseDestino = baseDeDron(destino);
        // se recorren todas las posiciones posibles dentro de la ruta
        // y se calcula un costo artificial que mezcla distancia y prioridad
        for (size_t pos = 0; pos <= rutas[destino].size(); ++pos) {
            long long score = 0;
            const Pos& prev = (pos == 0) ? baseDestino : rutas[destino][pos - 1];
            score += distancia(prev, celda);
            if (pos < rutas[destino].size()) {
                score += distancia(celda, rutas[destino][pos]);
            }
            // penaliza romper el orden por urgencia para priorizar celdas críticas
            if (pos > 0) {
                int wPrev = pesoUrgencia(rutas[destino][pos - 1]);
                int wCelda = pesoUrgencia(celda);
                if (wCelda > wPrev) {
                    score += (wCelda - wPrev);
                }
            }
            if (pos < rutas[destino].size()) {
                int wNext = pesoUrgencia(rutas[destino][pos]);
                int wCelda = pesoUrgencia(celda);
                if (wNext > wCelda) {
                    score += (wNext - wCelda);
                }
            }

            if (score < mejorScore) {
                mejorScore = score;
                mejorPos = pos;
            }
        }
        rutas[destino].insert(rutas[destino].begin() + mejorPos, celda);
    };

    
    // detectar colisiones en las rutas
    auto detectarColisiones = [&](const RutaTick& rutas) {
        std::vector<ColisionDetalle> detalles;
        if (rutas.empty()) return detalles;
        auto esBasePos = [&](const Pos& p) {
            for (const auto& base : grid.bases) {
                if (base.posicion == p) return true;
            }
            return false;
        };
        int ticks = rutas[0].size();
        for (int t = 0; t < ticks; ++t) {
            std::map<Pos, std::vector<int>> ocupacion;
            for (int d = 0; d < nDrones; ++d) {
                if (static_cast<size_t>(t) >= rutas[d].size()) continue;
                Pos pos = rutas[d][t];
                if (t == 0 && esBasePos(pos)) continue; // múltiples drones pueden partir de la misma base
                ocupacion[pos].push_back(d);
            }
            // toda celda ocupada por más de un dron en el mismo tick es una colisión candidata
            for (auto& entry : ocupacion) {
                if (entry.second.size() > 1) {
                    detalles.push_back({t, entry.first, entry.second});
                }
            }
        }
        return detalles;
    };


    //----------------- bucle principal del algoritmo -----------------
    for (int iter = 0; iter < Kiter; ++iter)
    {
        auto vecino = actual;

        // reconstruye la ruta temporal completa para detectar colisiones reales por tick


        /**/
        
        // NOTA: lo mas eficiente sería solamente recalcular los segmentos de ruta que realmente cambiaron
        static RutaTick rutasActuales;
        static std::vector<ColisionDetalle> detallesColision;

        //if (iter == 0 || iter % 10 == 0) {
        rutasActuales = Decodificador::generarRutaPorTick(grid, actual, Tticks);
        detallesColision = detectarColisiones(rutasActuales);
        //}
            
        
        //auto rutasActuales = Decodificador::generarRutaPorTick(grid, actual, Tticks);
        //auto detallesColision = detectarColisiones(rutasActuales);

        int colisionesActuales = 0;
        for (const auto& detalle : detallesColision) {
            colisionesActuales += static_cast<int>(detalle.drones.size()) - 1;
        }
        double shiftProb = 0.0;
        // cuanto más choques haya, mayor la probabilidad de aplicar un corrimiento masivo (shift)
        if (colisionesActuales > 0) {
            double rawProb = static_cast<double>(colisionesActuales) /
                             (static_cast<double>(colisionesActuales) + SA_SHIFT_BASE_WEIGHT);
            shiftProb = std::min(SA_SHIFT_PROB_MAX, rawProb);
        }
        bool aplicarShift = std::uniform_real_distribution<>(0.0, 1.0)(rng) < shiftProb;
        double probResolucion = 0.0;
        // probabilidad de ejecutar un operador específico para resolver colisiones detectadas
        if (colisionesActuales > 0) {
            double rawRes = static_cast<double>(colisionesActuales) /
                            (static_cast<double>(colisionesActuales) + SA_RESOLUCION_BASE);
            probResolucion = std::min(SA_RESOLUCION_PROB_MAX, rawRes);
        }

        if (aplicarShift) {
            int d = rng() % nDrones;
            if (vecino[d].size() <= 1)
                continue;
            std::rotate(vecino[d].begin(), vecino[d].begin() + 1, vecino[d].end());
        } else {
            bool aplicarResolucion = !detallesColision.empty() &&
                std::uniform_real_distribution<>(0.0, 1.0)(rng) < probResolucion;
            if (aplicarResolucion) {
                // se selecciona una colisión y se redistribuye una celda entre los drones involucrados
                const auto& detalle = detallesColision[rng() % detallesColision.size()];
                if (detalle.drones.size() < 2)
                    continue;
                size_t idxA = rng() % detalle.drones.size();
                size_t idxB = rng() % detalle.drones.size();
                if (idxA == idxB) {
                    idxB = (idxB + 1) % detalle.drones.size();
                }
                int dronA = detalle.drones[idxA];
                int dronB = detalle.drones[idxB];
                int donante = vecino[dronA].size() >= vecino[dronB].size() ? dronA : dronB;
                int receptor = (donante == dronA) ? dronB : dronA;
                if (vecino[donante].empty())
                    continue;
                int idx = rng() % vecino[donante].size();
                Pos celda = vecino[donante][idx];
                vecino[donante].erase(vecino[donante].begin() + idx);
                insertarHeuristico(vecino, receptor, celda);
            } else {
                int movimiento = rng() % 5;

                if (movimiento == 0) {
                    // swap interno: cambia el orden de dos visitas en la misma ruta
                    int d = rng() % nDrones;
                    if (vecino[d].size() < 2)
                        continue;
                    int i = rng() % vecino[d].size();
                    int j = rng() % vecino[d].size();
                    if (i == j)
                        continue;
                    std::swap(vecino[d][i], vecino[d][j]);
                } else if (movimiento == 1) {
                    // mover una celda de un dron a otro preservando la heurística de inserción
                    int origen = rng() % nDrones;
                    if (vecino[origen].empty())
                        continue;
                    int destino = rng() % nDrones;
                    int idx = rng() % vecino[origen].size();
                    Pos celda = vecino[origen][idx];
                    vecino[origen].erase(vecino[origen].begin() + idx);
                    insertarHeuristico(vecino, destino, celda);
                } else if (movimiento == 2) {
                    // intercambio directo de visitas entre dos drones
                    int origen = rng() % nDrones;
                    if (vecino[origen].empty())
                        continue;
                    int destino = rng() % nDrones;
                    if (vecino[destino].empty())
                        continue;
                    int idxOrigen = rng() % vecino[origen].size();
                    int idxDestino = rng() % vecino[destino].size();
                    std::swap(vecino[origen][idxOrigen], vecino[destino][idxDestino]);
                } else if (movimiento == 3) {
                    // inversión de un subcamino, útil para alterar trayectos largos
                    int d = rng() % nDrones;
                    if (vecino[d].size() < 3)
                        continue;
                    int i = rng() % (vecino[d].size() - 1);
                    int j = rng() % (vecino[d].size() - i - 1) + i + 1;
                    if (j - i >= 1) {
                        std::reverse(vecino[d].begin() + i, vecino[d].begin() + j + 1);
                    } else {
                        continue;
                    }
                } else {
                    // reemplazo la visita menos urgente por otra de mayor peso global
                    if (urgenciasOrdenadas.empty())
                        continue;
                    int d = rng() % nDrones;
                    if (vecino[d].empty())
                        continue;
                    int idxMin = 0;
                    int pesoMin = pesoUrgencia(vecino[d][0]);
                    for (size_t idx = 1; idx < vecino[d].size(); ++idx) {
                        int pesoActual = pesoUrgencia(vecino[d][idx]);
                        if (pesoActual < pesoMin) {
                            pesoMin = pesoActual;
                            idxMin = static_cast<int>(idx);
                        }
                    }
                    bool reemplazo = false;
                    for (const auto& urg : urgenciasOrdenadas) {
                        int pesoNuevo = pesoUrgencia(urg);
                        if (pesoNuevo > pesoMin) {
                            vecino[d][idxMin] = urg;
                            reemplazo = true;
                            break;
                        }
                    }
                    if (!reemplazo) continue;
                }
            }
        }

        int scoreVecino = Evaluador::evaluar(grid, vecino, Tticks);
        int delta = scoreVecino - scoreActual;

        // aceptar la solución vecina si es mejor o con cierta probabilidad, según la temperatura (SA)
        if (delta < 0 || std::exp(-delta / T) > std::uniform_real_distribution<>(0.0, 1.0)(rng)){
            actual = vecino;
            scoreActual = scoreVecino;

            if (scoreVecino < mejorScore){
                mejor = vecino;
                mejorScore = scoreVecino;

                // si es mejor solucion se generan rutas por tick y se cuentan colisiones
                auto rutasTick = Decodificador::generarRutaPorTick(grid, mejor, Tticks);
                int colisiones = Decodificador::contarColisiones(rutasTick, grid);

                std::cout << "[+] Nueva mejor solución: " << mejorScore << " | Colisiones: " << colisiones << "\n";
            }
        }

        T *= SA_FACTOR_ENFRIAMIENTO; // se reduce la temperatura por factor alpha
    }

    return mejor;
}
