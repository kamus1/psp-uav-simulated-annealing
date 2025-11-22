#include "ruta_planner.h"
#include "a_star.h"
#include "time_stats.h"
#include <algorithm>
#include <chrono>

RutaPlanner::RutaPlanner(const Grid& grid, int T, int nDrones)
    : grid_(&grid), T_(T), nDrones_(nDrones),
      rutas_(nDrones, std::vector<Pos>(T)),
      inicioObjetivo_(nDrones),
      finObjetivo_(nDrones),
      tickIdle_(nDrones, 0) {}

void RutaPlanner::generarCompleto(const std::vector<std::vector<Pos>>& secuencias) {
    for (int d = 0; d < nDrones_; ++d) {
        simularDron(d, 0, 0, secuencias);
    }
}

void RutaPlanner::actualizar(const std::vector<std::vector<Pos>>& secuencias,
                             const std::vector<int>& cambiosDesde) {
    for (int d = 0; d < nDrones_; ++d) {
        if (d >= static_cast<int>(cambiosDesde.size())) break;
        int idx = cambiosDesde[d];
        if (idx < 0) continue;

        // Si el objetivo de inicio de recalculo es desconocido (nuevo frente
        // al historial) o está al inicio, se recalcula completo; de lo
        // contrario se aprovecha el prefijo previo.
        bool inicioDesconocido = (idx <= 0);
        if (!inicioDesconocido) {
            if (d >= static_cast<int>(inicioObjetivo_.size())) {
                inicioDesconocido = true;
            } else {
                const auto& inicios = inicioObjetivo_[d];
                inicioDesconocido = idx >= static_cast<int>(inicios.size()) || inicios[idx] == -1;
            }
        }

        int objetivoInicio = inicioDesconocido ? 0 : idx;
        int tickInicio = inicioDesconocido ? 0 : tickInicioDesde(d, idx);
        if (inicioDesconocido) {
            tickIdle_[d] = 0;
        }

        simularDron(d, objetivoInicio, tickInicio, secuencias);
    }
}

int RutaPlanner::tickInicioDesde(int dron, int objetivoIdx) const {
    if (objetivoIdx <= 0) return 0;
    if (dron >= static_cast<int>(inicioObjetivo_.size())) return 0;
    const auto& inicios = inicioObjetivo_[dron];
    if (objetivoIdx < static_cast<int>(inicios.size()) && inicios[objetivoIdx] != -1) {
        return inicios[objetivoIdx];
    }
    return tickIdle_[dron];
}

void RutaPlanner::prepararVectores(int dron, int objetivoInicio, size_t nuevoTam) {
    auto copiarPrefijo = [&](const std::vector<int>& anterior) {
        std::vector<int> nuevo(nuevoTam, -1);
        int limite = std::min(objetivoInicio, static_cast<int>(nuevoTam));
        for (int i = 0; i < limite && i < static_cast<int>(anterior.size()); ++i) {
            nuevo[i] = anterior[i];
        }
        return nuevo;
    };
    inicioObjetivo_[dron] = copiarPrefijo(inicioObjetivo_[dron]);
    finObjetivo_[dron] = copiarPrefijo(finObjetivo_[dron]);
}

void RutaPlanner::simularDron(int dron, int objetivoInicio, int tickInicio,
                              const std::vector<std::vector<Pos>>& secuencias) {
    if (dron >= nDrones_) return;
    if (tickInicio < 0) tickInicio = 0;
    auto startTiempo = std::chrono::steady_clock::now();

    if (static_cast<int>(rutas_[dron].size()) != T_) {
        rutas_[dron].assign(T_, Pos());
    }

    const auto& seq = secuencias[dron];
    prepararVectores(dron, objetivoInicio, seq.size());

    if (tickInicio == 0) {
        if (!grid_->bases.empty()) {
            rutas_[dron][0] = grid_->bases[dron % grid_->bases.size()].posicion;
        } else {
            rutas_[dron][0] = Pos(0, 0);
        }
    }

    Pos actual = rutas_[dron][tickInicio];
    size_t puntero = static_cast<size_t>(objetivoInicio);
    if (puntero < seq.size()) {
        inicioObjetivo_[dron][puntero] = tickInicio;
    } else {
        tickIdle_[dron] = tickInicio;
    }

    for (int tick = tickInicio + 1; tick < T_; ++tick) {
        if (puntero < seq.size()) {
            Pos destino = seq[puntero];
            auto camino = aStar(*grid_, actual, destino);
            if (camino.size() >= 2) {
                actual = camino[1];
            } else {
                finObjetivo_[dron][puntero] = tick;
                ++puntero;
                if (puntero < seq.size()) {
                    inicioObjetivo_[dron][puntero] = tick;
                } else {
                    tickIdle_[dron] = tick;
                }
            }
        }
        rutas_[dron][tick] = actual;
    }

    if (puntero >= seq.size()) {
        tickIdle_[dron] = std::min(tickIdle_[dron], T_ - 1);
        for (size_t idx = puntero; idx < seq.size(); ++idx) {
            if (idx < inicioObjetivo_[dron].size() && inicioObjetivo_[dron][idx] == -1) {
                inicioObjetivo_[dron][idx] = T_ - 1;
            }
            if (idx < finObjetivo_[dron].size()) {
                finObjetivo_[dron][idx] = T_ - 1;
            }
        }
    }

    auto endTiempo = std::chrono::steady_clock::now();
    TimeStats::addDecodificador(std::chrono::duration<double>(endTiempo - startTiempo).count());
}
