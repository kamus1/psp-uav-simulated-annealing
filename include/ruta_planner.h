#pragma once
#include "grid.h"
#include "ruta_types.h"
#include <vector>

class RutaPlanner {
public:
    RutaPlanner(const Grid& grid, int T, int nDrones);
    void generarCompleto(const std::vector<std::vector<Pos>>& secuencias);
    void actualizar(const std::vector<std::vector<Pos>>& secuencias,
                    const std::vector<int>& cambiosDesde);
    const RutaTick& rutas() const { return rutas_; }

private:
    void simularDron(int dron, int objetivoInicio, int tickInicio,
                     const std::vector<std::vector<Pos>>& secuencias);
    int tickInicioDesde(int dron, int objetivoIdx) const;
    void prepararVectores(int dron, int objetivoInicio, size_t nuevoTam);

    const Grid* grid_;
    int T_;
    int nDrones_;
    RutaTick rutas_;
    std::vector<std::vector<int>> inicioObjetivo_;
    std::vector<std::vector<int>> finObjetivo_;
    std::vector<int> tickIdle_;
};
