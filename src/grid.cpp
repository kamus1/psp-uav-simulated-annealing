#include "grid.h"


// constructor de Grid, crea una matriz de r vectores de c celdas (Celda tiene urgencia y esObstaculo)
Grid::Grid(int r, int c)
    : rows(r), cols(c), celdas(r, std::vector<Celda>(c)) {}

// establece una celda como obstaculo
void Grid::setObstaculo(int r, int c) {
    if (esValida(r, c)) celdas[r][c].esObstaculo = true;
}

// establece la urgencia de una celda
void Grid::setUrgencia(int r, int c, int valor) {
    if (esValida(r, c)) {
        celdas[r][c].urgenciaInicial = valor;
        urgencias.emplace_back(r, c);
    }
}


// agrega una base
void Grid::addBase(int id, int r, int c) {
    if (esValida(r, c)) {
        bases.push_back({id, {r, c}});
    }
}

// verifica si una celda es valida
bool Grid::esValida(int r, int c) const {
    return r >= 0 && r < rows && c >= 0 && c < cols;
}
