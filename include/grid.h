#pragma once
#include <vector>

// representa una posición en la grilla (fila, columna).
struct Pos {
    int row;
    int col;
    Pos(int r = 0, int c = 0) : row(r), col(c) {} // constructor que inicializa row y col

    // operadores para comparar posiciones
    bool operator==(const Pos& other) const {
        return row == other.row && col == other.col;
    }
    bool operator!=(const Pos& other) const {
        return !(*this == other);
    }
    bool operator<(const Pos& other) const {
        return row < other.row || (row == other.row && col < other.col);
    }
};

// representa una celda en la grilla
struct Celda {
    int urgenciaInicial = 0;
    bool esObstaculo = false;
};

// representa una base
struct Base {
    int id;
    Pos posicion;
};

// se define a la clase Grid
class Grid {
public:
    int rows, cols;
    std::vector<std::vector<Celda>> celdas;
    std::vector<Pos> urgencias;
    std::vector<Base> bases;

    Grid(int r = 0, int c = 0); 

    void setObstaculo(int r, int c);             // establece una celda como obstaculo
    void setUrgencia(int r, int c, int valor);   // establece la urgencia de una celda
    void addBase(int id, int r, int c);          // agrega una base
    bool esValida(int r, int c) const;           // verifica si una celda es valida
    
};


