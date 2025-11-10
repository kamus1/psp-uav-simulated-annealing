#include "parser.h"
#include <fstream>
#include <iostream>

Grid Parser::leerInstancia(const std::string& ruta) {
    // se abre el archivo
    std::ifstream file(ruta);
    if (!file.is_open()) {
        std::cerr << "Error al abrir el archivo: " << ruta << std::endl;
        exit(1);
    }

    // se leen dimensiones de la grilla
    std::string label;
    int rows, cols; 
    file >> label >> rows;
    file >> label >> cols;


    Grid grid(rows, cols); // se crea la grilla vacia con las dimensiones, llamando a constructor de Grid (grid.h)
    
    // se leen los obstaculos
    int num_obstaculos;
    file >> label >> num_obstaculos;
    for (int i = 0; i < num_obstaculos; ++i) {
        int r, c; file >> r >> c;
        grid.setObstaculo(r, c);
    }

    // se leen las urgencias
    int num_urgencias;
    file >> label >> num_urgencias;
    for (int i = 0; i < num_urgencias; ++i) {
        int r, c, u; file >> r >> c >> u;
        grid.setUrgencia(r, c, u);
    }

    // se leen las bases
    int num_bases;
    file >> label >> num_bases;
    for (int i = 0; i < num_bases; ++i) {
        int id, r, c; file >> id >> r >> c;
        grid.addBase(id, r, c);
    }

    return grid;
}
