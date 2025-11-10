#include <iostream>
#include <chrono>
#include "parser.h"
#include "solucion.h"
#include "decodificador.h"
#include "evaluador.h"
#include "simulated_annealing.h"
#include <filesystem>
#include <fstream>


// función para exportar datos a csv, recibe el grid, las rutas y la carpeta donde se exportarán los archivos
void exportarCSV(const Grid& grid, const RutaTick& rutasTick, const std::string& carpeta) {
    std::filesystem::create_directory(carpeta); // crea la carpeta si no existe

    // rutas.csv
    std::ofstream rutasFile(carpeta + "/rutas.csv");
    rutasFile << "tick,dron,row,col\n";
    for (size_t d = 0; d < rutasTick.size(); ++d) {
        for (size_t t = 0; t < rutasTick[d].size(); ++t) {
            rutasFile << t << "," << d << "," << rutasTick[d][t].row << "," << rutasTick[d][t].col << "\n";
        }
    }

    // bases.csv
    std::ofstream basesFile(carpeta + "/bases.csv");
    basesFile << "id,row,col\n";
    for (size_t i = 0; i < grid.bases.size(); ++i) {
        basesFile << i << "," << grid.bases[i].posicion.row << "," << grid.bases[i].posicion.col << "\n";
    }

    // obstaculos.csv
    std::ofstream obstFile(carpeta + "/obstaculos.csv");
    obstFile << "row,col\n";
    for (int r = 0; r < grid.rows; ++r) {
        for (int c = 0; c < grid.cols; ++c) {
            if (grid.celdas[r][c].esObstaculo) {
                obstFile << r << "," << c << "\n";
            }
        }
    }

    // urgencias.csv
    std::ofstream urgenciasFile(carpeta + "/urgencias.csv");
    urgenciasFile << "row,col,peso\n";
    for (int r = 0; r < grid.rows; ++r) {
        for (int c = 0; c < grid.cols; ++c) {
            if (grid.celdas[r][c].urgenciaInicial > 0) {
                urgenciasFile << r << "," << c << "," << grid.celdas[r][c].urgenciaInicial << "\n";
            }
        }
    }

    std::cout << "Datos exportados a carpeta '" << carpeta << "'\n";
}



int main(int argc, char* argv[]) {
    if (argc < 5) {
        std::cerr << "Uso: ./PSP-UAV <ruta_instancia.txt> <d_drones> <K_iter> <T_ticks>\n";
        return 1;
    }
    std::string ruta = argv[1];
    int drones = std::stoi(argv[2]); // cantidad drones
    int K = std::stoi(argv[3]);  // iteraciones SA
    int T = std::stoi(argv[4]);  // duración de operación


    // se guarda en grid la grilla leida desde el archivo con la funcion Parser::leerInstancia
    Grid grid = Parser::leerInstancia(ruta);


    std::cout << "### Ejecutando Simulated Annealing con " << drones << " drones, " << K << " iteraciones, " << T << " ticks...\n";
    auto start = std::chrono::steady_clock::now();
    auto mejorSecuencia = SimulatedAnnealing::ejecutar(grid, drones, T, K);     // se ejecuta el algoritmo Simulated Annealing
    auto end = std::chrono::steady_clock::now(); 
    
    double segundos = std::chrono::duration<double>(end - start).count();


    // se evalua la mejor secuencia con la funcion Evaluador::evaluar para entregar resultado final
    int score = Evaluador::evaluar(grid, mejorSecuencia, T);

    std::cout << "\nResultado final:\n";
    std::cout << "Urgencia acumulada: " << score << "\n";
    std::cout << "Ventana de operación T: " << T << "\n";
    std::cout << "Drones Utilizados: " << drones << "\n"; 
    std::cout << "Tiempo de ejecución: " << segundos << " segundos\n";
    std::cout << "Rutas:\n";


    // se utiliza el decodificador para generar las rutas
    auto rutasTick = Decodificador::generarRutaPorTick(grid, mejorSecuencia, T);

    // se imprimen las rutas
    for (int d = 0; d < drones; ++d) {
        std::cout << "D" << d << ": B" << (d % grid.bases.size()) << " ";
        for (const auto& p : rutasTick[d]) {
            std::cout << "(" << p.row << "," << p.col << ")";

            if(p != rutasTick[d].back()) {
                std::cout << " - ";
            }
        }
        
        std::cout << "\n\n";
    }


    // se exportan los datos a csv si flag --export es proporcionada
    bool exportar = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--export") {
            exportar = true;
            break;
        }
    }
    
    if (exportar) {
        exportarCSV(grid, rutasTick, "exported_data");
    }

    
    return 0;
    
}




