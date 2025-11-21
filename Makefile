CXX = g++
CXXFLAGS = -std=c++17 -Wall -O2
SRC = src/main.cpp src/parser.cpp src/grid.cpp src/solucion.cpp src/decodificador.cpp src/evaluador.cpp src/simulated_annealing.cpp src/time_stats.cpp src/a_star.cpp src/ruta_planner.cpp

INC = -Iinclude
TARGET = PSP-UAV

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(INC) $(SRC) -o $(TARGET)

clean:
	rm -f $(TARGET)
