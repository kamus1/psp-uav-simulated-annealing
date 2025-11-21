#pragma once

namespace TimeStats {
void reset();
void addEvaluador(double seconds);
void addDecodificador(double seconds);
double totalEvaluador();
double totalDecodificador();
}
