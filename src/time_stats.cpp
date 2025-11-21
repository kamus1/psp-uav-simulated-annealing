#include "time_stats.h"

namespace {
double evaluadorSeconds = 0.0;
double decodificadorSeconds = 0.0;
}

void TimeStats::reset() {
    evaluadorSeconds = 0.0;
    decodificadorSeconds = 0.0;
}

void TimeStats::addEvaluador(double seconds) {
    evaluadorSeconds += seconds;
}

void TimeStats::addDecodificador(double seconds) {
    decodificadorSeconds += seconds;
}

double TimeStats::totalEvaluador() {
    return evaluadorSeconds;
}

double TimeStats::totalDecodificador() {
    return decodificadorSeconds;
}
