#include "a_star.h"
#include <queue>
#include <map>
#include <cmath>
#include <algorithm>

namespace {

int costo(const Pos&, const Pos&) {
    return 1;
}

int heuristica(const Pos& a, const Pos& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

std::vector<Pos> vecinos(const Grid& grid, const Pos& p) {
    std::vector<Pos> res;
    for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
            if (dr == 0 && dc == 0) continue;
            int nr = p.row + dr;
            int nc = p.col + dc;
            if (grid.esValida(nr, nc) && !grid.celdas[nr][nc].esObstaculo) {
                res.emplace_back(nr, nc);
            }
        }
    }
    res.push_back(p);
    return res;
}

} // namespace

std::vector<Pos> aStar(const Grid& grid, const Pos& start, const Pos& goal) {
    std::priority_queue<std::pair<int, Pos>, std::vector<std::pair<int, Pos>>, std::greater<>> frontera;
    frontera.emplace(0, start);

    std::map<Pos, Pos> cameFrom;
    std::map<Pos, int> costoActual;
    cameFrom[start] = start;
    costoActual[start] = 0;

    while (!frontera.empty()) {
        Pos actual = frontera.top().second;
        frontera.pop();

        if (actual == goal) break;

        for (auto& next : vecinos(grid, actual)) {
            int nuevoCosto = costoActual[actual] + costo(actual, next);
            if (!costoActual.count(next) || nuevoCosto < costoActual[next]) {
                costoActual[next] = nuevoCosto;
                int prioridad = nuevoCosto + heuristica(next, goal);
                frontera.emplace(prioridad, next);
                cameFrom[next] = actual;
            }
        }
    }

    std::vector<Pos> path;
    if (!cameFrom.count(goal)) return path;

    for (Pos p = goal; p != start; p = cameFrom[p])
        path.push_back(p);
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}
