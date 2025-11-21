#pragma once
#include "grid.h"
#include <vector>

std::vector<Pos> aStar(const Grid& grid, const Pos& start, const Pos& goal);
