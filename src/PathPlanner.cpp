/*
    Implémentation de l'algorithme A* optimisé
 */

#include "PathPlanner.h"
#include <algorithm>
#include <iostream>
#include <cmath>
#include <queue>

// Résolution de la recherche (en pixels)
const int GRID_STEP = 5;
// Marge de sécurité autour des obstacles (en pixels)
const int SAFETY_MARGIN = 10;
// Nombre max d'itérations A*
const int MAX_ITERATIONS = 50000;

// ============ PriorityQueue ============

void PriorityQueue::push(float priority, const Cell& cell)
{
    m_elements.push_back(std::make_pair(priority, cell));
}

Cell PriorityQueue::pop()
{
    if (isEmpty()) {
        return Cell(-1, -1);
    }

    auto minIt = std::min_element(m_elements.begin(), m_elements.end(),
        [](const std::pair<float, Cell>& a, const std::pair<float, Cell>& b) {
            return a.first < b.first;
        });

    Cell result = minIt->second;
    m_elements.erase(minIt);
    return result;
}

bool PriorityQueue::isEmpty() const
{
    return m_elements.empty();
}

void PriorityQueue::clear()
{
    m_elements.clear();
}

// ============ PathPlanner ============

PathPlanner::PathPlanner(){}


// Vérifie si une cellule est traversable avec marge de sécurité

bool PathPlanner::isCellSafe(int x, int y, const OccupancyGrid& grid) const
{
    // Vérifier la cellule elle-même et une zone autour
    for (int dy = -SAFETY_MARGIN; dy <= SAFETY_MARGIN; dy += 3) {
        for (int dx = -SAFETY_MARGIN; dx <= SAFETY_MARGIN; dx += 3) {
            int nx = x + dx;
            int ny = y + dy;
            
            if (!grid.isInBounds(nx, ny)) {
                return false;  // Hors limites = pas sûr
            }
            
            OccupancyGrid::CellState state = grid.getCellState(nx, ny);
            if (state == OccupancyGrid::OCCUPIED) {
                return false;  // Obstacle proche
            }
        }
    }
    return true;
}


//   Vérifie si une cellule est traversable
 
bool PathPlanner::isCellPassable(int x, int y, const OccupancyGrid& grid) const
{
    // Marge réduite pour les passages étroits
    const int smallMargin = 5;
    
    for (int dy = -smallMargin; dy <= smallMargin; dy += 2) {
        for (int dx = -smallMargin; dx <= smallMargin; dx += 2) {
            int nx = x + dx;
            int ny = y + dy;
            
            if (!grid.isInBounds(nx, ny)) {
                continue;
            }
            
            if (grid.getCellState(nx, ny) == OccupancyGrid::OCCUPIED) {
                return false;
            }
        }
    }
    return true;
}

std::vector<Cell> PathPlanner::findPath(const Cell& start, const Cell& goal, const OccupancyGrid& grid)
{
    // Arrondir aux coordonnées de grille réduite
    Cell gridStart(
        (start.x / GRID_STEP) * GRID_STEP,
        (start.y / GRID_STEP) * GRID_STEP
    );
    Cell gridGoal(
        (goal.x / GRID_STEP) * GRID_STEP,
        (goal.y / GRID_STEP) * GRID_STEP
    );

    // Vérifications de base
    if (!grid.isInBounds(gridStart.x, gridStart.y) || 
        !grid.isInBounds(gridGoal.x, gridGoal.y)) {
        std::cout << "[PATH] Départ ou arrivée hors limites" << std::endl;
        return std::vector<Cell>();
    }

    // Vérifier que la destination n'est pas un obstacle
    if (grid.getCellState(gridGoal.x, gridGoal.y) == OccupancyGrid::OCCUPIED) {
        std::cout << "[PATH] Destination est un obstacle" << std::endl;
        return std::vector<Cell>();
    }

    // Initialisation A*
    PriorityQueue openSet;
    openSet.push(0, gridStart);

    std::map<Cell, Cell> parents;
    std::map<Cell, float> gScore;
    gScore[gridStart] = 0;

    std::map<Cell, bool> closedSet;
    
    int iterations = 0;

    // Boucle principale A*
    while (!openSet.isEmpty() && iterations < MAX_ITERATIONS) {
        iterations++;
        Cell current = openSet.pop();

        // Objectif atteint (avec tolérance)
        if (std::abs(current.x - gridGoal.x) <= GRID_STEP && 
            std::abs(current.y - gridGoal.y) <= GRID_STEP) {
            std::cout << "[PATH] Chemin trouvé en " << iterations << " itérations" << std::endl;
            return reconstructPath(parents, gridStart, current);
        }

        if (closedSet[current]) {
            continue;
        }
        closedSet[current] = true;

        // Explorer les voisins (8 directions avec step)
        const int dx[] = {0, 0, -GRID_STEP, GRID_STEP, 
                          -GRID_STEP, -GRID_STEP, GRID_STEP, GRID_STEP};
        const int dy[] = {-GRID_STEP, GRID_STEP, 0, 0, 
                          -GRID_STEP, GRID_STEP, -GRID_STEP, GRID_STEP};

        for (int i = 0; i < 8; ++i) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];
            Cell neighbor(nx, ny);

            if (!grid.isInBounds(nx, ny)) continue;
            if (closedSet[neighbor]) continue;

            // Vérifier si la cellule est traversable
            // D'abord essayer avec marge de sécurité, sinon marge réduite
            bool passable = isCellSafe(nx, ny, grid);
            if (!passable) {
                passable = isCellPassable(nx, ny, grid);
            }
            if (!passable) continue;

            // Coût du mouvement
            float moveCost = (i >= 4) ? (GRID_STEP * 1.414f) : static_cast<float>(GRID_STEP);
            
            // Pénalité pour les cellules UNEXPLORED (on préfère rester dans les zones connues)
            if (grid.getCellState(nx, ny) == OccupancyGrid::UNEXPLORED) {
                moveCost *= 2.0f;  // Coût double pour traverser l'inconnu
            }

            float tentativeG = gScore[current] + moveCost;

            if (gScore.find(neighbor) == gScore.end() || tentativeG < gScore[neighbor]) {
                gScore[neighbor] = tentativeG;
                float h = heuristic(neighbor, gridGoal);
                openSet.push(tentativeG + h, neighbor);
                parents[neighbor] = current;
            }
        }
    }

    if (iterations >= MAX_ITERATIONS) {
        std::cout << "[PATH] Limite d'itérations atteinte (" << MAX_ITERATIONS << ")" << std::endl;
    } else {
        std::cout << "[PATH] Aucun chemin trouvé de (" << start.x << "," << start.y << ") à (" << goal.x << "," << goal.y << ")" << std::endl;
    }
    
    return std::vector<Cell>();
}

std::vector<Cell> PathPlanner::simplifyPath(const std::vector<Cell>& path, const OccupancyGrid& grid)
{
    if (path.size() <= 2) {
        return path;
    }

    std::vector<Cell> simplified;
    simplified.push_back(path[0]);

    size_t i = 0;
    while (i < path.size() - 1) {
        size_t farthest = i + 1;
        for (size_t j = i + 2; j < path.size(); ++j) {
            if (isLineOfSightClear(path[i], path[j], grid)) {
                farthest = j;
            }
        }
        simplified.push_back(path[farthest]);
        i = farthest;
    }

    std::cout << "[PATH] Simplifié: " << path.size() << " -> " << simplified.size() << " waypoints" << std::endl;
    return simplified;
}

bool PathPlanner::isLineOfSightClear(const Cell& a, const Cell& b, const OccupancyGrid& grid) const
{
    int dx = std::abs(b.x - a.x);
    int dy = std::abs(b.y - a.y);
    int sx = (a.x < b.x) ? 1 : -1;
    int sy = (a.y < b.y) ? 1 : -1;
    int err = dx - dy;

    int x = a.x;
    int y = a.y;

    while (true) {
        // Vérifier avec une petite marge
        for (int my = -3; my <= 3; ++my) {
            for (int mx = -3; mx <= 3; ++mx) {
                if (grid.isInBounds(x + mx, y + my) &&
                    grid.getCellState(x + mx, y + my) == OccupancyGrid::OCCUPIED) {
                    return false;
                }
            }
        }

        if (x == b.x && y == b.y) break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }

    return true;
}

float PathPlanner::heuristic(const Cell& a, const Cell& b) const
{
    float dx = static_cast<float>(a.x - b.x);
    float dy = static_cast<float>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<Cell> PathPlanner::getNeighbors(const Cell& cell, const OccupancyGrid& grid) const
{
    std::vector<Cell> neighbors;
    const int dx[] = {0, 0, -GRID_STEP, GRID_STEP};
    const int dy[] = {-GRID_STEP, GRID_STEP, 0, 0};

    for (int i = 0; i < 4; ++i) {
        int nx = cell.x + dx[i];
        int ny = cell.y + dy[i];

        if (grid.isInBounds(nx, ny) && isCellPassable(nx, ny, grid)) {
            neighbors.push_back(Cell(nx, ny));
        }
    }

    return neighbors;
}

std::vector<Cell> PathPlanner::getNeighbors8(const Cell& cell, const OccupancyGrid& grid) const
{
    std::vector<Cell> neighbors;
    const int dx[] = {0, 0, -GRID_STEP, GRID_STEP, -GRID_STEP, -GRID_STEP, GRID_STEP, GRID_STEP};
    const int dy[] = {-GRID_STEP, GRID_STEP, 0, 0, -GRID_STEP, GRID_STEP, -GRID_STEP, GRID_STEP};

    for (int i = 0; i < 8; ++i) {
        int nx = cell.x + dx[i];
        int ny = cell.y + dy[i];

        if (grid.isInBounds(nx, ny) && isCellPassable(nx, ny, grid)) {
            neighbors.push_back(Cell(nx, ny));
        }
    }

    return neighbors;
}

std::vector<Cell> PathPlanner::reconstructPath(const std::map<Cell, Cell>& parents, const Cell& start, const Cell& goal) const
{
    std::vector<Cell> path;
    Cell current = goal;

    int maxSteps = 10000;  // Sécurité
    int steps = 0;

    while (!(current == start) && steps < maxSteps) {
        path.push_back(current);
        auto it = parents.find(current);
        if (it == parents.end()) {
            std::cout << "[PATH] Erreur: chemin incomplet" << std::endl;
            return std::vector<Cell>();
        }
        current = it->second;
        steps++;
    }
    path.push_back(start);

    std::reverse(path.begin(), path.end());

    std::cout << "[PATH] Chemin reconstruit: " << path.size() << " waypoints" << std::endl;
    return path;
}
