/*
 Planification de chemin avec A* 
 * 
 * Utilisé pour:
 * - Retour à l'origine après les comportements automatiques
 * - Navigation vers les frontières en exploration
 */

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include <map>
#include <cmath>
#include "OccupancyGrid.h"

/*
 Structure représentant une cellule de la grille
 */
struct Cell {
    int x;
    int y;

    Cell(int x_ = 0, int y_ = 0) : x(x_), y(y_) {}

    bool operator==(const Cell& other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Cell& other) const {
        return !(*this == other);
    }

    bool operator<(const Cell& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
};

/*
File de priorité simple pour A*
 */
class PriorityQueue {
public:
    void push(float priority, const Cell& cell);
    Cell pop();
    bool isEmpty() const;
    void clear();

private:
    std::vector<std::pair<float, Cell>> m_elements;
};

/**
 * Planificateur de chemin utilisant A*
 */
class PathPlanner {
public:
    /*
     Constructeur
     */
    PathPlanner();

    /**
     * Trouver un chemin entre deux points
     * start Point de départ
     *  goal Point d'arrivée
     * grid Grille d'occupation
     * return Vecteur de cellules formant le chemin (vide si impossible)
     */
    std::vector<Cell> findPath(const Cell& start, const Cell& goal,const OccupancyGrid& grid);

    /*
    Simplifier un chemin (réduire les waypoints)
     */
    std::vector<Cell> simplifyPath(const std::vector<Cell>& path,const OccupancyGrid& grid);

    /*
        Vérifier si une ligne est libre
     */
    bool isLineOfSightClear(const Cell& a, const Cell& b,const OccupancyGrid& grid) const;

private:
    /* Heuristique (distance euclidienne)
     */
    float heuristic(const Cell& a, const Cell& b) const;

    /*Vérifie si une cellule est sûre (avec marge de sécurité)
     */
    bool isCellSafe(int x, int y, const OccupancyGrid& grid) const;

    /*Vérifie si une cellule est traversable (marge réduite)
     */
    bool isCellPassable(int x, int y, const OccupancyGrid& grid) const;

    /*Obtenir les voisins valides
     */
    std::vector<Cell> getNeighbors(const Cell& cell,const OccupancyGrid& grid) const;

    /* Obtenir les voisins en 8 directions
     */
    std::vector<Cell> getNeighbors8(const Cell& cell,const OccupancyGrid& grid) const;

    /* Reconstruire le chemin depuis la map des parents
     */
    std::vector<Cell> reconstructPath(const std::map<Cell, Cell>& parents,const Cell& start, const Cell& goal) const;
};

#endif // PATH_PLANNER_H
