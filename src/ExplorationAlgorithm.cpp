/*
 * Algorithme d'exploration par frontières avec WallFollower pour déblocage
 * 
 * Stratégie:
 * 1. Explorer en mode frontier-based normalement
 * 2. Si bloqué, passer en mode WallFollower pendant un certain temps
 * 3. Après le wall-following, reprendre l'exploration frontier-based
 * 
 * Convention LiDAR:
 * - Rayon 180 = avant, Rayon 270 = droite
 * - Rayon 90 = gauche, Rayon 0 = arrière
 */


#include "ExplorationAlgorithm.h"
#include <iostream>
#include <cmath>
#include <limits>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ExplorationAlgorithm::ExplorationAlgorithm(float moveSpeed, float rotateSpeed)
    : m_currentState(FINDING_FRONTIER),
      m_targetFrontier(-1, -1),
      m_currentPathIndex(0),
      m_moveSpeed(moveSpeed),
      m_rotateSpeed(rotateSpeed),
      m_stuckCounter(0),
      m_lastPosition(0, 0),
      m_wallFollower(40.0f,moveSpeed, rotateSpeed),  // Même paramètres
      m_wallFollowCounter(0),
      m_wallFollowMaxDuration(90)  // ~3 secondes à 30 FPS
{
    std::cout << "[EXPLORE] Initialisé avec WallFollower de secours" << std::endl;
}

void ExplorationAlgorithm::update(Robot& robot, const std::vector<float>& lidarData, const OccupancyGrid& grid)
{
    if (m_currentState == COMPLETED) {
        return;
    }

    // ===== MODE WALL_FOLLOWING (déblocage) =====
    if (m_currentState == WALL_FOLLOWING) {
        m_wallFollowCounter++;
        
        // Utiliser le WallFollower
        m_wallFollower.update(robot, lidarData);
        
        // Vérifier si on a assez suivi le mur
        if (m_wallFollowCounter >= m_wallFollowMaxDuration) {
            std::cout << "[EXPLORE] Fin du wall-following, reprise exploration" << std::endl;
            m_currentState = FINDING_FRONTIER;
            m_wallFollowCounter = 0;
            m_stuckCounter = 0;
            m_wallFollower.reset();
        }
        
        return;
    }

    // ===== DÉTECTION DE BLOCAGE =====
    float dx = robot.getX() - m_lastPosition.x;
    float dy = robot.getY() - m_lastPosition.y;
    float moved = std::sqrt(dx*dx + dy*dy);
    
    if (moved < 0.5f) {
        m_stuckCounter++;
    } else {
        m_stuckCounter = 0;
    }
    m_lastPosition = robot.getPosition();

    // Si bloqué trop longtemps -> passer en wall-following
    if (m_stuckCounter > 40) {
        std::cout << "[EXPLORE] Bloqué! Activation WallFollower pour déblocage..." << std::endl;
        // robot.rotate(robot.getTheta() - M_PI/2);
        m_currentState = WALL_FOLLOWING;
        m_wallFollowCounter = 0;
        m_stuckCounter = 0;
        m_wallFollower.reset();
        return;
    }

    // ===== ÉVITEMENT D'OBSTACLES PRIORITAIRE =====
    float front = getMinDistanceInRange(lidarData, 170, 190);
    float left = getMinDistanceInRange(lidarData, 80, 100);
    float right = getMinDistanceInRange(lidarData, 260, 280);

    if (front < 30.0f) {
        // Obstacle très proche - tourner vers le côté le plus libre
        if (left > right) {
            robot.rotate(-m_rotateSpeed * 2.0f);
        } else {
            robot.rotate(m_rotateSpeed * 2.0f);
        }
        return;
    }

    // ===== MACHINE À ÉTATS EXPLORATION =====
    switch (m_currentState) {
        case FINDING_FRONTIER:
            findFrontierAndNavigate(robot, lidarData, grid);
            break;

        case MOVING_TO_FRONTIER:
            moveTowardsFrontier(robot, lidarData, grid);
            break;

        case WALL_FOLLOWING:
        case COMPLETED:
            break;
    }
}

void ExplorationAlgorithm::findFrontierAndNavigate(Robot& robot, const std::vector<float>& lidarData, const OccupancyGrid& grid)
{
    static int noFrontierCounter = 0;
    
    std::vector<Cell> frontiers = findAllFrontiers(grid);

    if (frontiers.empty()) {
        noFrontierCounter++;
        
        // Explorer pour trouver des frontières
        float front = getMinDistanceInRange(lidarData, 170, 190);
        float left = getMinDistanceInRange(lidarData, 80, 100);
        float right = getMinDistanceInRange(lidarData, 260, 280);
        
        // Aller vers la direction la plus ouverte
        if (front > 50.0f && front >= left && front >= right) {
            robot.moveForward(m_moveSpeed);
        } else if (left > right) {
            robot.rotate(-m_rotateSpeed);
            if (left > 20.0f) robot.moveForward(m_moveSpeed * 0.5f);
        } else {
            robot.rotate(m_rotateSpeed);
            if (right > 20.0f) robot.moveForward(m_moveSpeed * 0.5f);
        }
        
        // Après beaucoup d'essais sans frontière, terminer
        if (noFrontierCounter > 600) {
            std::cout << "[EXPLORE] ✓ Exploration TERMINÉE" << std::endl;
            m_currentState = COMPLETED;
            noFrontierCounter = 0;
        }
        return;
    }
    
    // Frontières trouvées
    noFrontierCounter = 0;
    
    // Sélectionner la frontière la plus proche
    m_targetFrontier = selectClosestFrontier(frontiers, robot.getX(), robot.getY());
    m_currentState = MOVING_TO_FRONTIER;
    
    std::cout << "[EXPLORE] Cible: (" << m_targetFrontier.x << ", " << m_targetFrontier.y << ") - " << frontiers.size() << " frontières" << std::endl;
}

void ExplorationAlgorithm::moveTowardsFrontier(Robot& robot, const std::vector<float>& lidarData, const OccupancyGrid& grid)
{
    // Vérifier si la cible est atteinte
    float distToTarget = distanceTo(robot.getX(), robot.getY(), static_cast<float>(m_targetFrontier.x), static_cast<float>(m_targetFrontier.y));

    if (distToTarget < 25.0f) {
        m_currentState = FINDING_FRONTIER;
        m_currentPath.clear();
        return;
    }

    // Vérifier si la cible est encore une frontière valide
    if (!isFrontierCell(grid, m_targetFrontier.x, m_targetFrontier.y)) {
        m_currentState = FINDING_FRONTIER;
        m_currentPath.clear();
        return;
    }

    // Calculer le chemin si nécessaire
    if (m_currentPath.empty()) {
        Cell start(static_cast<int>(robot.getX()), static_cast<int>(robot.getY()));
        Cell goal(m_targetFrontier.x, m_targetFrontier.y);
        
        m_currentPath = m_pathPlanner.findPath(start, goal, grid);
        m_currentPathIndex = 0;
        
        if (m_currentPath.empty()) {
            // Pas de chemin trouvé - changer de frontière
            std::cout << "[EXPLORE] Pas de chemin vers frontière, en cherche une autre" << std::endl;
            m_currentState = FINDING_FRONTIER;
            return;
        }
        
        // Simplifier le chemin
        m_currentPath = m_pathPlanner.simplifyPath(m_currentPath, grid);
        std::cout << "[EXPLORE] Chemin calculé: " << m_currentPath.size() << " waypoints" << std::endl;
    }

    // Suivre le chemin
    if (m_currentPathIndex >= static_cast<int>(m_currentPath.size())) {
        m_currentState = FINDING_FRONTIER;
        m_currentPath.clear();
        return;
    }

    Cell waypoint = m_currentPath[m_currentPathIndex];
    float distToWaypoint = distanceTo(robot.getX(), robot.getY(),static_cast<float>(waypoint.x),static_cast<float>(waypoint.y));

    // Waypoint atteint - passer au suivant
    if (distToWaypoint < 15.0f) {
        m_currentPathIndex++;
        if (m_currentPathIndex >= static_cast<int>(m_currentPath.size())) {
            m_currentState = FINDING_FRONTIER;
            m_currentPath.clear();
        }
        return;
    }

    // Distances LiDAR
    float front = getMinDistanceInRange(lidarData, 170, 190);
    float left = getMinDistanceInRange(lidarData, 80, 100);
    float right = getMinDistanceInRange(lidarData, 260, 280);

    // Calculer l'angle vers le waypoint
    float targetAngle = std::atan2(
        static_cast<float>(waypoint.y) - robot.getY(),
        static_cast<float>(waypoint.x) - robot.getX()
    );
    
    float angleDiff = targetAngle - robot.getTheta();
    while (angleDiff > M_PI) angleDiff -= 2.0f * M_PI;
    while (angleDiff < -M_PI) angleDiff += 2.0f * M_PI;

    // Navigation avec évitement réactif
    if (front < 30.0f) {
        // Obstacle imprévu - contourner
        if (left > right) {
            robot.rotate(-m_rotateSpeed * 1.5f);
        } else {
            robot.rotate(m_rotateSpeed * 1.5f);
        }
        // Invalider le chemin pour le recalculer
        m_currentPath.clear();
    } else if (std::abs(angleDiff) > 0.25f) {
        // S'orienter vers le waypoint
        float rotSpeed = std::min(std::abs(angleDiff) * 0.5f, m_rotateSpeed);
        robot.rotate((angleDiff > 0) ? rotSpeed : -rotSpeed);
        // Avancer un peu même en tournant
        if (front > 50.0f) {
            robot.moveForward(m_moveSpeed * 0.3f);
        }
    } else {
        // Bien orienté - avancer
        robot.moveForward(m_moveSpeed);
    }
}

float ExplorationAlgorithm::getMinDistanceInRange(const std::vector<float>& lidarData, int start, int end) const
{
    if (lidarData.size() != 360) return 500.0f;
    
    float minDist = std::numeric_limits<float>::max();
    for (int i = start; i <= end; ++i) {
        int idx = ((i % 360) + 360) % 360;
        if (lidarData[idx] > 0 && lidarData[idx] < minDist) {
            minDist = lidarData[idx];
        }
    }
    return minDist;
}

bool ExplorationAlgorithm::isComplete() const
{
    return m_currentState == COMPLETED;
}

void ExplorationAlgorithm::reset()
{
    m_currentState = FINDING_FRONTIER;
    m_targetFrontier = Cell(-1, -1);
    m_currentPath.clear();
    m_currentPathIndex = 0;
    m_stuckCounter = 0;
    m_wallFollowCounter = 0;
    m_wallFollower.reset();
    std::cout << "[EXPLORE] Réinitialisé" << std::endl;
}

bool ExplorationAlgorithm::isFrontierCell(const OccupancyGrid& grid, int x, int y) const
{
    if (!grid.isInBounds(x, y)) return false;
    if (grid.getCellState(x, y) != OccupancyGrid::FREE) return false;

    // Vérifier les 8 voisins
    const int dx[] = {0, 0, -1, 1, -1, -1, 1, 1};
    const int dy[] = {-1, 1, 0, 0, -1, 1, -1, 1};

    for (int i = 0; i < 8; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (grid.isInBounds(nx, ny) && 
            grid.getCellState(nx, ny) == OccupancyGrid::UNEXPLORED) {
            return true;
        }
    }
    return false;
}

std::vector<Cell> ExplorationAlgorithm::findAllFrontiers(const OccupancyGrid& grid) const
{
    std::vector<Cell> frontiers;
    int step =  5; 
    for (int y = step; y < grid.getHeight() - step; y += step) {
        for (int x = step; x < grid.getWidth() - step; x += step) {
            if (isFrontierCell(grid, x, y)) {
                frontiers.push_back(Cell(x, y));
            }
        }
    }
    return frontiers;
}

Cell ExplorationAlgorithm::selectClosestFrontier(const std::vector<Cell>& frontiers, float robotX, float robotY) const
{
    if (frontiers.empty()) return Cell(-1, -1);

    float minDist = std::numeric_limits<float>::max();
    Cell closest = frontiers[0];

    for (const Cell& f : frontiers) {
        float dist = distanceTo(robotX, robotY,static_cast<float>(f.x), static_cast<float>(f.y));
        // Préférer les frontières pas trop proches (éviter oscillations)
        if (dist > 30.0f && dist < minDist) {
            minDist = dist;
            closest = f;
        }
    }

    // Si aucune frontière à bonne distance, prendre la plus proche
    if (minDist == std::numeric_limits<float>::max()) {
        for (const Cell& f : frontiers) {
            float dist = distanceTo(robotX, robotY, static_cast<float>(f.x),static_cast<float>(f.y));
            if (dist < minDist) {
                minDist = dist;
                closest = f;
            }
        }
    }

    return closest;
}

float ExplorationAlgorithm::distanceTo(float x1, float y1, float x2, float y2) const
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}
