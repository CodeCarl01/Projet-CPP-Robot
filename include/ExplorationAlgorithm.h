/*
 Algorithme d'exploration automatique 

 * Implémente l'exploration basée sur les frontières:
 * 1. Trouver les cellules frontières (FREE adjacentes à UNEXPLORED)
 * 2. Choisir la frontière la plus proche
 * 3. Planifier un chemin vers cette frontière
 * 4. Suivre le chemin
 * 5. Répéter jusqu'à ce qu'il n'y ait plus de frontières
 * 
 * En cas de blocage, utilise temporairement le WallFollower pour se dégager.
 */

#ifndef EXPLORATION_ALGORITHM_H
#define EXPLORATION_ALGORITHM_H

#include <vector>
#include "Robot.h"
#include "OccupancyGrid.h"
#include "PathPlanner.h"
#include "WallFollower.h"

class ExplorationAlgorithm {
public:
    /*
         États de l'exploration
     */
    enum State {
        FINDING_FRONTIER,    // Cherche une nouvelle frontière
        MOVING_TO_FRONTIER,  // Se déplace vers la frontière
        WALL_FOLLOWING,      // Utilise WallFollower pour se débloquer
        COMPLETED            // Exploration terminée
    };

    /**
     * Constructeur
     * Paramètre moveSpeed Vitesse de déplacement
     * Paramètre rotateSpeed Vitesse de rotation
     */
    ExplorationAlgorithm(float moveSpeed = 3.0f, float rotateSpeed = 0.15f);

    /*
     * Mettre à jour l'exploration
     * robot Référence au robot
     *lidarData Données du LiDAR
     * grid Grille d'occupation
     */
    void update(Robot& robot, const std::vector<float>& lidarData, 
                const OccupancyGrid& grid);

    /*
    Vérifier si l'exploration est terminée
     */
    bool isComplete() const;

    /*
     *  Réinitialiser l'algorithme
     */
    void reset();

    /*
     * Obtenir l'état actuel
     */
    State getState() const { return m_currentState; }

    /*
     *Obtenir la cible actuelle
     */
    Cell getTargetFrontier() const { return m_targetFrontier; }

    /*
    * Obtenir le chemin actuel
     */
    std::vector<Cell> getCurrentPath() const { return m_currentPath; }

private:
    State m_currentState;
    Cell m_targetFrontier;
    std::vector<Cell> m_currentPath;
    int m_currentPathIndex;

    PathPlanner m_pathPlanner;
    float m_moveSpeed;
    float m_rotateSpeed;

    // Compteur pour détecter les blocages
    int m_stuckCounter;
    cv::Point2f m_lastPosition;

    // WallFollower pour le déblocage
    WallFollower m_wallFollower;
    int m_wallFollowCounter;        // Durée du wall following
    int m_wallFollowMaxDuration;    // Durée max avant retour à l'exploration

    /**
         Trouver une frontière et commencer la navigation
     */
    void findFrontierAndNavigate(Robot& robot, const std::vector<float>& lidarData,const OccupancyGrid& grid);

    /*
      Se déplacer vers la frontière cible
     */
    void moveTowardsFrontier(Robot& robot, const std::vector<float>& lidarData,const OccupancyGrid& grid);

    /*
     Obtenir la distance minimale dans une plage de rayons
     */
    float getMinDistanceInRange(const std::vector<float>& lidarData, int start, int end) const;

    /*
       Vérifier si une cellule est une frontière
    */
    bool isFrontierCell(const OccupancyGrid& grid, int x, int y) const;

    /*
     Trouver toutes les cellules frontières
    */
    std::vector<Cell> findAllFrontiers(const OccupancyGrid& grid) const;

    /*
     Sélectionner la frontière la plus proche
    */
    Cell selectClosestFrontier(const std::vector<Cell>& frontiers,float robotX, float robotY) const;

    /*
    Calculer la distance à un point
     */
    float distanceTo(float x1, float y1, float x2, float y2) const;
};

#endif // EXPLORATION_ALGORITHM_H
