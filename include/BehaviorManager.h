/*
  Gestionnaire de comportements
 * 
 * Gère les différents comportements du robot selon le tag ArUco détecté:
 * - Tag 0: Contrôle manuel
 * - Tag 1: Exploration automatique
 * - Tag 2: Suivi de murs (wall-following)
 * 
 * Le comportement persiste même si le tag disparaît,
 * et change uniquement lors de la détection d'un nouveau tag différent.
 */

#ifndef BEHAVIOR_MANAGER_H
#define BEHAVIOR_MANAGER_H

#include "Robot.h"
#include "OccupancyGrid.h"
#include "KeyboardControl.h"
#include "ExplorationAlgorithm.h"
#include "WallFollower.h"
#include "PathPlanner.h"
#include <vector>
#include <string>

class BehaviorManager {
public:
    // Types de comportements disponibles
    enum BehaviorType {
        MANUAL = 0,
        EXPLORATION = 1,
        WALL_FOLLOW = 2,
        RETURN_TO_ORIGIN = 3
    };
    
    BehaviorManager(BehaviorType initialBehavior = MANUAL);

    ~BehaviorManager();

    /*
        Mettre à jour le comportement selon le tag détecté
        return true si le comportement a changé
     */
    bool updateBehavior(int tagID);

    // Exécuter le comportement actuel

    void executeBehavior(Robot& robot, const std::vector<float>& lidarData,OccupancyGrid& grid, int key = -1);
    
    //Vérifier si le comportement actuel est terminé
    
    bool isBehaviorComplete() const;

    
    //Obtenir le comportement actuel

    BehaviorType getCurrentBehavior() const;

    //Obtenir le nom du comportement actuel
     
    std::string getCurrentBehaviorName() const;

    // Définir directement le comportement
     
    void setBehavior(BehaviorType behavior);

    
    //Déclencher le retour à l'origine

    void triggerReturnToOrigin();

    // Vérifier si c'est un comportement automatique
    bool isAutomaticBehavior() const;

    // Réinitialiser les algorithmes

    void reset();

private:
    BehaviorType m_currentBehavior;
    bool m_behaviorComplete;

    // Modules de comportement
    KeyboardControl m_keyboardControl;
    ExplorationAlgorithm m_exploration;
    WallFollower m_wallFollower;
    PathPlanner m_pathPlanner;

    // Pour le retour à l'origine
    std::vector<Cell> m_returnPath;
    int m_returnPathIndex;
    bool m_isReturning;

    // Convertir un ID de tag en comportement
    
    BehaviorType tagToBehavior(int tagID) const;

    //Exécuter le retour à l'origine
    void executeReturnToOrigin(Robot& robot, const std::vector<float>& lidarData,const OccupancyGrid& grid);
};

#endif // BEHAVIOR_MANAGER_H
