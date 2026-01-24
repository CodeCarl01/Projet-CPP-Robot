/*
 Implémentation du gestionnaire de comportements
*/

#include "BehaviorManager.h"
#include <iostream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

BehaviorManager::BehaviorManager(BehaviorType initialBehavior)
    : m_currentBehavior(initialBehavior),
      m_behaviorComplete(false),
      m_keyboardControl(5.0f, 0.15f),
      m_exploration(4.0f, 0.12f),
      m_wallFollower(40.0f, 4.0f, 0.12f),
      m_returnPathIndex(0),
      m_isReturning(false)
{
    std::cout << "[BEHAVIOR] Gestionnaire initialisé en mode "<< getCurrentBehaviorName() << std::endl;
}

BehaviorManager::~BehaviorManager(){}

bool BehaviorManager::updateBehavior(int tagID)
{
    if (tagID < 0) {
        // Aucun tag détecté - le comportement persiste
        return false;
    }

    BehaviorType newBehavior = tagToBehavior(tagID);

    if (newBehavior != m_currentBehavior) {
        std::cout << "[BEHAVIOR] Changement: " << getCurrentBehaviorName() 
                  << " -> ";
        
        m_currentBehavior = newBehavior;
        m_behaviorComplete = false;
        m_isReturning = false;

        // Réinitialiser les algorithmes
        m_exploration.reset();
        m_wallFollower.reset();
        m_returnPath.clear();
        m_returnPathIndex = 0;

        std::cout << getCurrentBehaviorName() << std::endl;
        return true;
    }

    return false;
}

void BehaviorManager::executeBehavior(Robot& robot, const std::vector<float>& lidarData, OccupancyGrid& grid, int key)
{
    // Si en retour à l'origine
    if (m_isReturning) {
        executeReturnToOrigin(robot, lidarData, grid);
        return;
    }

    switch (m_currentBehavior) {
        case MANUAL:
            // Le contrôle manuel est géré par les touches clavier
            m_keyboardControl.processKey(key, robot);
            m_behaviorComplete = false;
            break;

        case EXPLORATION:
            m_exploration.update(robot, lidarData, grid);
            m_behaviorComplete = m_exploration.isComplete();
            
            if (m_behaviorComplete) {
                std::cout << "[BEHAVIOR] Exploration terminée - Retour à l'origine" << std::endl;
                triggerReturnToOrigin();
            }
            break;

        case WALL_FOLLOW:
            m_wallFollower.update(robot, lidarData);
            m_behaviorComplete = m_wallFollower.isComplete();
            
            if (m_behaviorComplete) {
                std::cout << "[BEHAVIOR] Suivi de murs terminé - Retour à l'origine" << std::endl;
                triggerReturnToOrigin();
            }
            break;

        case RETURN_TO_ORIGIN:
            executeReturnToOrigin(robot, lidarData, grid);
            break;
    }
}

bool BehaviorManager::isBehaviorComplete() const
{
    return m_behaviorComplete && !m_isReturning;
}

BehaviorManager::BehaviorType BehaviorManager::getCurrentBehavior() const
{
    if (m_isReturning) {
        return RETURN_TO_ORIGIN;
    }
    return m_currentBehavior;
}

std::string BehaviorManager::getCurrentBehaviorName() const
{
    if (m_isReturning) {
        return "RETURN_TO_ORIGIN";
    }

    switch (m_currentBehavior) {
        case MANUAL:
            return "MANUAL";
        case EXPLORATION:
            return "EXPLORATION";
        case WALL_FOLLOW:
            return "WALL_FOLLOW";
        case RETURN_TO_ORIGIN:
            return "RETURN_TO_ORIGIN";
        default:
            return "UNKNOWN";
    }
}

void BehaviorManager::setBehavior(BehaviorType behavior)
{
    if (behavior != m_currentBehavior) {
        m_currentBehavior = behavior;
        m_behaviorComplete = false;
        m_isReturning = false;
        
        // Réinitialiser
        m_exploration.reset();
        m_wallFollower.reset();
        
        std::cout << "[BEHAVIOR] Mode défini: " << getCurrentBehaviorName() << std::endl;
    }
}

void BehaviorManager::triggerReturnToOrigin()
{
    m_isReturning = true;
    m_returnPath.clear();
    m_returnPathIndex = 0;
    std::cout << "[BEHAVIOR] Démarrage du retour à l'origine" << std::endl;
}

bool BehaviorManager::isAutomaticBehavior() const
{
    return m_currentBehavior != MANUAL;
}

void BehaviorManager::reset()
{
    m_exploration.reset();
    m_wallFollower.reset();
    m_returnPath.clear();
    m_returnPathIndex = 0;
    m_behaviorComplete = false;
    m_isReturning = false;
}

BehaviorManager::BehaviorType BehaviorManager::tagToBehavior(int tagID) const
{
    switch (tagID) {
        case 0:
            return MANUAL;
        case 1:
            return EXPLORATION;
        case 2:
            return WALL_FOLLOW;
        default:
            return m_currentBehavior; // Garder le comportement actuel
    }
}

void BehaviorManager::executeReturnToOrigin(Robot& robot, const std::vector<float>& lidarData,const OccupancyGrid& grid)
{
    static std::vector<Cell> returnPath;
    static int pathIndex = 0;
    static int stuckCounter = 0;
    static cv::Point2f lastPosition(0, 0);
    
    // Obtenir la position d'origine
    cv::Point2f origin = robot.getOriginPosition();

    // Vérifier si on est arrivé
    if (robot.isAtPosition(origin.x, origin.y, 20.0f)) {
        std::cout << "[BEHAVIOR] ✓ Retour à l'origine terminé!" << std::endl;
        m_isReturning = false;
        m_behaviorComplete = true;
        m_currentBehavior = MANUAL;
        returnPath.clear();
        pathIndex = 0;
        stuckCounter = 0;
        return;
    }

    // Détecter si bloqué
    float dx = robot.getX() - lastPosition.x;
    float dy = robot.getY() - lastPosition.y;
    float moved = std::sqrt(dx*dx + dy*dy);
    lastPosition = robot.getPosition();
    
    if (moved < 0.5f) {
        stuckCounter++;
    } else {
        stuckCounter = 0;
    }

    // Si bloqué, recalculer le chemin
    if (stuckCounter > 40) {
        std::cout << "[BEHAVIOR] Bloqué en retour - recalcul du chemin" << std::endl;
        returnPath.clear();
        stuckCounter = 0;
        
        // Petite rotation pour se dégager
        robot.rotate(0.5f);
        return;
    }

    // Calculer le chemin si nécessaire
    if (returnPath.empty()) {
        Cell start(static_cast<int>(robot.getX()), static_cast<int>(robot.getY()));
        Cell goal(static_cast<int>(origin.x), static_cast<int>(origin.y));

        returnPath = m_pathPlanner.findPath(start, goal, grid);
        pathIndex = 0;

        if (returnPath.empty()) {
            std::cout << "[BEHAVIOR] Pas de chemin trouvé - navigation directe" << std::endl;
            // Navigation directe de secours
            float targetAngle = std::atan2(origin.y - robot.getY(), origin.x - robot.getX());
            float angleDiff = targetAngle - robot.getTheta();
            while (angleDiff > M_PI) angleDiff -= 2 * M_PI;
            while (angleDiff < -M_PI) angleDiff += 2 * M_PI;
            
            float front = 500.0f;
            for (int i = 170; i <= 190 && i < static_cast<int>(lidarData.size()); ++i) {
                if (lidarData[i] < front) front = lidarData[i];
            }
            
            if (front < 30.0f) {
                robot.rotate(0.15f);
            } else if (std::abs(angleDiff) > 0.2f) {
                robot.rotate((angleDiff > 0) ? 0.12f : -0.12f);
            } else {
                robot.moveForward(4.0f);
            }
            return;
        }
        
        // Simplifier le chemin
        returnPath = m_pathPlanner.simplifyPath(returnPath, grid);
        std::cout << "[BEHAVIOR] Chemin retour calculé: " << returnPath.size() << " waypoints" << std::endl;
    }

    // Suivre le chemin
    if (pathIndex >= static_cast<int>(returnPath.size())) {
        returnPath.clear();
        pathIndex = 0;
        return;
    }

    Cell waypoint = returnPath[pathIndex];
    float distToWaypoint = std::sqrt(
        std::pow(robot.getX() - waypoint.x, 2) + 
        std::pow(robot.getY() - waypoint.y, 2)
    );

    // Waypoint atteint - passer au suivant
    if (distToWaypoint < 15.0f) {
        pathIndex++;
        return;
    }

    // Calculer l'angle vers le waypoint
    float targetAngle = std::atan2(
        static_cast<float>(waypoint.y) - robot.getY(),
        static_cast<float>(waypoint.x) - robot.getX()
    );
    float angleDiff = targetAngle - robot.getTheta();
    while (angleDiff > M_PI) angleDiff -= 2 * M_PI;
    while (angleDiff < -M_PI) angleDiff += 2 * M_PI;

    // Obtenir distances LiDAR
    float front = 500.0f;
    float left = 500.0f;
    float right = 500.0f;
    
    if (lidarData.size() == 360) {
        for (int i = 170; i <= 190; ++i) {
            if (lidarData[i] < front) front = lidarData[i];
        }
        for (int i = 80; i <= 100; ++i) {
            if (lidarData[i] < left) left = lidarData[i];
        }
        for (int i = 260; i <= 280; ++i) {
            if (lidarData[i] < right) right = lidarData[i];
        }
    }

    // Navigation avec évitement
    if (front < 30.0f) {
        // Obstacle - recalculer le chemin
        returnPath.clear();
        if (left > right) {
            robot.rotate(-0.15f);
        } else {
            robot.rotate(0.15f);
        }
    } else if (std::abs(angleDiff) > 0.2f) {
        // S'orienter vers le waypoint
        float rotSpeed = std::min(std::abs(angleDiff) * 0.4f, 0.12f);
        robot.rotate((angleDiff > 0) ? rotSpeed : -rotSpeed);
        if (front > 50.0f) {
            robot.moveForward(2.0f);
        }
    } else {
        // Avancer vers le waypoint
        robot.moveForward(4.0f);
    }
}