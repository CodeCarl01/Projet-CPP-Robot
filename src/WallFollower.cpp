/*
Algorithme de suivi de murs adaptatif
 * 
 * Choisit automatiquement le mur le plus proche (gauche ou droite)
 * au démarrage et le suit. Si les distances sont égales, choisit 
 * le mur droit par défaut.
 * 
 * Convention LiDAR:
 * - Rayon 0 = arrière, Rayon 90 = gauche
 * - Rayon 180 = avant, Rayon 270 = droite
 */

#include "WallFollower.h"
#include <opencv2/core.hpp>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

WallFollower::WallFollower(float wallDistance, float moveSpeed, float rotateSpeed)
    : m_wallDistance(wallDistance),
      m_moveSpeed(moveSpeed),
      m_rotateSpeed(rotateSpeed),
      m_started(false),
      m_wallChosen(false),
      m_wallSide(WALL_RIGHT),  // Par défaut
      m_stepCount(0),
      m_complete(false)
{
    std::cout << "[WALL] Initialisé - Distance mur: " << wallDistance << ", Vitesse: " << moveSpeed << std::endl;
}

WallFollower::~WallFollower()
{
}

float WallFollower::getMinDistanceInSector(const std::vector<float>& lidarData, int startAngle, int endAngle) const
{
    float minDist = std::numeric_limits<float>::max();
    
    for (int i = startAngle; i <= endAngle; ++i) {
        int idx = ((i % 360) + 360) % 360;
        if (lidarData[idx] > 0 && lidarData[idx] < minDist) {
            minDist = lidarData[idx];
        }
    }
    
    return minDist;
}

void WallFollower::chooseWallSide(const std::vector<float>& lidarData)
{
    float leftDist = getMinDistanceInSector(lidarData, 80, 100);
    float rightDist = getMinDistanceInSector(lidarData, 260, 280);
    
    // Choisir le mur le plus proche
    // Si égaux, choisir le droit par défaut
    if (leftDist < rightDist) {
        m_wallSide = WALL_LEFT;
        std::cout << "[WALL] Mur GAUCHE choisi (L:" << (int)leftDist << " < R:" << (int)rightDist << ")" << std::endl;
    } else {
        m_wallSide = WALL_RIGHT;
        std::cout << "[WALL] Mur DROIT choisi (R:" << (int)rightDist << " <= L:" << (int)leftDist << ")" << std::endl;
    }
    
    m_wallChosen = true;
}

void WallFollower::update(Robot& robot, const std::vector<float>& lidarData)
{
    static int stuckCounter = 0;
    static cv::Point2f lastPosition(0, 0);
    
    if (m_complete) {
        return;
    }

    if (lidarData.size() != 360) {
        std::cerr << "[WALL] Erreur: LiDAR invalide" << std::endl;
        return;
    }

    // Enregistrer la position de départ
    if (!m_started) {
        m_startPosition = robot.getPosition();
        m_started = true;
        m_stepCount = 0;
        lastPosition = m_startPosition;
        stuckCounter = 0;
    }

    // Choisir le mur à suivre au premier pas
    if (!m_wallChosen) {
        chooseWallSide(lidarData);
    }

    m_stepCount++;

    // === DÉTECTION DE BLOCAGE ===
    float dx = robot.getX() - lastPosition.x;
    float dy = robot.getY() - lastPosition.y;
    float moved = std::sqrt(dx*dx + dy*dy);
    
    if (moved < 0.5f) {
        stuckCounter++;
    } else {
        stuckCounter = 0;
    }
    lastPosition = robot.getPosition();

    // Si bloqué trop longtemps - manœuvre de déblocage
    if (stuckCounter > 30) {
        std::cout << "[WALL] Bloqué! Manoeuvre de déblocage..." << std::endl;
        
        // Reculer un peu et tourner fortement loin du mur
        float awayRotation = (m_wallSide == WALL_RIGHT) ? -m_rotateSpeed : m_rotateSpeed;
        robot.rotate(awayRotation * 3.0f);
        
        // Après plusieurs tentatives, changer de côté
        if (stuckCounter > 60) {
            m_wallSide = (m_wallSide == WALL_RIGHT) ? WALL_LEFT : WALL_RIGHT;
            std::cout << "[WALL] Changement de côté: " 
                      << ((m_wallSide == WALL_RIGHT) ? "DROIT" : "GAUCHE") << std::endl;
            stuckCounter = 0;
        }
        return;
    }

    // Mesures LiDAR
    float front = getMinDistanceInSector(lidarData, 170, 190);
    float left = getMinDistanceInSector(lidarData, 80, 100);
    float right = getMinDistanceInSector(lidarData, 260, 280);
    float frontLeft = getMinDistanceInSector(lidarData, 120, 150);
    float frontRight = getMinDistanceInSector(lidarData, 210, 240);

    // Seuils
    const float DANGER = 15.0f;
    const float TOO_CLOSE = m_wallDistance * 0.06f;
    const float IDEAL_MIN = m_wallDistance * 0.1f;
    const float IDEAL_MAX = m_wallDistance * 0.5f;
    const float TOO_FAR = m_wallDistance * 1.5f;


    float wallDist, frontWallDist;
    float awayRotation, towardRotation;
    
    if (m_wallSide == WALL_RIGHT) {
        wallDist = right;
        frontWallDist = frontRight;
        awayRotation = -m_rotateSpeed;
        towardRotation = m_rotateSpeed;
    } else {
        wallDist = left;
        frontWallDist = frontLeft;
        awayRotation = m_rotateSpeed;
        towardRotation = -m_rotateSpeed;
    }

    // Debug toutes les 30 frames
    if (m_stepCount % 30 == 0) {
        const char* side = (m_wallSide == WALL_RIGHT) ? "R" : "L";
        std::cout << "[WALL-" << side << "] F:" << (int)front << " Wall:" << (int)wallDist << std::endl;
    }

    // ===========================================
    // ALGORITHME DE SUIVI DE MUR ADAPTATIF
    // ===========================================

    // CAS 1: Danger frontal - tourner loin du mur rapidement
    if (front < DANGER) {
        robot.rotate(awayRotation * 2.5f);
        return;
    }

    // CAS 2: Obstacle devant - tourner loin du mur
    if (front < m_wallDistance) {
        robot.rotate(awayRotation * 1.5f);
        return;
    }

    // CAS 3: Obstacle en avant côté mur - ajuster
    if (frontWallDist < TOO_CLOSE) {
        robot.rotate(awayRotation * 0.8f);
        robot.moveForward(m_moveSpeed * 0.4f);
        return;
    }

    // CAS 4: Pas de mur du côté choisi - tourner vers le mur pour le trouver
    if (wallDist > TOO_FAR) {
        robot.rotate(towardRotation * 0.6f);
        robot.moveForward(m_moveSpeed * 0.7f);
        return;
    }

    // CAS 5: Mur présent - le suivre avec correction proportionnelle
    if (wallDist < TOO_FAR) {
        float targetDist = (IDEAL_MIN + IDEAL_MAX) / 2.0f;
        
        float correction = 0.0f;
        
        if (wallDist < TOO_CLOSE) {
            // Trop proche - s'éloigner ET avancer
            correction = awayRotation * 0.5f;
            robot.rotate(correction);
            robot.moveForward(m_moveSpeed * 0.6f);  // Avancer même si trop proche
            return;
        } else if (wallDist > IDEAL_MAX) {
            // Trop loin - se rapprocher
            correction = towardRotation * 0.3f;
        } else {
            // Distance bonne - correction fine
            float error = wallDist - targetDist;
            correction = error * 0.003f;
            if (m_wallSide == WALL_LEFT) {
                correction = -correction;
            }
            correction = std::max(-m_rotateSpeed * 0.2f, 
                                  std::min(m_rotateSpeed * 0.2f, correction));
        }
        
        robot.rotate(correction);
        robot.moveForward(m_moveSpeed);
        return;
    }

    // CAS 6: Par défaut - avancer
    robot.moveForward(m_moveSpeed);

    // Vérifier retour au point de départ
    if (m_stepCount > 400) {
        float distX = robot.getX() - m_startPosition.x;
        float distY = robot.getY() - m_startPosition.y;
        float distToStart = std::sqrt(distX * distX + distY * distY);

        if (distToStart < 30.0f) {
            m_complete = true;
            std::cout << "[WALL] ✓ Boucle complète!" << std::endl;
        }
    }
}

bool WallFollower::isComplete() const
{
    return m_complete;
}

void WallFollower::reset()
{
    m_started = false;
    m_wallChosen = false;
    m_wallSide = WALL_RIGHT;
    m_stepCount = 0;
    m_complete = false;
    std::cout << "[WALL] Réinitialisé" << std::endl;
}

float WallFollower::getMinDistance(const std::vector<float>& lidarData, int start, int end) const
{
    return getMinDistanceInSector(lidarData, start, end);
}

bool WallFollower::hasObstacleFront(const std::vector<float>& lidarData) const
{
    return getMinDistanceInSector(lidarData, 170, 190) < m_wallDistance;
}

bool WallFollower::hasWallRight(const std::vector<float>& lidarData) const
{
    return getMinDistanceInSector(lidarData, 260, 280) < m_wallDistance * 2.0f;
}

bool WallFollower::hasWallLeft(const std::vector<float>& lidarData) const
{
    return getMinDistanceInSector(lidarData, 80, 100) < m_wallDistance * 2.0f;
}