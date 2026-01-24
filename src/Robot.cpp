/*
  Implémentation de la classe Robot
*/

#include "Robot.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Robot::Robot()
    : Robot(0.0f, 0.0f, 0.0f)
{
}

Robot::Robot(float x, float y, float theta)
    : m_position(x, y),
      m_theta(normalizeAngle(theta)),
      m_originPosition(x, y),
      m_originTheta(theta),
      m_v(0.0f),
      m_w(0.0f),
      m_collisionEnabled(true),
      m_lidarData(nullptr),
      m_minSafeDistance(15.0f)
{
}

void Robot::move(float dx, float dy)
{
    if (m_collisionEnabled && !canMove(dx, dy)) {
        return; // Collision détectée, ne pas bouger
    }
    m_position.x += dx;
    m_position.y += dy;
}

void Robot::moveForward(float distance)
{
    float dx = distance * std::cos(m_theta);
    float dy = distance * std::sin(m_theta);
    
    if (m_collisionEnabled && !canMove(dx, dy)) {
        return; // Collision détectée, ne pas bouger
    }
    m_position.x += dx;
    m_position.y += dy;
}

void Robot::rotate(float dtheta)
{
    m_theta = normalizeAngle(m_theta + dtheta);
}

void Robot::setPosition(float x, float y, float theta)
{
    m_position = cv::Point2f(x, y);
    m_theta = normalizeAngle(theta);
}

float Robot::getX() const
{
    return m_position.x;
}

float Robot::getY() const
{
    return m_position.y;
}

float Robot::getTheta() const
{
    return m_theta;
}

cv::Point2f Robot::getPosition() const
{
    return m_position;
}

void Robot::resetToOrigin()
{
    m_position = m_originPosition;
    m_theta = m_originTheta;
    m_v = 0.0f;
    m_w = 0.0f;
}

void Robot::setOrigin(float x, float y, float theta)
{
    m_originPosition = cv::Point2f(x, y);
    m_originTheta = theta;
}

cv::Point2f Robot::getOriginPosition() const
{
    return m_originPosition;
}

float Robot::getOriginTheta() const
{
    return m_originTheta;
}

void Robot::setLinearVelocity(float v)
{
    m_v = v;
}

void Robot::setAngularVelocity(float w)
{
    m_w = w;
}

float Robot::getLinearVelocity() const
{
    return m_v;
}

float Robot::getAngularVelocity() const
{
    return m_w;
}

void Robot::update(float dt)
{
    // Modèle cinématique unicycle
    float dx = m_v * std::cos(m_theta) * dt;
    float dy = m_v * std::sin(m_theta) * dt;
    move(dx, dy);
    rotate(m_w * dt);
}

bool Robot::isAtPosition(float targetX, float targetY, float tolerance) const
{
    float dx = targetX - m_position.x;
    float dy = targetY - m_position.y;
    float distance = std::sqrt(dx * dx + dy * dy);
    return distance <= tolerance;
}

float Robot::normalizeAngle(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

void Robot::setCollisionDetection(bool enabled)
{
    m_collisionEnabled = enabled;
}

void Robot::setLidarReference(const std::vector<float>* lidarData)
{
    m_lidarData = lidarData;
}

void Robot::setMinSafeDistance(float distance)
{
    m_minSafeDistance = distance;
}

bool Robot::canMove(float dx, float dy) const
{
    if (!m_lidarData || m_lidarData->size() != 360) {
        return true; // Pas de données LiDAR, permettre le mouvement
    }

    // Nouvelle position proposée
    float newX = m_position.x + dx;
    float newY = m_position.y + dy;

    // Vérifier plusieurs rayons autour de la direction du mouvement
    // Convention: rayon 180 = avant, rayon 270 = droite, rayon 90 = gauche
    
    // Calculer l'angle du mouvement relatif au robot
    float moveAngle = std::atan2(dy, dx);
    float relativeAngle = moveAngle - m_theta;
    
    // Normaliser entre 0 et 2*PI
    while (relativeAngle < 0) relativeAngle += 2 * M_PI;
    while (relativeAngle >= 2 * M_PI) relativeAngle -= 2 * M_PI;
    
    // Convertir en index LiDAR (ajouter 180 car rayon 180 = avant)
    int centerRay = static_cast<int>(relativeAngle * 180.0f / M_PI) + 180;
    centerRay = centerRay % 360;
    
    // Distance du mouvement
    float moveDist = std::sqrt(dx * dx + dy * dy);

    // Vérifier les rayons dans un arc de ±30 degrés autour de la direction
    for (int offset = -30; offset <= 30; offset += 5) {
        int rayIdx = (centerRay + offset + 360) % 360;
        float rayDist = (*m_lidarData)[rayIdx];
        
        // Si un obstacle est plus proche que la distance de mouvement + marge de sécurité
        if (rayDist < moveDist + m_minSafeDistance) {
            return false; // Collision imminente
        }
    }
    
    return true;
}
