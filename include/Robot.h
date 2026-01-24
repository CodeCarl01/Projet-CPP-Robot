/*
 Classe principale du robot 
 * 
 * Gère la position, l'orientation et les déplacements du robot
 * dans l'environnement 2D de simulation.
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <opencv2/core.hpp>
#include <cmath>

class Robot {
public:
    /*
     Constructeur par défaut - robot à l'origine
     */
    Robot();

    /**
     *  Constructeur avec position initiale
     * x Position X initiale (en pixels)
     * y Position Y initiale (en pixels)
     * theta Orientation initiale (en radians)
     */
    Robot(float x, float y, float theta);

    // ============ API de déplacement ============

    /**
     *  Déplacement relatif dans le repère du monde
     *  dx Déplacement en X
     * dy Déplacement en Y
     */
    void move(float dx, float dy);

    /**
     *  Avancer dans la direction actuelle du robot
     *  distance Distance à parcourir
     */
    void moveForward(float distance);

    /**
     * Rotation du robot
     * dtheta Angle de rotation (en radians)
     */
    void rotate(float dtheta);

    /**
     *Définir la position absolue
     *x Nouvelle position X
     *  y Nouvelle position Y
     *  theta Nouvelle orientation
     */
    void setPosition(float x, float y, float theta);

    // ============ Accesseurs ============

    float getX() const;
    float getY() const;
    float getTheta() const;
    cv::Point2f getPosition() const;

    // ============ Origine ============

    /**
     *  Réinitialiser le robot à sa position d'origine
     */
    void resetToOrigin();

    /**
     Définir la position d'origine
     */
    void setOrigin(float x, float y, float theta);

    /**
      Obtenir la position d'origine
     */
    cv::Point2f getOriginPosition() const;
    float getOriginTheta() const;

    // ============ Vélocités (pour le modèle cinématique) ============

    void setLinearVelocity(float v);
    void setAngularVelocity(float w);
    float getLinearVelocity() const;
    float getAngularVelocity() const;

    /**
     * Mise à jour de la position selon le modèle unicycle
     * dt Intervalle de temps
     */
    void update(float dt);

    /**
     *  Vérifier si le robot est proche d'une position cible
     * targetX Position X cible
     *  targetY Position Y cible
     * tolerance Tolérance de distance
     */
    bool isAtPosition(float targetX, float targetY, float tolerance = 5.0f) const;

    /**
      Normaliser un angle entre -PI et PI
     */
    static float normalizeAngle(float angle);

    /**
      Activer/désactiver la détection de collision
     */
    void setCollisionDetection(bool enabled);
    
    /**
       Définir la référence au LiDAR pour la détection de collision
     */
    void setLidarReference(const std::vector<float>* lidarData);
    
    /**
     Vérifier si un déplacement est possible (pas de collision)
     */
    bool canMove(float dx, float dy) const;

    /**
       Distance minimale de sécurité
     */
    void setMinSafeDistance(float distance);

private:
    cv::Point2f m_position;      // Position actuelle (x, y)
    float m_theta;               // Orientation actuelle (radians)

    cv::Point2f m_originPosition; // Position d'origine [0,0]
    float m_originTheta;          // Orientation d'origine

    float m_v;  // Vélocité linéaire
    float m_w;  // Vélocité angulaire
    
    // Détection de collision
    bool m_collisionEnabled;
    const std::vector<float>* m_lidarData;
    float m_minSafeDistance;
};

#endif // ROBOT_H
