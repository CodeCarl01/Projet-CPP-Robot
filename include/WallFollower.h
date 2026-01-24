/**
 Algorithme de suivi de murs 
 * 
 * Choisit automatiquement le mur le plus proche (gauche ou droite)
 * et le suit. Si les distances sont égales, choisit le mur droit.
 */

#ifndef WALL_FOLLOWER_H
#define WALL_FOLLOWER_H

#include <vector>
#include "Robot.h"

class WallFollower {
public:
    /*
     Côté du mur à suivre
     */
    enum WallSide {
        WALL_RIGHT,  // Suivre le mur à droite (main droite)
        WALL_LEFT    // Suivre le mur à gauche (main gauche)
    };

    /**
     * Constructeur
     * wallDistance Distance à maintenir du mur (pixels)
     * moveSpeed Vitesse de déplacement
     *rotateSpeed Vitesse de rotation
     */
    WallFollower(float wallDistance = 30.0f, float moveSpeed = 3.0f, float rotateSpeed = 0.1f);

    /*
     Destructeur
     */
    ~WallFollower();

    /**
     *  Mettre à jour le comportement du robot
     * robot Référence au robot
     *  lidarData Données du LiDAR (360 mesures)
     */
    void update(Robot& robot, const std::vector<float>& lidarData);

    /**
     *Vérifier si le suivi est terminé
     * (ex: retour au point de départ)
     */
    bool isComplete() const;

    /*
    Réinitialiser l'algorithme
     */
    void reset();

    /*
     Obtenir les paramètres
     */
    float getWallDistance() const { return m_wallDistance; }
    float getMoveSpeed() const { return m_moveSpeed; }
    float getRotateSpeed() const { return m_rotateSpeed; }
    WallSide getWallSide() const { return m_wallSide; }

    /*
     Modifier les paramètres
     */
    void setWallDistance(float distance) { m_wallDistance = distance; }
    void setMoveSpeed(float speed) { m_moveSpeed = speed; }
    void setRotateSpeed(float speed) { m_rotateSpeed = speed; }

private:
    float m_wallDistance;   // Distance désirée du mur
    float m_moveSpeed;      // Vitesse de déplacement
    float m_rotateSpeed;    // Vitesse de rotation

    // État interne
    bool m_started;
    bool m_wallChosen;      // Le mur a-t-il été choisi ?
    WallSide m_wallSide;    // Côté du mur choisi
    cv::Point2f m_startPosition;
    int m_stepCount;
    bool m_complete;

    // Secteurs LiDAR (indices)
    static const int FRONT_START = 170;
    static const int FRONT_END = 190;
    static const int RIGHT_START = 260;
    static const int RIGHT_END = 280;
    static const int LEFT_START = 80;
    static const int LEFT_END = 100;

    /**
       Choisir quel mur suivre (le plus proche)
     */
    void chooseWallSide(const std::vector<float>& lidarData);

    /**
      Obtenir la distance minimale dans un secteur
     */
    float getMinDistance(const std::vector<float>& lidarData, int start, int end) const;

    /**
      Obtenir la distance minimale dans un secteur (angles)
     */
    float getMinDistanceInSector(const std::vector<float>& lidarData, int startAngle, int endAngle) const;

    /**
      Vérifier si un obstacle est devant
     */
    bool hasObstacleFront(const std::vector<float>& lidarData) const;

    /**
      Vérifier si un mur est à droite
     */
    bool hasWallRight(const std::vector<float>& lidarData) const;

    /**
     Vérifier si un mur est à gauche
     */
    bool hasWallLeft(const std::vector<float>& lidarData) const;
};

#endif // WALL_FOLLOWER_H