/*
 Simulation du capteur LiDAR 
 * 
 * Simule un LiDAR 360° qui utilise une carte de référence
 * pour calculer les distances via raycasting.
 * 
 * Convention: 
 * - Rayon 0 = arrière du robot
 * - Rayon 180 = avant du robot (aligné avec la caméra)
 */

#ifndef LIDAR_H
#define LIDAR_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <string>

class LiDAR {
public:
    /**
     * @brief Constructeur avec chemin de la carte
     * @param mapImagePath Chemin vers l'image de la carte (PNG/JPG)
     * @param maxRange Portée maximale du LiDAR en pixels (défaut: 500)
     */
    explicit LiDAR(const std::string& mapImagePath, float maxRange = 500.0f);

    /**
     * @brief Vérifie si le LiDAR est valide (carte chargée)
     */
    bool isValid() const;

    /**
     * Obtenir les 360 mesures de distance
     * x Position X du robot (pixels)
     * y Position Y du robot (pixels)
     * theta Orientation du robot (radians)
     * return Vecteur de 360 distances (une par degré)
     */
    std::vector<float> getMeasurements(float x, float y, float theta) const;

    /**
     *  Version alternative avec cv::Point2f
     */
    std::vector<float> scan(const cv::Point2f& position, float theta) const;

    /**
     *  Obtenir le nombre de rayons (toujours 360)
     */
    int getNumRays() const { return 360; }

    /**
     * Obtenir la portée maximale
     */
    float getMaxRange() const { return m_maxRange; }

    /**
     *  Obtenir les dimensions de la carte
     */
    int getMapWidth() const;
    int getMapHeight() const;

    /**
     *  Vérifier si un point est un obstacle
     */
    bool isObstacle(int x, int y) const;

private:
    cv::Mat m_map;        // Carte de référence en niveaux de gris
    float m_maxRange;     // Portée maximale en pixels

    /**
     * Algorithme de raycasting pour un rayon
     * startX Point de départ X
     * startY Point de départ Y
     * angle Angle du rayon
     * return Distance jusqu'à l'obstacle ou maxRange
     */
    float castRay(float startX, float startY, float angle) const;
};

#endif // LIDAR_H
