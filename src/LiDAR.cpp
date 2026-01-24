/*
 Implémentation du LiDAR simulé avec raycasting
*/

#include "LiDAR.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LiDAR::LiDAR(const std::string& mapImagePath, float maxRange) : m_maxRange(maxRange)
{
    // Charger la carte en niveaux de gris
    m_map = cv::imread(mapImagePath, cv::IMREAD_GRAYSCALE);
    
}

bool LiDAR::isValid() const
{
    return !m_map.empty();
}

std::vector<float> LiDAR::getMeasurements(float x, float y, float theta) const
{
    return scan(cv::Point2f(x, y), theta);
}

std::vector<float> LiDAR::scan(const cv::Point2f& position, float theta) const
{
    std::vector<float> ranges(360, m_maxRange);
    
    if (!isValid()) {
        return ranges;
    }

    // Pour chaque degré (0 à 359)
    for (int i = 0; i < 360; ++i) {
        // Convention: rayon 0 = arrière, rayon 180 = avant (caméra)
        // Le rayon i est à (theta + (i - 180) degrés) par rapport à l'axe X global
        float rayAngle = theta + static_cast<float>(i - 180) * (M_PI / 180.0f);
        
        ranges[i] = castRay(position.x, position.y, rayAngle);
    }

    return ranges;
}

float LiDAR::castRay(float startX, float startY, float angle) const
{
    // Direction du rayon
    float dx = std::cos(angle);
    float dy = std::sin(angle);

    // Pas d'incrémentation (résolution du raycasting)
    const float step = 1.0f;
    
    float distance = 0.0f;
    float x = startX;
    float y = startY;

    while (distance < m_maxRange) {
        // Convertir en coordonnées entières
        int ix = static_cast<int>(std::round(x));
        int iy = static_cast<int>(std::round(y));

        // Vérifier les limites de la carte
        if (ix < 0 || iy < 0 || ix >= m_map.cols || iy >= m_map.rows) {
            return distance; // Sorti de la carte
        }

        // Vérifier si c'est un obstacle (pixel sombre)
        uchar pixel = m_map.at<uchar>(iy, ix);
        if (pixel < 128) {
            return distance; // Obstacle détecté
        }

        // Avancer le long du rayon
        x += dx * step;
        y += dy * step;
        distance += step;
    }

    return m_maxRange;
}

int LiDAR::getMapWidth() const
{
    return isValid() ? m_map.cols : 0;
}

int LiDAR::getMapHeight() const
{
    return isValid() ? m_map.rows : 0;
}

bool LiDAR::isObstacle(int x, int y) const
{
    if (!isValid()) return true;
    if (x < 0 || y < 0 || x >= m_map.cols || y >= m_map.rows) return true;
    
    return m_map.at<uchar>(y, x) < 128;
}
