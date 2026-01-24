/*
    Implémentation de la grille d'occupation
 */

#include "OccupancyGrid.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

OccupancyGrid::OccupancyGrid(int width, int height, float resolution)
    : m_width(width),
      m_height(height),
      m_resolution(resolution)
{
    // Initialiser toutes les cellules comme UNEXPLORED
    m_grid.resize(m_width * m_height, UNEXPLORED);
}

void OccupancyGrid::updateFromLidar(float robotX, float robotY, float robotTheta, const std::vector<float>& lidarData)
{
    // Convertir la position du robot en coordonnées grille
    int robotGx = worldToGridX(robotX);
    int robotGy = worldToGridY(robotY);

    // Marquer la position du robot et autour comme libre
    for (int dy = -3; dy <= 3; ++dy) {
        for (int dx = -3; dx <= 3; ++dx) {
            if (isInBounds(robotGx + dx, robotGy + dy)) {
                m_grid[idx(robotGx + dx, robotGy + dy)] = FREE;
            }
        }
    }

    // Traiter chaque rayon LiDAR
    for (size_t i = 0; i < lidarData.size() && i < 360; ++i) {
        float distance = lidarData[i];
        
        if (distance <= 0) continue;

        float rayAngle = robotTheta + static_cast<float>(static_cast<int>(i) - 180) * (M_PI / 180.0f);

        // Calculer le point d'impact en coordonnées monde
        float endX = robotX + distance * std::cos(rayAngle);
        float endY = robotY + distance * std::sin(rayAngle);

        // Convertir en coordonnées grille
        int endGx = worldToGridX(endX);
        int endGy = worldToGridY(endY);

        // Tracer une ligne libre entre le robot et le point d'impact
        traceLine(robotGx, robotGy, endGx, endGy);

        // Marquer le point d'impact comme occupé (si la distance < max range)
        if (distance < 480.0f && isInBounds(endGx, endGy)) {
            m_grid[idx(endGx, endGy)] = OCCUPIED;
            // Épaissir légèrement les obstacles pour la visualisation
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (isInBounds(endGx + dx, endGy + dy)) {
                        m_grid[idx(endGx + dx, endGy + dy)] = OCCUPIED;
                    }
                }
            }
        }
    }
}

OccupancyGrid::CellState OccupancyGrid::getCellState(int x, int y) const
{
    if (!isInBounds(x, y)) {
        return UNEXPLORED;
    }
    return m_grid[idx(x, y)];
}

void OccupancyGrid::setCellState(int x, int y, CellState state)
{
    if (isInBounds(x, y)) {
        m_grid[idx(x, y)] = state;
    }
}

cv::Mat OccupancyGrid::toImage() const
{
    cv::Mat img(m_height, m_width, CV_8UC3);

    for (int y = 0; y < m_height; ++y) {
        for (int x = 0; x < m_width; ++x) {
            CellState state = m_grid[idx(x, y)];
            cv::Vec3b color;

            switch (state) {
                case UNEXPLORED:
                    color = cv::Vec3b(128, 128, 128);  // Gris
                    break;
                case FREE:
                    color = cv::Vec3b(255, 255, 255);  // Blanc
                    break;
                case OCCUPIED:
                    color = cv::Vec3b(0, 0, 0);        // Noir
                    break;
            }

            img.at<cv::Vec3b>(y, x) = color;
        }
    }

    return img;
}

void OccupancyGrid::clear()
{
    std::fill(m_grid.begin(), m_grid.end(), UNEXPLORED);
}

bool OccupancyGrid::isInBounds(int x, int y) const
{
    return (x >= 0 && x < m_width && y >= 0 && y < m_height);
}

int OccupancyGrid::worldToGridX(float worldX) const
{
    return static_cast<int>(worldX / m_resolution);
}

int OccupancyGrid::worldToGridY(float worldY) const
{
    return static_cast<int>(worldY / m_resolution);
}

float OccupancyGrid::gridToWorldX(int gridX) const
{
    return static_cast<float>(gridX) * m_resolution;
}

float OccupancyGrid::gridToWorldY(int gridY) const
{
    return static_cast<float>(gridY) * m_resolution;
}

void OccupancyGrid::traceLine(int x0, int y0, int x1, int y1)
{
    // Algorithme de Bresenham pour tracer une ligne
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int x = x0;
    int y = y0;

    while (true) {
        // Marquer comme libre (sauf le point final qui sera marqué occupé)
        if (isInBounds(x, y)) {
            // Ne pas écraser un obstacle existant
            if (m_grid[idx(x, y)] != OCCUPIED) {
                m_grid[idx(x, y)] = FREE;
            }
        }

        // Arrivé au point final?
        if (x == x1 && y == y1) break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}
