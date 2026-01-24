/**
  Grille d'occupation 
 * Représente l'environnement sous forme de grille 2D
 * avec trois états possibles par cellule:
 * - UNEXPLORED (gris)
 * - FREE (blanc)
 * - OCCUPIED (noir)
 */

#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <opencv2/opencv.hpp>
#include <vector>

class OccupancyGrid {
public:
    /*
     *États possibles d'une cellule
     */
    enum CellState {
        UNEXPLORED = 0,  // Gris (128, 128, 128)
        FREE = 1,        // Blanc (255, 255, 255)
        OCCUPIED = 2     // Noir (0, 0, 0)
    };

    /**
     * Constructeur
     *  width Largeur de la grille (en cellules)
     *  height Hauteur de la grille (en cellules)
     *  resolution Résolution (pixels par cellule, ex: 1.0 = 1 pixel = 1 cellule)
     */
    OccupancyGrid(int width, int height, float resolution = 1.0f);

    /**
     *  Mettre à jour la grille avec les données LiDAR
     *  robotX Position X du robot (en pixels)
     * robotY Position Y du robot (en pixels)
     * robotTheta Orientation du robot (en radians)
     * lidarData 360 mesures de distance
     */
    void updateFromLidar(float robotX, float robotY, float robotTheta, const std::vector<float>& lidarData);

    /**
     * Obtenir l'état d'une cellule
     * x Coordonnée X de la cellule
     * y Coordonnée Y de la cellule
     * return État de la cellule
     */
    CellState getCellState(int x, int y) const;

    /*
    Définir l'état d'une cellule
     */
    void setCellState(int x, int y, CellState state);

    /*
     Convertir la grille en image OpenCV
     return Image BGR
     */
    cv::Mat toImage() const;

    /**
     * Réinitialiser la grille (tout en UNEXPLORED)
     */
    void clear();

    /**
     * Obtenir les dimensions
     */
    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }
    float getResolution() const { return m_resolution; }

    /**
     * Vérifier si une coordonnée est dans les limites
     */
    bool isInBounds(int x, int y) const;

    /**
     * Convertir coordonnées monde -> grille
     */
    int worldToGridX(float worldX) const;
    int worldToGridY(float worldY) const;

    /**
     *  Convertir coordonnées grille -> monde
     */
    float gridToWorldX(int gridX) const;
    float gridToWorldY(int gridY) const;

private:
    int m_width;          // Largeur en cellules
    int m_height;         // Hauteur en cellules
    float m_resolution;   // Pixels par cellule

    std::vector<CellState> m_grid;  // Données de la grille

    /*
    Obtenir l'index dans le vecteur
     */
    inline int idx(int x, int y) const { return y * m_width + x; }

    /*
    Tracer une ligne libre entre deux points (Bresenham)
     */
    void traceLine(int x0, int y0, int x1, int y1);
};

#endif // OCCUPANCY_GRID_H
