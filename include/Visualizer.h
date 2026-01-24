/*
 Gestion de l'affichage graphique 
 * 
 * Affiche trois fenêtres:
 * 1. Carte de référence (statique)
 * 2. Carte d'occupation (dynamique)
 * 3. Flux vidéo avec tags ArUco
 */

#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <opencv2/opencv.hpp>
#include "OccupancyGrid.h"

class Visualizer {
public:
    /**
     Constructeur
     */
    Visualizer();

    /**
     Destructeur
     */
    ~Visualizer();

    /**
     Afficher la carte de référence (Fenêtre 1)
      map Image de la carte
     */
    void displayReferenceMap(const cv::Mat& map);

    /**
     * Afficher la carte d'occupation avec le robot (Fenêtre 2)
     * grid Grille d'occupation
     *  robotX Position X du robot
     * robotY Position Y du robot
     *  robotTheta Orientation du robot
     */
    void displayOccupancyGrid(const OccupancyGrid& grid,
                              float robotX, float robotY, float robotTheta);

    /**
     * Afficher le flux caméra (Fenêtre 3)
     *  frame Image annotée avec les tags ArUco
     */
    void displayCameraFeed(const cv::Mat& frame);

    /**
     *  Mettre à jour toutes les fenêtres
     */
    void updateAll();

    /**
     *  Afficher le robot sur une image
     *  img Image sur laquelle dessiner
     * x Position X
     * y Position Y
     * theta Orientation
     *  scale Échelle de conversion
     */
    void drawRobot(cv::Mat& img, float x, float y, float theta, float scale = 1.0f);

    /**
      Afficher les informations d'état
     */
    void displayStatus(const std::string& behavior, int tagId, int frameCount);

    /*
    Fermer toutes les fenêtres
     */
    void closeAll();

private:
    cv::Mat m_referenceMap;    // Fenêtre 1
    cv::Mat m_occupancyImg;    // Fenêtre 2
    cv::Mat m_cameraFrame;     // Fenêtre 3

    // Noms des fenêtres
    const std::string WINDOW_REF = "Carte de Reference";
    const std::string WINDOW_OCC = "Carte d'Occupation";
    const std::string WINDOW_CAM = "Camera + ArUco";

    // Paramètres du robot pour l'affichage
    const int ROBOT_RADIUS = 8;
    const int DIRECTION_LENGTH = 20;
};

#endif // VISUALIZER_H
