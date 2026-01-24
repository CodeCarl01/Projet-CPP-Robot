/*
 
Implémentation de l'affichage 

 */

#include "Visualizer.h"
#include <iostream>
#include <cmath>

Visualizer::Visualizer()
{
    // Créer les fenêtres
    cv::namedWindow(WINDOW_REF, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(WINDOW_OCC, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(WINDOW_CAM, cv::WINDOW_AUTOSIZE);

    // Positionner les fenêtres
    cv::moveWindow(WINDOW_REF, 50, 50);
    cv::moveWindow(WINDOW_OCC, 550, 50);
    cv::moveWindow(WINDOW_CAM, 50, 500);
}

Visualizer::~Visualizer()
{
    closeAll();
}

void Visualizer::displayReferenceMap(const cv::Mat& map)
{
    if (map.empty()) {
        std::cerr << "[VISUALIZER] Erreur: Carte de référence vide" << std::endl;
        return;
    }

    // Convertir en couleur si nécessaire
    if (map.channels() == 1) {
        cv::cvtColor(map, m_referenceMap, cv::COLOR_GRAY2BGR);
    }
    else {
        m_referenceMap = map.clone();
    }
}

void Visualizer::displayOccupancyGrid(const OccupancyGrid& grid,float robotX, float robotY, float robotTheta)
{
    // Obtenir l'image de la grille
    cv::Mat gridImg = grid.toImage();

    // Convertir en BGR si nécessaire
    if (gridImg.channels() == 1) {
        cv::cvtColor(gridImg, m_occupancyImg, cv::COLOR_GRAY2BGR);
    }
    else {
        m_occupancyImg = gridImg.clone();
    }

    // Dessiner le robot
    float scale = grid.getResolution();
    drawRobot(m_occupancyImg, robotX, robotY, robotTheta, 1.0f / scale);
}

void Visualizer::displayCameraFeed(const cv::Mat& frame)
{
    if (frame.empty()) {
        return;
    }
    m_cameraFrame = frame.clone();
}

void Visualizer::updateAll()
{
    // Afficher les fenêtres si les images sont disponibles
    if (!m_referenceMap.empty()) {
        cv::imshow(WINDOW_REF, m_referenceMap);
    }

    if (!m_occupancyImg.empty()) {
        cv::imshow(WINDOW_OCC, m_occupancyImg);
    }

    if (!m_cameraFrame.empty()) {
        cv::imshow(WINDOW_CAM, m_cameraFrame);
    }

}

void Visualizer::drawRobot(cv::Mat& img, float x, float y, float theta, float scale)
{
    // Convertir les coordonnées monde en pixels
    int px = static_cast<int>(x * scale);
    int py = static_cast<int>(y * scale);

    // Vérifier les limites
    if (px < 0 || py < 0 || px >= img.cols || py >= img.rows) {
        return;
    }

    // Dessiner le corps du robot (cercle bleu)
    cv::circle(img, cv::Point(px, py), ROBOT_RADIUS, cv::Scalar(255, 0, 0), -1);

    // Dessiner le contour (noir)
    cv::circle(img, cv::Point(px, py), ROBOT_RADIUS, cv::Scalar(0, 0, 0), 2);

    // Dessiner la direction (ligne verte)
    int endX = px + static_cast<int>(DIRECTION_LENGTH * std::cos(theta));
    int endY = py + static_cast<int>(DIRECTION_LENGTH * std::sin(theta));
    cv::line(img, cv::Point(px, py), cv::Point(endX, endY), cv::Scalar(0, 255, 0), 3);

    // Pointe de flèche
    cv::circle(img, cv::Point(endX, endY), 4, cv::Scalar(0, 255, 0), -1);
}

void Visualizer::displayStatus(const std::string& behavior, int tagId, int frameCount)
{
    // Ajouter les informations de statut sur la carte d'occupation
    if (m_occupancyImg.empty()) return;

    std::string statusText = "Mode: " + behavior;
    cv::putText(m_occupancyImg, statusText, cv::Point(10, 25),cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);

    std::string tagText = "Tag: " + (tagId >= 0 ? std::to_string(tagId) : "Aucun");
    cv::putText(m_occupancyImg, tagText, cv::Point(10, 50),cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);

    std::string frameText = "Frame: " + std::to_string(frameCount);
    cv::putText(m_occupancyImg, frameText, cv::Point(10, 75),cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
}

void Visualizer::closeAll()
{
    cv::destroyAllWindows();
}
