/*

 Implémentation de la détection des tags ArUco
 
 */

#include "ArucoDetector.h"
#include <iostream>

ArucoDetector::ArucoDetector(int dictionaryId)
    : m_lastStableId(-1),
      m_consecutiveCount(0),
      m_previousId(-1)
{
    // Créer le dictionnaire ArUco
    m_dictionary = cv::aruco::getPredefinedDictionary(dictionaryId);
    
    // Créer les paramètres de détection avec valeurs par défaut
    m_parameters = cv::aruco::DetectorParameters::create();
    
    // Optimiser les paramètres pour une meilleure détection
    m_parameters->adaptiveThreshConstant = 7;
    m_parameters->adaptiveThreshWinSizeMin = 3;
    m_parameters->adaptiveThreshWinSizeMax = 23;
    m_parameters->adaptiveThreshWinSizeStep = 10;
    m_parameters->minMarkerPerimeterRate = 0.03;
    m_parameters->maxMarkerPerimeterRate = 4.0;
    m_parameters->polygonalApproxAccuracyRate = 0.05;
    m_parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
}

int ArucoDetector::detectTag(const cv::Mat& frame)
{
    if (frame.empty()) {
        return -1;
    }

    // Détecter les marqueurs
    cv::aruco::detectMarkers(frame, m_dictionary, m_lastCorners, m_lastIds, m_parameters);

    // Retourner le premier ID détecté, ou -1 si aucun
    if (!m_lastIds.empty()) {
        return m_lastIds[0];
    }
    
    return -1;
}

std::vector<int> ArucoDetector::detectAllTags(const cv::Mat& frame)
{
    if (frame.empty()) {
        return std::vector<int>();
    }

    // Détecter les marqueurs
    cv::aruco::detectMarkers(frame, m_dictionary, m_lastCorners, m_lastIds, m_parameters);

    return m_lastIds;
}

cv::Mat ArucoDetector::getAnnotatedFrame(const cv::Mat& frame)
{
    if (frame.empty()) {
        return frame;
    }

    // Cloner l'image pour ne pas modifier l'originale
    cv::Mat annotated = frame.clone();

    // Détecter les marqueurs
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(annotated, m_dictionary, corners, ids, m_parameters);

    // Dessiner les marqueurs détectés
    if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(annotated, corners, ids);

        // Ajouter du texte avec l'ID du premier tag
        std::string text = "Tag ID: " + std::to_string(ids[0]);
        
        // Calculer la position du texte (au-dessus du premier tag)
        if (!corners.empty() && !corners[0].empty()) {
            cv::Point textPos(static_cast<int>(corners[0][0].x), static_cast<int>(corners[0][0].y) - 10);
            cv::putText(annotated, text, textPos, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        }

        // Afficher le comportement correspondant
        std::string behavior;
        switch (ids[0]) {
            case 0: behavior = "MANUEL"; break;
            case 1: behavior = "EXPLORATION"; break;
            case 2: behavior = "WALL-FOLLOW"; break;
            default: behavior = "INCONNU"; break;
        }
        
        cv::putText(annotated, "Mode: " + behavior, cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    }
    else {
        // Aucun tag détecté
        cv::putText(annotated, "Aucun tag detecte", cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }

    // Stocker les résultats
    m_lastIds = ids;
    m_lastCorners = corners;

    return annotated;
}

std::vector<std::vector<cv::Point2f>> ArucoDetector::getLastCorners() const
{
    return m_lastCorners;
}

std::vector<int> ArucoDetector::getLastIds() const
{
    return m_lastIds;
}

void ArucoDetector::setDetectorParameters(cv::Ptr<cv::aruco::DetectorParameters> params)
{
    m_parameters = params;
}

int ArucoDetector::getStableDetection(int currentId, int requiredCount)
{
    // Si même ID que précédemment, incrémenter le compteur
    if (currentId == m_previousId && currentId != -1) {
        m_consecutiveCount++;
    }
    else {
        // Reset du compteur
        m_consecutiveCount = 1;
        m_previousId = currentId;
    }

    // Si suffisamment de détections consécutives
    if (m_consecutiveCount >= requiredCount && currentId != -1) {
        m_lastStableId = currentId;
    }

    return m_lastStableId;
}
