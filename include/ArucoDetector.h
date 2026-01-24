/*
Détection des tags ArUco
 * 
 * Utilise OpenCV ArUco pour détecter et identifier
 * les tags dans le flux vidéo.
 * 
 * Tags utilisés:
 * - Tag 0: Contrôle manuel
 * - Tag 1: Exploration automatique
 * - Tag 2: Wall-following
 */

#ifndef ARUCO_DETECTOR_H
#define ARUCO_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>

class ArucoDetector {
public:
    //Constructeur avec choix du dictionnaire
    ArucoDetector(int dictionaryId = cv::aruco::DICT_4X4_50);

    /*
    Détecter les tags dans une image
    return ID du premier tag détecté, ou -1 si aucun
     */
    int detectTag(const cv::Mat& frame);

    /*
    Détecter tous les tags dans une image
    return Vecteur des IDs détectés
     */
    std::vector<int> detectAllTags(const cv::Mat& frame);

    /*
     * Obtenir l'image annotée avec les tags détectés
     * Retourne Image avec rectangles et IDs dessinés
     */
    cv::Mat getAnnotatedFrame(const cv::Mat& frame);

    /**
     * Obtenir les coins des tags détectés
     * Retourne un Vecteur des coins pour chaque tag
     */
    std::vector<std::vector<cv::Point2f>> getLastCorners() const;

    /*
     * Obtenir les IDs des derniers tags détectés
     */
    std::vector<int> getLastIds() const;

    /*
      Définir les paramètres de détection
     */
    void setDetectorParameters(cv::Ptr<cv::aruco::DetectorParameters> params);

    /*
     *  Filtrer les détections erratiques (stabilisation)
     *  Retourne ID stable ou -1
     */
    int getStableDetection(int currentId, int requiredCount = 3);

private:
    cv::Ptr<cv::aruco::Dictionary> m_dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> m_parameters;

    // Dernière détection
    std::vector<int> m_lastIds;
    std::vector<std::vector<cv::Point2f>> m_lastCorners;

    // Pour la stabilisation
    int m_lastStableId;
    int m_consecutiveCount;
    int m_previousId;
};

#endif // ARUCO_DETECTOR_H
