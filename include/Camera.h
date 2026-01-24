/**
  Gestion de la webcam 
 * 
 * Capture les images de la webcam pour la détection
 * des tags ArUco.
 */

#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <string>

class Camera {
public:
    /**
     * Constructeur
     * Index de la caméra (défaut: 0)
     */
    Camera(int cameraIndex = 0);

    /*
      Destructeur - libère la caméra
     */
    ~Camera();

    /* Initialiser la caméra
     * return true si succès
     */
    bool initialize();

    /*Capturer une frame
     * return Image capturée (peut être vide si erreur)
     */
    cv::Mat captureFrame();

    /*
     Vérifier si la caméra est ouverte
     */
    bool isOpened() const;

    /**
     * @brief Calibrer la caméra (simplifié)
     */
    void calibrate();

    /*
     *Charger les paramètres de calibration
     * Paramètre filename Fichier YAML
     */
    bool loadCalibration(const std::string& filename);

    /*
     *Sauvegarder les paramètres de calibration
     * Paramètre filename Fichier YAML
     */
    bool saveCalibration(const std::string& filename);

    /**
     *  Vérifier si calibrée
     */
    bool isCalibrated() const;

    /*
    Obtenir la matrice de la caméra
     */
    cv::Mat getCameraMatrix() const;

    /*
         Obtenir les coefficients de distorsion
     */
    cv::Mat getDistCoeffs() const;

    /*
        Définir la résolution
     */
    void setResolution(int width, int height);

private:
    cv::VideoCapture m_cap;
    int m_cameraIndex;
    bool m_calibrated;

    // Paramètres de calibration
    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoeffs;
};

#endif // CAMERA_H
