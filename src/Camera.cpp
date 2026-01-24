/*
    Implémentation de la gestion de la webcam
 */

#include "Camera.h"
#include <iostream>

Camera::Camera(int cameraIndex)
    : m_cameraIndex(cameraIndex),
      m_calibrated(false)
{
    // Initialiser les matrices de calibration par défaut
    m_cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    m_distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
}

Camera::~Camera()
{
    if (m_cap.isOpened()) {
        m_cap.release();
    }
}

bool Camera::initialize()
{
    // Ouvrir la caméra
    m_cap.open(m_cameraIndex);
    
    if (!m_cap.isOpened()) {
        std::cerr << "[CAMERA] Erreur: Impossible d'ouvrir la caméra "<< m_cameraIndex << std::endl;
        return false;
    }

    // Configuration par défaut
    m_cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    m_cap.set(cv::CAP_PROP_FPS, 30);

    std::cout << "[CAMERA] Caméra initialisée avec succès" << std::endl;
    return true;
}

cv::Mat Camera::captureFrame()
{
    cv::Mat frame;
    
    if (!m_cap.isOpened()) {
        return frame; // Retourne une image vide
    }

    m_cap >> frame;

    // Si calibrée, corriger la distorsion
    if (m_calibrated && !frame.empty()) {
        cv::Mat undistorted;
        cv::undistort(frame, undistorted, m_cameraMatrix, m_distCoeffs);
        return undistorted;
    }

    return frame;
}

bool Camera::isOpened() const
{
    return m_cap.isOpened();
}

void Camera::calibrate()
{
    
    // Matrice de caméra approximative pour une webcam 640x480
    m_cameraMatrix = (cv::Mat_<double>(3, 3) <<
        600, 0, 320,
        0, 600, 240,
        0, 0, 1);

    // Pas de distorsion par défaut
    m_distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

    m_calibrated = true;
    std::cout << "[CAMERA] Calibration par défaut appliquée" << std::endl;
}

bool Camera::loadCalibration(const std::string& filename)
{
    try {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cerr << "[CAMERA] Erreur: Impossible d'ouvrir " << filename << std::endl;
            return false;
        }

        fs["camera_matrix"] >> m_cameraMatrix;
        fs["distortion_coefficients"] >> m_distCoeffs;
        fs.release();

        m_calibrated = true;
        std::cout << "[CAMERA] Paramètres de calibration chargés depuis " << filename << std::endl;
        return true;
    }
    catch (const cv::Exception& e) {
        std::cerr << "[CAMERA] Erreur de chargement: " << e.what() << std::endl;
        return false;
    }
}

bool Camera::saveCalibration(const std::string& filename)
{
    try {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            std::cerr << "[CAMERA] Erreur: Impossible de créer " << filename << std::endl;
            return false;
        }

        fs << "camera_matrix" << m_cameraMatrix;
        fs << "distortion_coefficients" << m_distCoeffs;
        fs.release();

        std::cout << "[CAMERA] Paramètres sauvegardés dans " << filename << std::endl;
        return true;
    }
    catch (const cv::Exception& e) {
        std::cerr << "[CAMERA] Erreur de sauvegarde: " << e.what() << std::endl;
        return false;
    }
}

bool Camera::isCalibrated() const
{
    return m_calibrated;
}

cv::Mat Camera::getCameraMatrix() const
{
    return m_cameraMatrix.clone();
}

cv::Mat Camera::getDistCoeffs() const
{
    return m_distCoeffs.clone();
}

void Camera::setResolution(int width, int height)
{
    if (m_cap.isOpened()) {
        m_cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    }
}
