/**
 Programme principal de simulation de robot
 * 
 * Ce programme intègre tous les modules développés par l'équipe :
 * - Robot + LiDAR (Magloire)
 * - Caméra + ArUco (Badre)
 * - Comportements (Carl)
 * - OccupancyGrid + Visualisation (Axel)
 * 
 * Instructions:
 * - Montrez un tag ArUco devant la caméra pour changer de comportement
 * - Tag 0: Contrôle manuel (ZQSD/WASD ou flèches)
 * - Tag 1: Exploration automatique
 * - Tag 2: Suivi de murs (wall-following)
 * - ESC: Quitter
 * - R: Réinitialiser le robot à l'origine
 * - C: Effacer la carte d'occupation
 * - P: Afficher la position actuelle
 */

#include <iostream>
#include <opencv2/opencv.hpp>

// Modules de Magloire
#include "Robot.h"
#include "LiDAR.h"

// Modules de Badre
#include "Camera.h"
#include "ArucoDetector.h"

// Modules de Carl
#include "BehaviorManager.h"
#include "KeyboardControl.h"

// Modules d'Axel
#include "OccupancyGrid.h"
#include "Visualizer.h"

// ============ CONFIGURATION ============
const std::string MAP_PATH = "maps/map1.png";  // Chemin vers la carte
const float LIDAR_MAX_RANGE = 500.0f;          // Portée maximale du LiDAR (pixels)

// Position initiale du robot (dans une zone libre)
const float ROBOT_START_X = 100.0f;
const float ROBOT_START_Y = 100.0f;
const float ROBOT_START_THETA = 0.0f;  // Orienté vers la droite (Est)

void printUsage()
{
    std::cout << std::endl;
    std::cout << "╔══════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║       SIMULATION DE ROBOT - Projet C++ 2025              ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║  CONTRÔLES:                                              ║" << std::endl;
    std::cout << "║    Z/W/↑ : Avancer                                       ║" << std::endl;
    std::cout << "║    S/↓   : Reculer                                       ║" << std::endl;
    std::cout << "║    Q/A/← : Tourner à gauche                              ║" << std::endl;
    std::cout << "║    D/→   : Tourner à droite                              ║" << std::endl;
    std::cout << "║    R     : Réinitialiser à l'origine                     ║" << std::endl;
    std::cout << "║    C     : Effacer la carte d'occupation                 ║" << std::endl;
    std::cout << "║    P     : Afficher la position                          ║" << std::endl;
    std::cout << "║    ESC   : Quitter                                       ║" << std::endl;
    std::cout << "║                                                          ║" << std::endl;
    std::cout << "║  TAGS ARUCO:                                             ║" << std::endl;
    std::cout << "║    Tag 0 : Mode manuel                                   ║" << std::endl;
    std::cout << "║    Tag 1 : Exploration automatique                       ║" << std::endl;
    std::cout << "║    Tag 2 : Suivi de murs                                 ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;
    std::cout << std::endl;
}

int main(int argc, char** argv)
{
    std::cout << "==================================================" << std::endl;
    std::cout << "   SIMULATION DE ROBOT - Projet C++ 2025" << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << std::endl;

    // Permettre de spécifier la carte en argument
    std::string mapPath = MAP_PATH;
    if (argc > 1) {
        mapPath = argv[1];
        std::cout << "[INIT] Utilisation de la carte: " << mapPath << std::endl;
    }

    // ============ INITIALISATION DES MODULES ============

    std::cout << "[INIT] Chargement de la carte de référence..." << std::endl;
    cv::Mat referenceMap = cv::imread(mapPath, cv::IMREAD_GRAYSCALE);
    if (referenceMap.empty()) {
        std::cerr << "[ERREUR] Impossible de charger la carte: " << mapPath << std::endl;
        std::cerr << "[INFO] Assurez-vous que le fichier existe." << std::endl;
        std::cerr << "[INFO] Usage: " << argv[0] << " [chemin_carte.png]" << std::endl;
        return -1;
    }
    std::cout << "[INIT] ✓ Carte chargée (" << referenceMap.cols << "x"<< referenceMap.rows << " pixels)" << std::endl;

    // Calculer la position initiale (centre de la carte si pas spécifié)
    float startX = (ROBOT_START_X > 0) ? ROBOT_START_X : referenceMap.cols / 2.0f;
    float startY = (ROBOT_START_Y > 0) ? ROBOT_START_Y : referenceMap.rows / 2.0f;

    // Vérifier si la position de départ est valide (dans une zone libre)
    auto isPositionValid = [&referenceMap](int x, int y, int radius = 15) -> bool {
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                int px = x + dx;
                int py = y + dy;
                if (px < 0 || py < 0 || px >= referenceMap.cols || py >= referenceMap.rows) {
                    return false;
                }
                if (referenceMap.at<uchar>(py, px) < 128) {
                    return false;
                }
            }
        }
        return true;
    };

    // Si la position par défaut n'est pas valide, en chercher une autre
    if (!isPositionValid(static_cast<int>(startX), static_cast<int>(startY))) {
        std::cout << "[INIT] Position par défaut invalide, recherche d'une position libre..." << std::endl;
        bool found = false;
        for (int y = 50; y < referenceMap.rows - 50 && !found; y += 20) {
            for (int x = 50; x < referenceMap.cols - 50 && !found; x += 20) {
                if (isPositionValid(x, y)) {
                    startX = static_cast<float>(x);
                    startY = static_cast<float>(y);
                    found = true;
                    std::cout << "[INIT] Position libre trouvée à (" << startX << ", " << startY << ")" << std::endl;
                }
            }
        }
        if (!found) {
            std::cerr << "[ERREUR] Impossible de trouver une position de départ valide!" << std::endl;
            return -1;
        }
    }

    // Module Robot 
    std::cout << "[INIT] Initialisation du robot..." << std::endl;
    Robot robot(startX, startY, ROBOT_START_THETA);
    robot.setOrigin(startX, startY, ROBOT_START_THETA);
    robot.setCollisionDetection(true);
    robot.setMinSafeDistance(10.0f);  // Distance minimale de sécurité
    std::cout << "[INIT] ✓ Robot initialisé à (" << robot.getX() << ", "<< robot.getY() << ", " << robot.getTheta() << " rad)" << std::endl;
    std::cout << "[INIT] ✓ Détection de collision activée" << std::endl;

    // Module LiDAR 
    std::cout << "[INIT] Initialisation du LiDAR..." << std::endl;
    LiDAR lidar(mapPath, LIDAR_MAX_RANGE);
    if (!lidar.isValid()) {
        std::cerr << "[ERREUR] Échec de l'initialisation du LiDAR" << std::endl;
        return -1;
    }
    std::cout << "[INIT] ✓ LiDAR initialisé (360 rayons, portée: "<< LIDAR_MAX_RANGE << " px)" << std::endl;

    // Module Caméra 
    std::cout << "[INIT] Initialisation de la caméra..." << std::endl;
    Camera camera;
    bool cameraAvailable = camera.initialize();
    if (!cameraAvailable) {
        std::cout << "[WARN] Caméra non disponible - Mode sans caméra activé" << std::endl;
    } else {
        std::cout << "[INIT] ✓ Caméra initialisée" << std::endl;
    }

    // Module ArUco 
    std::cout << "[INIT] Initialisation du détecteur ArUco..." << std::endl;
    ArucoDetector arucoDetector;
    std::cout << "[INIT] ✓ Détecteur ArUco initialisé (DICT_4X4_50)" << std::endl;

    // Module OccupancyGrid 
    std::cout << "[INIT] Initialisation de la carte d'occupation..." << std::endl;
    OccupancyGrid occupancyGrid(referenceMap.cols, referenceMap.rows, 1.0f);
    std::cout << "[INIT] ✓ Carte d'occupation initialisée ("<< occupancyGrid.getWidth() << "x" << occupancyGrid.getHeight()<< " cellules)" << std::endl;

    // Module Visualizer
    std::cout << "[INIT] Initialisation de l'affichage..." << std::endl;
    Visualizer visualizer;
    std::cout << "[INIT] ✓ Visualiseur initialisé" << std::endl;

    // Module BehaviorManager
    std::cout << "[INIT] Initialisation du gestionnaire de comportements..." << std::endl;
    BehaviorManager behaviorManager(BehaviorManager::MANUAL);
    std::cout << "[INIT] ✓ Gestionnaire de comportements initialisé" << std::endl;

    printUsage();

    // ============ VARIABLES D'ÉTAT ============
    int currentTagID = -1;
    int previousTagID = -1;
    bool running = true;
    int frameCount = 0;

    // ============ BOUCLE PRINCIPALE ============
    std::cout << "[MAIN] Démarrage de la boucle principale..." << std::endl;

    while (running) {
        frameCount++;

        // ============ ÉTAPE 1: CAPTURE CAMÉRA + DÉTECTION ARUCO ============
        cv::Mat cameraFrame;
        cv::Mat annotatedFrame;

        if (cameraAvailable) {
            cameraFrame = camera.captureFrame();
            if (!cameraFrame.empty()) {
                // Détecter les tags avec stabilisation
                int rawTagID = arucoDetector.detectTag(cameraFrame);
                currentTagID = arucoDetector.getStableDetection(rawTagID, 3);
                annotatedFrame = arucoDetector.getAnnotatedFrame(cameraFrame);
            }
        } else {
            // Créer une image vide si pas de caméra
            annotatedFrame = cv::Mat::zeros(480, 640, CV_8UC3);
            cv::putText(annotatedFrame, "Camera non disponible", cv::Point(150, 240),
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            cv::putText(annotatedFrame, "Utilisez les touches pour controler", cv::Point(100, 280),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 1);
        }

        // ============ ÉTAPE 2: GESTION DES COMPORTEMENTS ============
        if (currentTagID != -1 && currentTagID != previousTagID) {
            std::cout << "[ARUCO] Tag détecté: ID = " << currentTagID << std::endl;
            behaviorManager.updateBehavior(currentTagID);
            previousTagID = currentTagID;
        }

        // ============ ÉTAPE 3: ACQUISITION DES DONNÉES LIDAR ============
        std::vector<float> lidarData = lidar.getMeasurements(
            robot.getX(),
            robot.getY(),
            robot.getTheta()
        );

        // Mettre à jour la référence LiDAR pour la détection de collision
        robot.setLidarReference(&lidarData);

        // ============ ÉTAPE 4: MISE À JOUR DE LA CARTE D'OCCUPATION ============
        occupancyGrid.updateFromLidar(
            robot.getX(),
            robot.getY(),
            robot.getTheta(),
            lidarData
        );

        // ============ ÉTAPE 5: GESTION DES ENTRÉES CLAVIER ============
        int key = cv::waitKey(30);  // 30ms = ~33 FPS

        // ESC pour quitter
        if (key == 27) {
            std::cout << "[MAIN] Touche ESC détectée, arrêt du programme..." << std::endl;
            running = false;
            continue;
        }

        // R pour réinitialiser
        if (key == 'r' || key == 'R') {
            std::cout << "[MAIN] Réinitialisation du robot à l'origine..." << std::endl;
            robot.resetToOrigin();
            behaviorManager.setBehavior(BehaviorManager::MANUAL);
        }

        // C pour effacer la carte
        if (key == 'c' || key == 'C') {
            std::cout << "[MAIN] Effacement de la carte d'occupation..." << std::endl;
            occupancyGrid.clear();
        }

        // P pour afficher la position
        if (key == 'p' || key == 'P') {
            std::cout << "[MAIN] Position: (" << robot.getX() << ", "
                      << robot.getY() << ", " << robot.getTheta() << " rad)" << std::endl;
            std::cout << "[MAIN] Mode: " << behaviorManager.getCurrentBehaviorName() << std::endl;
        }

        // ============ ÉTAPE 6: EXÉCUTION DU COMPORTEMENT ============
        behaviorManager.executeBehavior(robot, lidarData, occupancyGrid, key);

        // ============ ÉTAPE 7: AFFICHAGE ============
        // Fenêtre 1: Carte de référence avec robot
        cv::Mat refMapColor;
        cv::cvtColor(referenceMap, refMapColor, cv::COLOR_GRAY2BGR);
        visualizer.drawRobot(refMapColor, robot.getX(), robot.getY(), robot.getTheta());
        visualizer.displayReferenceMap(refMapColor);

        // Fenêtre 2: Carte d'occupation
        visualizer.displayOccupancyGrid(occupancyGrid, robot.getX(), robot.getY(), robot.getTheta());
        visualizer.displayStatus(behaviorManager.getCurrentBehaviorName(), currentTagID, frameCount);

        // Fenêtre 3: Flux vidéo
        visualizer.displayCameraFeed(annotatedFrame);

        // Mettre à jour les fenêtres
        visualizer.updateAll();

        // Afficher les stats toutes les 30 frames
        if (frameCount % 90 == 0) {
            std::cout << "[MAIN] Frame " << frameCount
                      << " | Pos: (" << static_cast<int>(robot.getX()) << ", "
                      << static_cast<int>(robot.getY()) << ")"
                      << " | Mode: " << behaviorManager.getCurrentBehaviorName()
                      << std::endl;
        }
    }

    // ============ NETTOYAGE ET FERMETURE ============
    std::cout << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << "   FERMETURE DU PROGRAMME" << std::endl;
    std::cout << "==================================================" << std::endl;

    std::cout << "[MAIN] Statistiques de la session:" << std::endl;
    std::cout << "  • Nombre de frames: " << frameCount << std::endl;
    std::cout << "  • Position finale: (" << robot.getX() << ", "
              << robot.getY() << ")" << std::endl;

    visualizer.closeAll();

    std::cout << "[MAIN] ✓ Programme terminé avec succès." << std::endl;
    return 0;
}
