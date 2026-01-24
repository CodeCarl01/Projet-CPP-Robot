/**
 Implémentation du contrôle clavier
 */

#include "KeyboardControl.h"
#include <opencv2/opencv.hpp>
#include <iostream>

KeyboardControl::KeyboardControl(float moveSpeed, float rotateSpeed): m_moveSpeed(moveSpeed), m_rotateSpeed(rotateSpeed)
{
    std::cout << "[KEYBOARD] Contrôle clavier initialisé" << std::endl;
    std::cout << "           Vitesse déplacement: " << m_moveSpeed << " pixels" << std::endl;
    std::cout << "           Vitesse rotation: " << m_rotateSpeed << " rad" << std::endl;
}

KeyboardControl::~KeyboardControl()
{
}

bool KeyboardControl::processKey(int key, Robot& robot)
{
    // Ignorer si aucune touche
    if (key == -1) {
        return false;
    }

    bool validKey = true;

    // Codes des flèches OpenCV (varient selon le système)
    // Linux: 65362 (haut), 65364 (bas), 65361 (gauche), 65363 (droite)
    // Ou: 82, 84, 81, 83 sur certains systèmes
    
    // Convertir les flèches Linux en codes unifiés
    int normalizedKey = key;
    if (key == 65362 || key == 2490368) normalizedKey = 1000; // Haut
    else if (key == 65364 || key == 2621440) normalizedKey = 1001; // Bas
    else if (key == 65361 || key == 2424832) normalizedKey = 1002; // Gauche
    else if (key == 65363 || key == 2555904) normalizedKey = 1003; // Droite

    // Gestion des touches
    switch (normalizedKey) {
        // AVANCER: Z, W, ou flèche haut
        case 'z':
        case 'Z':
        case 'w':
        case 'W':
        case 1000:  // Flèche haut (normalisée)
            robot.moveForward(m_moveSpeed);
            break;

        // RECULER: S ou flèche bas (mais pas 'S' majuscule qui est 83)
        case 's':
        case 1001:  // Flèche bas (normalisée)
            robot.moveForward(-m_moveSpeed);
            break;

        // TOURNER À GAUCHE: Q, A, ou flèche gauche
        case 'q':
        case 'a':
        case 'A':
        case 1002:  // Flèche gauche (normalisée)
            robot.rotate(-m_rotateSpeed);
            break;

        // TOURNER À DROITE: D ou flèche droite
        case 'd':
        case 'D':
        case 1003:  // Flèche droite (normalisée)
            robot.rotate(m_rotateSpeed);
            break;

        // DÉPLACEMENT LATÉRAL GAUCHE: E
        case 'e':
        case 'E':
            robot.move(-m_moveSpeed, 0);
            break;

        default:
            validKey = false;
            break;
    }

    return validKey;
}

int KeyboardControl::update(Robot& robot, int waitTime)
{
    int key = cv::waitKey(waitTime);
    processKey(key, robot);
    return key;
}

void KeyboardControl::setMoveSpeed(float speed)
{
    m_moveSpeed = speed;
}

void KeyboardControl::setRotateSpeed(float speed)
{
    m_rotateSpeed = speed;
}
