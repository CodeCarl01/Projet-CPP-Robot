/*
 Contrôle manuel du robot par clavier (Tâche 3 - Carl)
 * 
 * Permet de déplacer le robot manuellement avec les touches:
 * - Z/W ou ↑: Avancer
 * - S ou ↓: Reculer
 * - Q/A ou ←: Tourner à gauche
 * - D ou →: Tourner à droite
 */

#ifndef KEYBOARD_CONTROL_H
#define KEYBOARD_CONTROL_H

#include "Robot.h"

class KeyboardControl {
public:
    /**
     * Constructeur
     * moveSpeed Vitesse de déplacement (pixels par commande)
     * rotateSpeed Vitesse de rotation (radians par commande)
     */
    KeyboardControl(float moveSpeed = 5.0f, float rotateSpeed = 0.1f);

    /**
     Destructeur
     */
    ~KeyboardControl();

    /**
     Traiter une touche et déplacer le robot
     * key Code de la touche (de cv::waitKey)
     * robot Référence au robot
     * return true si une commande valide a été traitée
     */
    bool processKey(int key, Robot& robot);

    /**
     * Version simplifiée qui appelle cv::waitKey
     * robot Référence au robot
     * waitTime Temps d'attente (ms)
     * return Code de la touche pressée
     */
    int update(Robot& robot, int waitTime = 1);

    /**
     * Définir la vitesse de déplacement
     */
    void setMoveSpeed(float speed);

    /**
     * Définir la vitesse de rotation
     */
    void setRotateSpeed(float speed);

    /**
      Obtenir la vitesse de déplacement
     */
    float getMoveSpeed() const { return m_moveSpeed; }

    /*
    Obtenir la vitesse de rotation
     */
    float getRotateSpeed() const { return m_rotateSpeed; }

private:
    float m_moveSpeed;    // Vitesse de déplacement
    float m_rotateSpeed;  // Vitesse de rotation
};

#endif // KEYBOARD_CONTROL_H
