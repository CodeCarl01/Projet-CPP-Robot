# 🤖 Simulation de Robot en C++

## Projet de Programmation C++ - 2025

**Cours:** Programmation C++

---

## 👥 Équipe

| Membre | Rôle | Tâche |
|--------|------|-------|
| **Carl MENSAH** | Chef de groupe | Tâche 3: Intelligence du Robot / Comportements |
| **Magloire TCHANTEO-TIEKWE** | Développeur | Tâche 1: Gestion du Robot et du LiDAR |
| **Badre DEBBIOUI** | Développeur | Tâche 2: Vision, Caméra et Tags ArUco |
| **Axel BUSSIERE** | Développeur | Tâche 4: Carte d'Occupation + Infrastructure |

---

## 📋 Description

Ce projet est une simulation de robot en C++ qui:
- Se déplace dans un environnement 2D
- Utilise un LiDAR simulé (360 rayons) pour détecter les obstacles
- Utilise la caméra de l'ordinateur pour détecter des tags ArUco
- Construit une carte d'occupation en temps réel
- Change de comportement selon le tag ArUco détecté

---

## 🛠️ Prérequis

### Dépendances requises

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install -y build-essential cmake
sudo apt install -y libopencv-dev

# Vérifier l'installation
pkg-config --modversion opencv4
```

### Versions minimales
- CMake >= 3.10
- OpenCV >= 4.0
- Compilateur C++17 compatible (GCC >= 7, Clang >= 5)

---

## 🚀 Compilation

### Méthode standard

```bash
# Créer le dossier de build
mkdir build
cd build

# Configurer avec CMake
cmake ..

# Compiler
make
```

### Compilation en mode Debug

```bash
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
```

---

## 📦 Structure du Projet

```
RobotSimulation/
├── CMakeLists.txt          # Configuration CMake
├── README.md               # Ce fichier
├── main.cpp                # Programme principal
│
├── include/                # Headers (.h)
│   ├── Robot.h
│   ├── LiDAR.h
│   ├── Camera.h
│   ├── ArucoDetector.h
│   ├── BehaviorManager.h
│   ├── KeyboardControl.h
│   ├── ExplorationAlgorithm.h
│   ├── WallFollower.h
│   ├── PathPlanner.h
│   ├── OccupancyGrid.h
│   └── Visualizer.h
│
├── src/                    # Implémentations (.cpp)
│   ├── Robot.cpp
│   ├── LiDAR.cpp
│   ├── Camera.cpp
│   ├── ArucoDetector.cpp
│   ├── BehaviorManager.cpp
│   ├── KeyboardControl.cpp
│   ├── ExplorationAlgorithm.cpp
│   ├── WallFollower.cpp
│   ├── PathPlanner.cpp
│   ├── OccupancyGrid.cpp
│   └── Visualizer.cpp
│
├── maps/                   # Cartes de simulation
│   ├── map1.png
│   ├── map2.png
│   └── map3.png
│
├── aruco_tags/             # Tags ArUco à imprimer
│   ├── tag_0.png
│   ├── tag_1.png
│   └── tag_2.png
│
└── calibration/            # Paramètres de calibration caméra
    └── camera_params.yml
```

---

## 🎮 Utilisation

### Lancer la simulation

```bash
# Avec la carte par défaut
./robot_simulation

# Avec une carte spécifique
./robot_simulation maps/map2.png
```

### Contrôles

| Touche | Action |
|--------|--------|
| `Z` / `W` / `↑` | Avancer |
| `S` / `↓` | Reculer |
| `Q` / `A` / `←` | Tourner à gauche |
| `D` / `→` | Tourner à droite |
| `R` | Réinitialiser à l'origine |
| `C` | Effacer la carte d'occupation |
| `P` | Afficher la position |
| `ESC` | Quitter |

### Tags ArUco

| Tag ID | Comportement |
|--------|--------------|
| **0** | Contrôle manuel |
| **1** | Exploration automatique |
| **2** | Suivi de murs (wall-following) |

> **Note:** Le comportement persiste même si le tag disparaît. Il change uniquement lors de la détection d'un nouveau tag différent.

---

## 🖼️ Fenêtres d'affichage

Le programme affiche 3 fenêtres:

1. **Carte de Référence** - Carte originale avec position du robot
2. **Carte d'Occupation** - Carte construite en temps réel par le LiDAR
3. **Camera + ArUco** - Flux vidéo avec détection des tags

---

## 📚 Modules

### Tâche 1: Robot + LiDAR (Magloire)

- **Robot**: Gestion de la position, orientation et déplacements
- **LiDAR**: Simulation de 360 rayons avec raycasting

### Tâche 2: Caméra + ArUco (Badre)

- **Camera**: Capture et calibration de la webcam
- **ArucoDetector**: Détection des tags ArUco (dictionnaire 4x4_50)

### Tâche 3: Comportements (Carl)

- **BehaviorManager**: Coordination des comportements
- **KeyboardControl**: Contrôle manuel par clavier
- **ExplorationAlgorithm**: Exploration basée sur les frontières
- **WallFollower**: Suivi de murs (règle main droite)
- **PathPlanner**: Planification de chemin A*

### Tâche 4: Infrastructure (Axel)

- **OccupancyGrid**: Grille d'occupation (FREE/OCCUPIED/UNEXPLORED)
- **Visualizer**: Affichage des 3 fenêtres OpenCV

---

## 🔧 Algorithmes utilisés

### Raycasting (LiDAR)
- Parcours pixel par pixel le long de chaque rayon
- Détection des obstacles (pixels sombres < 128)

### A* (PathPlanner)
- Heuristique: Distance euclidienne
- Voisinage: 8 directions
- Simplification du chemin par ligne de vue

### Exploration par frontières
- Frontière = cellule FREE adjacente à UNEXPLORED
- Sélection de la frontière la plus proche
- Navigation avec A*

### Suivi de murs
- Règle de la main droite
- Maintien d'une distance constante au mur
- Correction proportionnelle de la trajectoire

---

## 📝 Conventions

### Convention LiDAR
- **Rayon 0** = arrière du robot
- **Rayon 90** = gauche du robot
- **Rayon 180** = avant du robot (aligné avec la caméra)
- **Rayon 270** = droite du robot

### Détection de collision
- Le robot dispose d'une détection de collision basée sur les données LiDAR
- Les mouvements vers des obstacles sont automatiquement bloqués
- Distance de sécurité : 10 pixels
- Arc de vérification : ±30° autour de la direction du mouvement

### Position de départ
- Position par défaut : (100, 100)
- Détection automatique si la position est dans un obstacle
- Rayon de sécurité requis : 15 pixels libres

### Convention de couleurs (OccupancyGrid)
- **Gris (128, 128, 128)**: UNEXPLORED
- **Blanc (255, 255, 255)**: FREE
- **Noir (0, 0, 0)**: OCCUPIED

### Unités
- Positions: pixels
- Angles: radians
- Distances LiDAR: pixels

---

## ⚠️ Limitations connues

1. La caméra peut ne pas fonctionner si aucune webcam n'est connectée
2. La détection ArUco peut être instable avec un mauvais éclairage
3. L'exploration peut être lente sur de grandes cartes
4. La détection de collision est basée sur le LiDAR (distance de sécurité configurable)

---

## 📖 Sources et Références

- [OpenCV ArUco Tutorial](https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html)
- [OpenCV Camera Calibration](https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html)
- [A* Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm)
- [Frontier-based Exploration](https://ieeexplore.ieee.org/document/613851)
- [Wall Following Robot](https://en.wikipedia.org/wiki/Maze_solving_algorithm)

---

