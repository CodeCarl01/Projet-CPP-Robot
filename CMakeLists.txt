# =========================================================================
# CMakeLists.txt - Projet de Simulation de Robot
# =========================================================================
# Auteurs: Carl (Chef), Magloire, Badre, Axel
# Date: Novembre 2025
# Description: Configuration CMake pour la compilation du projet
# =========================================================================

cmake_minimum_required(VERSION 3.10)

# Nom du projet
project(RobotSimulation VERSION 1.0 LANGUAGES CXX)

# Standard C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Mode de compilation (Debug par défaut)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Options de compilation
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# =========================================================================
# DÉPENDANCES
# =========================================================================

# Trouver OpenCV (requis)
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    message(STATUS "OpenCV version: ${OpenCV_VERSION}")
    message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "OpenCV non trouvé! Installez-le avec: sudo apt install libopencv-dev")
endif()

# =========================================================================
# CONFIGURATION DES FICHIERS
# =========================================================================

# Inclure les headers
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include)

# Lister tous les fichiers source
set(SOURCES
    src/Robot.cpp
    src/LiDAR.cpp
    src/Camera.cpp
    src/ArucoDetector.cpp
    src/OccupancyGrid.cpp
    src/Visualizer.cpp
    src/KeyboardControl.cpp
    src/PathPlanner.cpp
    src/WallFollower.cpp
    src/ExplorationAlgorithm.cpp
    src/BehaviorManager.cpp
    main.cpp
)

# Lister les headers (pour l'IDE)
set(HEADERS
    include/Robot.h
    include/LiDAR.h
    include/Camera.h
    include/ArucoDetector.h
    include/OccupancyGrid.h
    include/Visualizer.h
    include/KeyboardControl.h
    include/PathPlanner.h
    include/WallFollower.h
    include/ExplorationAlgorithm.h
    include/BehaviorManager.h
)

# =========================================================================
# CRÉATION DE L'EXÉCUTABLE
# =========================================================================

# Créer l'exécutable principal
add_executable(robot_simulation ${SOURCES} ${HEADERS})

# Lier les bibliothèques
target_link_libraries(robot_simulation ${OpenCV_LIBS})

# =========================================================================
# CONFIGURATION DE L'INSTALLATION
# =========================================================================

# Copier les ressources dans le répertoire de build
file(COPY ${CMAKE_SOURCE_DIR}/maps DESTINATION ${CMAKE_BINARY_DIR})
file(COPY ${CMAKE_SOURCE_DIR}/aruco_tags DESTINATION ${CMAKE_BINARY_DIR})

# Créer le dossier calibration s'il n'existe pas
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/calibration)

# =========================================================================
# OPTIONS SUPPLÉMENTAIRES
# =========================================================================

# Option pour activer les warnings stricts
option(ENABLE_STRICT_WARNINGS "Activer les warnings stricts" OFF)
if(ENABLE_STRICT_WARNINGS)
    target_compile_options(robot_simulation PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

# Afficher les informations de configuration
message(STATUS "=========================================")
message(STATUS "Configuration du projet:")
message(STATUS "  - Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  - C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  - OpenCV: ${OpenCV_VERSION}")
message(STATUS "=========================================")

# =========================================================================
# CIBLES PERSONNALISÉES
# =========================================================================

# Cible pour nettoyer les fichiers générés
add_custom_target(clean-all
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/CMakeFiles
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/CMakeCache.txt
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/cmake_install.cmake
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/Makefile
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/robot_simulation
    COMMENT "Nettoyage complet du répertoire de build"
)

# Cible pour exécuter le programme
add_custom_target(run
    COMMAND robot_simulation
    DEPENDS robot_simulation
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Exécution de la simulation"
)
