/**
 * @file minimal_vio_example.cpp
 * @brief Exemple minimal d'intégration OpenVINS en mode ROS-free
 * @author Projet IMT Atlantique × IRT b-com
 * 
 * Ce fichier montre comment :
 * 1. Initialiser VioManager avec une configuration
 * 2. Alimenter le système avec des données IMU et caméra
 * 3. Récupérer la pose estimée pour transmission à Overview
 */

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "utils/sensor_data.h"

#include <iostream>
#include <Eigen/Dense>

int main(int argc, char** argv) {
    
    std::cout << "=== Exemple minimal OpenVINS ROS-free ===" << std::endl;
    
    // ========================================================================
    // ÉTAPE 1 : Configuration du système VIO
    // ========================================================================
    
    ov_msckf::VioManagerOptions params;
    
    // Charger depuis un fichier YAML (recommandé)
    if (argc > 1) {
        params = ov_msckf::VioManagerOptions();
        // TODO: Implémenter parser YAML ou utiliser ROS parser
        std::cout << "[INFO] Configuration chargée depuis: " << argv[1] << std::endl;
    } else {
        // Configuration minimale manuelle
        params.state_options.num_cameras = 1;  // Monocular
        params.state_options.max_clone_size = 11;
        params.state_options.max_slam_features = 25;
        params.state_options.do_fej = true;
        params.state_options.imu_avg = true;
        
        params.init_window_time = 2.0;
        params.init_imu_thresh = 1.5;
        
        std::cout << "[INFO] Configuration par défaut (monocular)" << std::endl;
    }
    
    // ========================================================================
    // ÉTAPE 2 : Initialiser le VioManager
    // ========================================================================
    
    ov_msckf::VioManager vio_manager(params);
    std::cout << "[OK] VioManager initialisé" << std::endl;
    
    // ========================================================================
    // ÉTAPE 3 : Boucle principale - Traitement des mesures
    // ========================================================================
    
    double current_time = 0.0;
    int imu_count = 0;
    int cam_count = 0;
    
    std::cout << "\n--- Simulation d'acquisition de données ---" << std::endl;
    
    // Simulation de 100 itérations
    for (int i = 0; i < 100; i++) {
        
        current_time += 0.01; // 100 Hz
        
        // ---------------------------------------------------------------------
        // A) Mesures IMU (typiquement 100-400 Hz)
        // ---------------------------------------------------------------------
        if (i % 1 == 0) { // À chaque itération (100 Hz)
            
            ov_core::ImuData imu_data;
            imu_data.timestamp = current_time;
            
            // Données simulées (remplacer par vraies données de votre IMU)
            imu_data.wm << 0.01, -0.02, 0.001;  // Gyroscope (rad/s)
            imu_data.am << 0.05, 0.03, 9.81;     // Accéléromètre (m/s²)
            
            vio_manager.feed_measurement_imu(imu_data);
            imu_count++;
        }
        
        // ---------------------------------------------------------------------
        // B) Mesures Caméra (typiquement 20-30 Hz)
        // ---------------------------------------------------------------------
        if (i % 5 == 0) { // Tous les 5 itérations (20 Hz)
            
            ov_core::CameraData cam_data;
            cam_data.timestamp = current_time;
            
            // Image simulée (remplacer par vraie image de votre caméra)
            cv::Mat dummy_image = cv::Mat::zeros(480, 640, CV_8UC1);
            
            cam_data.sensor_ids.push_back(0);  // Camera ID = 0
            cam_data.images.push_back(dummy_image);
            cam_data.masks.push_back(cv::Mat());  // Pas de masque
            
            vio_manager.feed_measurement_camera(cam_data);
            cam_count++;
        }
        
        // ---------------------------------------------------------------------
        // C) Récupération de la pose estimée
        // ---------------------------------------------------------------------
        if (vio_manager.initialized()) {
            
            // Récupérer l'état actuel du système
            auto state = vio_manager.get_state();
            
            // Position dans le repère global (x, y, z)
            Eigen::Vector3d position = state->_imu->pos();
            
            // Orientation (quaternion)
            Eigen::Vector4d orientation = state->_imu->quat();
            
            // Vitesse
            Eigen::Vector3d velocity = state->_imu->vel();
            
            // Affichage périodique
            if (i % 20 == 0) {
                std::cout << "[t=" << current_time << "s] "
                          << "Pos: [" << position.transpose() << "] | "
                          << "Ori: [" << orientation.transpose() << "]"
                          << std::endl;
            }
            
            // ================================================================
            // ICI : Envoi vers le serveur Overview de b-com
            // ================================================================
            // 
            // Exemple de format de transmission :
            // {
            //   "timestamp": current_time,
            //   "position": {x: position(0), y: position(1), z: position(2)},
            //   "orientation": {qw: orientation(3), qx: orientation(0), 
            //                   qy: orientation(1), qz: orientation(2)},
            //   "velocity": {vx: velocity(0), vy: velocity(1), vz: velocity(2)}
            // }
            //
            // TODO: Implémenter OverviewClient::sendPose(...)
        }
    }
    
    std::cout << "\n=== Résumé ===" << std::endl;
    std::cout << "Mesures IMU traitées: " << imu_count << std::endl;
    std::cout << "Images traitées: " << cam_count << std::endl;
    std::cout << "Système initialisé: " << (vio_manager.initialized() ? "OUI" : "NON") << std::endl;
    
    return 0;
}
