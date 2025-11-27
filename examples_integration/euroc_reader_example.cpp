/**
 * @file euroc_reader_example.cpp
 * @brief Lecteur de dataset EuRoC pour alimenter OpenVINS avec vraies données
 * @author Projet IMT Atlantique × IRT b-com
 * 
 * Ce fichier :
 * 1. Lit les fichiers CSV du dataset EuRoC (IMU + images)
 * 2. Alimente OpenVINS avec les mesures réelles
 * 3. Affiche et sauvegarde la trajectoire estimée
 * 
 * Usage:
 *   ./euroc_reader_example /path/to/mav0 [config.yaml]
 */

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "utils/sensor_data.h"
#include "state/State.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>

// Structure pour stocker les données IMU
struct ImuMeasurement {
    double timestamp;
    Eigen::Vector3d gyro;
    Eigen::Vector3d accel;
};

// Structure pour stocker les timestamps d'images
struct ImageMeasurement {
    double timestamp;
    std::string filename;
};

// Fonction pour lire le fichier CSV de l'IMU
std::vector<ImuMeasurement> read_imu_data(const std::string& imu_csv_path) {
    std::vector<ImuMeasurement> imu_data;
    std::ifstream file(imu_csv_path);
    
    if (!file.is_open()) {
        std::cerr << "[ERROR] Impossible d'ouvrir: " << imu_csv_path << std::endl;
        return imu_data;
    }
    
    std::string line;
    std::getline(file, line); // Skip header
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;
        
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }
        
        if (tokens.size() < 7) continue;
        
        ImuMeasurement meas;
        meas.timestamp = std::stod(tokens[0]) * 1e-9; // nanosec -> sec
        meas.gyro << std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]);
        meas.accel << std::stod(tokens[4]), std::stod(tokens[5]), std::stod(tokens[6]);
        
        imu_data.push_back(meas);
    }
    
    file.close();
    std::cout << "[INFO] Chargé " << imu_data.size() << " mesures IMU" << std::endl;
    return imu_data;
}

// Fonction pour lire le fichier CSV des images
std::vector<ImageMeasurement> read_image_data(const std::string& cam_csv_path) {
    std::vector<ImageMeasurement> img_data;
    std::ifstream file(cam_csv_path);
    
    if (!file.is_open()) {
        std::cerr << "[ERROR] Impossible d'ouvrir: " << cam_csv_path << std::endl;
        return img_data;
    }
    
    std::string line;
    std::getline(file, line); // Skip header
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;
        
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }
        
        if (tokens.size() < 2) continue;
        
        ImageMeasurement meas;
        meas.timestamp = std::stod(tokens[0]) * 1e-9; // nanosec -> sec
        meas.filename = tokens[1];
        
        img_data.push_back(meas);
    }
    
    file.close();
    std::cout << "[INFO] Chargé " << img_data.size() << " timestamps d'images" << std::endl;
    return img_data;
}

int main(int argc, char** argv) {
    
    std::cout << "========================================" << std::endl;
    std::cout << "  Lecteur Dataset EuRoC pour OpenVINS  " << std::endl;
    std::cout << "========================================" << std::endl;
    
    // ========================================================================
    // ÉTAPE 1 : Vérifier les arguments
    // ========================================================================
    
    if (argc < 2) {
        std::cerr << "[ERROR] Usage: " << argv[0] << " <path_to_mav0> [config.yaml]" << std::endl;
        std::cerr << "Exemple: " << argv[0] << " ~/datasets/mav0/" << std::endl;
        return -1;
    }
    
    std::string dataset_path = argv[1];
    if (dataset_path.back() != '/') dataset_path += "/";
    
    std::cout << "[INFO] Dataset: " << dataset_path << std::endl;
    
    // ========================================================================
    // ÉTAPE 2 : Configuration OpenVINS
    // ========================================================================
    
    ov_msckf::VioManagerOptions params;
    
    // Utiliser la config YAML si disponible
    if (argc > 2) {
        std::cout << "[INFO] Utilisation config YAML: " << argv[2] << std::endl;
        // TODO: Parser YAML config
    }
    
    // Configuration minimale pour EuRoC (monocular)
    params.state_options.num_cameras = 1;
    params.state_options.max_clone_size = 11;
    params.state_options.max_slam_features = 50;
    params.state_options.do_fej = true;
    params.use_stereo = false;
    
    std::cout << "[INFO] Configuration: monocular sans calibration (simulateur)" << std::endl;
    std::cout << "[WARN] Pour de meilleurs résultats, utilisez: " << std::endl;
    std::cout << "[WARN]   ./euroc_reader_example ~/datasets/mav0/ ~/workspace/open_vins/config/euroc_mav/estimator_config.yaml" << std::endl;
    
    // ========================================================================
    // ÉTAPE 3 : Initialiser VioManager
    // ========================================================================
    
    ov_msckf::VioManager vio_manager(params);
    std::cout << "[OK] VioManager initialisé" << std::endl;
    
    // ========================================================================
    // ÉTAPE 4 : Charger les données du dataset
    // ========================================================================
    
    std::string imu_file = dataset_path + "imu0/data.csv";
    std::string cam0_file = dataset_path + "cam0/data.csv";
    std::string cam1_file = dataset_path + "cam1/data.csv";
    std::string cam0_img_dir = dataset_path + "cam0/data/";
    std::string cam1_img_dir = dataset_path + "cam1/data/";
    
    auto imu_data = read_imu_data(imu_file);
    auto cam0_data = read_image_data(cam0_file);
    auto cam1_data = read_image_data(cam1_file);
    
    if (imu_data.empty() || cam0_data.empty()) {
        std::cerr << "[ERROR] Impossible de charger les données du dataset!" << std::endl;
        return -1;
    }
    
    // ========================================================================
    // ÉTAPE 5 : Traiter les données en ordre chronologique
    // ========================================================================
    
    size_t imu_idx = 0;
    size_t cam0_idx = 0;
    size_t cam1_idx = 0;
    
    int frame_count = 0;
    std::ofstream traj_file("trajectory_estimated.txt");
    traj_file << "# timestamp tx ty tz qx qy qz qw" << std::endl;
    
    std::cout << "\n[INFO] Démarrage du traitement..." << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    
    // Traiter jusqu'à épuisement des images
    while (cam0_idx < cam0_data.size()) {
        
        double cam_timestamp = cam0_data[cam0_idx].timestamp;
        
        // Alimenter avec toutes les mesures IMU jusqu'à cette image
        while (imu_idx < imu_data.size() && imu_data[imu_idx].timestamp <= cam_timestamp) {
            ov_core::ImuData imu_msg;
            imu_msg.timestamp = imu_data[imu_idx].timestamp;
            imu_msg.wm = imu_data[imu_idx].gyro;
            imu_msg.am = imu_data[imu_idx].accel;
            
            vio_manager.feed_measurement_imu(imu_msg);
            imu_idx++;
        }
        
        // Charger les images cam0 et cam1
        std::string img0_path = cam0_img_dir + cam0_data[cam0_idx].filename;
        cv::Mat img0 = cv::imread(img0_path, cv::IMREAD_GRAYSCALE);
        
        if (img0.empty()) {
            std::cerr << "[WARN] Image introuvable: " << img0_path << std::endl;
            cam0_idx++;
            continue;
        }
        
        // Préparer les données caméra
        ov_core::CameraData cam_msg;
        cam_msg.timestamp = cam_timestamp;
        cam_msg.sensor_ids.push_back(0);
        cam_msg.images.push_back(img0);
        cam_msg.masks.push_back(cv::Mat());
        
        // Note: On ignore cam1 pour l'instant (mode monocular)
        
        // Alimenter OpenVINS
        vio_manager.feed_measurement_camera(cam_msg);
        frame_count++;
        
        // Récupérer et afficher la pose estimée
        if (vio_manager.initialized()) {
            auto state = vio_manager.get_state();
            Eigen::Vector3d pos = state->_imu->pos();
            Eigen::Vector4d quat = state->_imu->quat();
            Eigen::Vector3d vel = state->_imu->vel();
            
            // Sauvegarder dans fichier
            traj_file << std::fixed << std::setprecision(9) 
                      << cam_timestamp << " "
                      << pos(0) << " " << pos(1) << " " << pos(2) << " "
                      << quat(0) << " " << quat(1) << " " << quat(2) << " " << quat(3)
                      << std::endl;
            
            // Affichage périodique
            if (frame_count % 10 == 0) {
                std::cout << "[Frame " << frame_count << "] "
                          << "t=" << std::setprecision(3) << cam_timestamp << "s | "
                          << "Pos: [" << std::setprecision(2) 
                          << pos(0) << ", " << pos(1) << ", " << pos(2) << "] | "
                          << "Vel: " << vel.norm() << " m/s"
                          << std::endl;
            }
        } else {
            if (frame_count % 10 == 0) {
                std::cout << "[Frame " << frame_count << "] Initialisation en cours..." << std::endl;
            }
        }
        
        cam0_idx++;
    }
    
    traj_file.close();
    
    // ========================================================================
    // RÉSUMÉ
    // ========================================================================
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Traitement terminé" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Images traitées: " << frame_count << std::endl;
    std::cout << "Mesures IMU: " << imu_idx << std::endl;
    std::cout << "Système initialisé: " << (vio_manager.initialized() ? "OUI" : "NON") << std::endl;
    std::cout << "Trajectoire sauvegardée: trajectory_estimated.txt" << std::endl;
    
    return 0;
}
