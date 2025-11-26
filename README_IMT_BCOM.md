# Projet VIO Embarqué - IMT Atlantique × IRT b-com

## 🎯 Objectif
Développer un système embarqué de Visual-Inertial Odometry temps réel pour applications industrielles et logistiques, transmettant les données d'odométrie au serveur **Overview** de b-com.

## 📋 Statut actuel (26 Nov 2025)
- ✅ OpenVINS compilé en mode ROS-free (Ceres 2.2.0)
- ✅ Patch compatibilité Ceres Manifold API appliqué
- ✅ Simulateur fonctionnel et validé
- ✅ Exemples d'intégration créés (`examples_integration/`)
- ✅ Dataset EuRoC MH_01 téléchargé et testé
- ✅ Documentation complète pour reproduction

## 🏗️ Architecture prévue

```
┌─────────────────┐
│  Caméra(s)      │──┐
│  + IMU          │  │
└─────────────────┘  │
                     ▼
┌──────────────────────────────┐
│     OpenVINS (modifié)       │
│  - VioManager                │
│  - feed_measurement_imu()    │
│  - feed_measurement_camera() │
└──────────────────────────────┘
                     │
                     ▼
┌──────────────────────────────┐
│  Interface Overview (TODO)   │
│  - Format pose (x,y,z,quat)  │
│  - Timestamp synchronisé     │
│  - Protocole réseau (TCP/UDP)│
└──────────────────────────────┘
                     │
                     ▼
         ┌───────────────────┐
         │ Serveur Overview  │
         │    (IRT b-com)    │
         └───────────────────┘
```

## 📂 Structure des fichiers

```
open_vins/
├── config_imt_bcom/          # Configs personnalisées (à créer)
│   ├── camera_industrielle.yaml
│   └── imu_config.yaml
├── examples_integration/      # ✅ Exemples d'intégration
│   ├── minimal_vio_example.cpp
│   ├── CMakeLists.txt
│   └── README.md
└── interface_overview/        # Module interface b-com (à créer)
    ├── OverviewClient.h
    └── OverviewClient.cpp
```

## 🚀 Quick Start - Reproduire les tests

### 1. Compilation du projet

```bash
# Installation des dépendances (Ubuntu 24.04)
sudo apt install -y libeigen3-dev libboost-all-dev libceres-dev \
                    libopencv-dev git cmake build-essential

# Compilation OpenVINS en mode ROS-free
cd ~/workspace/open_vins/ov_msckf
mkdir -p build && cd build
cmake -DENABLE_ROS=OFF ..
make -j$(nproc)
sudo make install
```

### 2. Test du simulateur

```bash
cd ~/workspace/open_vins
./ov_msckf/build/run_simulation config/rpng_sim/estimator_config.yaml
```

**Attendu :** Affichage continu de la pose estimée (Ctrl+C pour arrêter).

### 3. Test de l'exemple d'intégration

```bash
cd ~/workspace/open_vins/examples_integration
mkdir -p build && cd build
cmake .. && make
./minimal_vio_example
```

**Attendu :** Simulation de 100 itérations avec affichage périodique de la pose.

### 4. Télécharger et tester avec dataset réel (optionnel)

```bash
# Téléchargement EuRoC MH_01 (~1.7GB, nécessite WiFi)
mkdir -p ~/datasets && cd ~/datasets
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
unzip MH_01_easy.zip
# Dataset extrait dans ~/datasets/mav0/
```

## 🔧 Modifications apportées

### Patch Ceres 2.x (fichiers modifiés)
- `ov_init/src/ceres/State_JPLQuatLocal.h` : Migration de `LocalParameterization` vers `Manifold`
- `ov_init/src/ceres/State_JPLQuatLocal.cpp` : Ajout méthodes `Minus()` et `MinusJacobian()`
- `config/rpng_sim/estimator_config.yaml` : Correction chemin trajectoire simulation

## 📝 TODO

- [x] Étudier API VioManager pour récupération de pose
- [x] Créer exemple minimal d'intégration
- [x] Tester sur dataset EuRoC
- [ ] Définir format de communication avec Overview (protocole réseau)
- [ ] Benchmarker performances temps réel sur plateforme embarquée
- [ ] Définir calibration capteurs industriels
- [ ] Implémenter classe `OverviewClient` pour transmission données
- [ ] Tester avec vraies caméra/IMU industrielles

## 📚 Ressources

- [Documentation OpenVINS](https://docs.openvins.com/)
- [Paper ICRA 2020](https://pgeneva.com/downloads/papers/Geneva2020ICRA.pdf)
- [Repo GitHub original](https://github.com/rpng/open_vins)

