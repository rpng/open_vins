# Projet VIO Embarqué - IMT Atlantique × IRT b-com

## 🎯 Objectif
Développer un système embarqué de Visual-Inertial Odometry temps réel pour applications industrielles et logistiques, transmettant les données d'odométrie au serveur **Overview** de b-com.

## 📋 Statut actuel (26 Nov 2025)
- ✅ OpenVINS compilé en mode ROS-free (Ceres 2.2.0)
- ✅ Patch compatibilité Ceres Manifold API appliqué
- ✅ Simulateur fonctionnel
- ⏳ Test sur dataset réel (EuRoC) à venir

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
├── examples_integration/      # Exemples d'intégration (à créer)
│   └── minimal_vio_example.cpp
└── interface_overview/        # Module interface b-com (à créer)
    ├── OverviewClient.h
    └── OverviewClient.cpp
```

## 🔧 Modifications apportées

### Patch Ceres 2.x (fichiers modifiés)
- `ov_init/src/ceres/State_JPLQuatLocal.h` : Migration de `LocalParameterization` vers `Manifold`
- `ov_init/src/ceres/State_JPLQuatLocal.cpp` : Ajout méthodes `Minus()` et `MinusJacobian()`
- `config/rpng_sim/estimator_config.yaml` : Correction chemin trajectoire simulation

## 📝 TODO

- [ ] Étudier API VioManager pour récupération de pose
- [ ] Créer exemple minimal d'intégration
- [ ] Définir format de communication avec Overview
- [ ] Tester sur dataset EuRoC (quand WiFi disponible)
- [ ] Benchmarker performances temps réel
- [ ] Définir calibration capteurs industriels
- [ ] Implémenter interface réseau Overview

## 📚 Ressources

- [Documentation OpenVINS](https://docs.openvins.com/)
- [Paper ICRA 2020](https://pgeneva.com/downloads/papers/Geneva2020ICRA.pdf)
- [Repo GitHub original](https://github.com/rpng/open_vins)

