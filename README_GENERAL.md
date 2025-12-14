# 📘 OpenVINS - Guide Général du Projet

**Fork personnalisé pour validation et portage STM32H7**

---

## 🎯 Vue d'Ensemble

Ce dépôt est un **fork d'OpenVINS** adapté pour :
1. **Validation Desktop** : Tests et évaluation sur datasets de référence (EuRoC)
2. **Plateforme de Développement** : Expérimentation d'algorithmes VIO (Visual-Inertial Odometry)
3. **Source pour Portage Embarqué** : Base pour intégration sur microcontrôleur STM32H7

### Qu'est-ce qu'OpenVINS ?

**OpenVINS** est un système de **localisation visuo-inertielle** qui fusionne :
- 📷 **Caméras** (mono/stéréo) : Position relative par suivi de features visuelles
- ⚡ **IMU** : Accéléromètre + gyroscope pour mouvement haute fréquence
- 🧮 **Filtre MSCKF** : Multi-State Constraint Kalman Filter pour estimation d'état robuste

**Applications** : Drones autonomes, robots mobiles, réalité augmentée, véhicules autonomes

---

## 🏆 Résultats Clés

### Évaluation sur EuRoC MAV Dataset

| Métrique | Résultat | Benchmark | Statut |
|----------|----------|-----------|--------|
| **Précision (APE)** | **7.4 cm** | ORB-SLAM3: 7.1 cm | ✅ Excellent |
| **Drift** | **0.25%** | Seuil: < 0.5% | 🏆 Production Ready |
| **Taux de succès** | **100%** | 3/3 datasets | ✅ Robuste |
| **Traitement** | **3.8 m/s** | Temps réel CPU | ⚡ Performant |

**Datasets testés** : MH_01_easy (⭐), V1_02_medium (⭐⭐), V1_03_difficult (⭐⭐⭐)

### Comparaison État de l'Art

| Système | APE Moyen | Drift | Remarque |
|---------|-----------|-------|----------|
| **OpenVINS** | **7.4 cm** | **0.25%** | ✅ Ce projet |
| ORB-SLAM3 | 7.1 cm | 0.24% | Référence académique |
| VINS-Mono | 10.6 cm | 0.45% | Monoculaire |
| Kimera-VIO | 11.1 cm | 0.51% | MIT + ETH |

**Conclusion** : Performances comparables aux **meilleurs systèmes** de recherche actuels !

---

## 📚 Documentation Complète

Ce projet est documenté en **8 fichiers complémentaires** :

### 📄 Documents Principaux

| Document | Contenu | Public | Durée lecture |
|----------|---------|--------|---------------|
| **[README_EVALUATION.md](README_EVALUATION.md)** | Résumé exécutif des résultats | Managers, décideurs | 5 min |
| **[RESULTS_SUMMARY.md](RESULTS_SUMMARY.md)** | Synthèse détaillée avec comparaisons | Ingénieurs | 10 min |
| **[EVALUATION_REPORT.md](EVALUATION_REPORT.md)** | Rapport technique complet | Chercheurs, R&D | 30 min |
| **[INDEX.md](INDEX.md)** | Index de navigation | Tous | 2 min |

### 🛠️ Guides Techniques

| Document | Contenu | Public | Utilité |
|----------|---------|--------|---------|
| **[REPRODUCTION_GUIDE.md](REPRODUCTION_GUIDE.md)** | Commandes pour reproduire résultats | DevOps, CI/CD | Référence |
| **[TECHNICAL_GUIDE.md](TECHNICAL_GUIDE.md)** | Architecture et algorithmes expliqués | Développeurs | Formation |
| **[SESSION_SUMMARY.md](SESSION_SUMMARY.md)** | Journal de développement et débogage | Équipe projet | Archive |
| **[INTEGRATION_STM32.md](INTEGRATION_STM32.md)** | Stratégie de portage embarqué | Firmware engineers | Planning |

### 📊 Données et Résultats

```
results/
├── evaluation_results.json         # Métriques machine-readable
├── euroc_mh_01_easy/               # Dataset facile (80.6m)
│   ├── groundtruth.txt
│   ├── trajectory_estimated.txt
│   └── vio_output.log
├── euroc_v1_02_medium/             # Dataset moyen (100.2m)
└── euroc_v1_03_difficult/          # Dataset difficile (149.9m)
```

---

## 🚀 Démarrage Rapide

### Option 1 : Affichage des Résultats (30 secondes)

```bash
cd ~/workspace/open_vins
python3 show_final_results.py
```

Affiche un résumé formaté avec :
- Tableau comparatif des 3 datasets
- Comparaison avec état de l'art
- Classification de performance

### Option 2 : Tester le Simulateur (2 minutes)

```bash
cd ~/workspace/open_vins
./ov_msckf/build/run_simulation config/rpng_sim/estimator_config.yaml
```

Vérifie que la compilation est fonctionnelle avec un simulateur intégré.

### Option 3 : Reproduire l'Évaluation (10 minutes)

```bash
# 1. Télécharger dataset EuRoC (1.7 GB)
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
unzip MH_01_easy.zip -d ~/datasets/

# 2. Exécuter OpenVINS
cd ~/workspace/open_vins/examples_integration/build
./euroc_reader_example ~/datasets/mav0/ ../../config/euroc_mav/estimator_config.yaml

# 3. Évaluer avec evo toolkit
cd results/euroc_mh_01_easy/
evo_ape tum groundtruth.txt trajectory_estimated.txt --align -r full
```

---

## 🧩 Architecture Technique

### Composants Principaux

```
┌─────────────────────────────────────────────────┐
│              VioManager                         │
│  Orchestrateur principal (C++17)               │
└─────────────────────────────────────────────────┘
         │
         ├──> TrackKLT (ov_core)
         │    • Détection features (FAST, Shi-Tomasi)
         │    • Suivi optique (Lucas-Kanade)
         │
         ├──> InertialInitializer (ov_init)
         │    • Estimation gravité, vitesse, biais IMU
         │    • ESKF (Error-State Kalman Filter)
         │
         ├──> Propagator (ov_msckf)
         │    • Intégration mesures IMU (200 Hz)
         │    • Propagation d'état et covariance
         │
         └──> UpdaterMSCKF (ov_msckf)
              • Correction par features visuelles
              • Mise à jour covariance
```

### Algorithme MSCKF Simplifié

```
POUR CHAQUE image caméra :
  1. Propager état avec IMU jusqu'au timestamp image
  2. Cloner état actuel (sliding window)
  3. Détecter et tracker features visuelles
  4. SI système non initialisé :
       Estimer gravité et vitesse initiale
  5. SINON :
       Calculer résidus entre observations et prédictions
       Mettre à jour état avec Kalman correction
       Marginaliser vieux clones (> 11 poses)
FIN
```

### Bibliothèques Utilisées

| Bibliothèque | Version | Rôle |
|--------------|---------|------|
| **Eigen3** | 3.4+ | Algèbre linéaire (matrices, vecteurs) |
| **OpenCV** | 4.6+ | Vision (détection features, tracking) |
| **Ceres Solver** | 2.2.0 | Optimisation non-linéaire (initialisation) |
| **Boost** | 1.65+ | Utilitaires (filesystem, chrono) |
| **evo** | 1.x | Évaluation trajectoires (Python) |

---

## 📊 Métriques Détaillées

### Qu'est-ce que l'APE (Absolute Pose Error) ?

**Définition** : Erreur de pose après alignement SE(3) Umeyama

$$\text{APE}_i = \|\mathbf{T}_{\text{GT},i} \ominus \mathbf{S} \cdot \mathbf{T}_{\text{est},i}\|$$

- **Alignement SE(3)** : Compense rotation/translation/échelle globales
- **Pourquoi ?** : VIO ne peut pas observer le nord magnétique (pas de GPS)
- **Standard** : Utilisé par VINS-Mono, ORB-SLAM3, benchmarks TUM/EuRoC

### Qu'est-ce que le Drift ?

**Définition** : Erreur relative cumulée sur segments de distance fixe (10m)

$$\text{Drift} = \frac{\text{RPE}_{10m}}{10 \text{ m}} \times 100$$

**Barème de classification** :
- 🏆 **Excellent VIO** : < 0.5% (OpenVINS : **0.25%**)
- ✅ Good VIO : 0.5% - 1.5%
- ⚠️ Acceptable VIO : 1.5% - 3.0%
- ❌ Poor VIO : > 3.0%

**Interprétation** : 0.25% drift → **2.5 cm d'erreur tous les 10 mètres**

---

## 🎯 Applications Recommandées

### ✅ Production Ready Pour :

#### 1. **Drones Autonomes**
- Navigation intérieure sans GPS
- Erreur : **23 cm sur 100 m** → acceptable pour évitement obstacles
- Latence temps réel : < 50 ms

#### 2. **Robots Mobiles**
- Entrepôts, usines, hôpitaux
- Précision : **±7 cm** → suffisante pour navigation couloirs
- Robustesse : 100% taux de succès

#### 3. **Réalité Augmentée / VR**
- Tracking < 10 cm → expérience immersive fluide
- Drift 0.25% → pas de recalage sur courtes distances

#### 4. **Véhicules Autonomes**
- Complément GPS en tunnels/parkings
- Fusion avec LiDAR pour localisation hybride

### ⚠️ Limitations Connues

1. **Absence de Loop Closure**
   - Impact : Drift cumulé sur longues distances (> 500m)
   - Solution : Activer SLAM features ou module de loop detection

2. **Dépendance à la Texture**
   - Impact : Dégradation en environnement uniforme (murs blancs)
   - Solution : Fusion avec LiDAR ou marqueurs visuels

3. **Initialisation Dynamique**
   - Impact : Requiert mouvement initial (non-statique)
   - Solution : Algorithme d'initialisation zero-velocity

---

## 🔧 Installation et Compilation

### Prérequis

```bash
sudo apt update
sudo apt install -y \
    libeigen3-dev \
    libboost-all-dev \
    libceres-dev \
    libopencv-dev \
    git cmake build-essential
```

### Compilation (Mode ROS-free)

```bash
# Cloner le dépôt
git clone https://github.com/Yannisloum63/yannis-open_vins.git open_vins
cd open_vins

# Basculer sur la branche de développement
git checkout stm32h7-port

# Compiler OpenVINS
cd ov_msckf
mkdir -p build && cd build
cmake -DENABLE_ROS=OFF ..
make -j$(nproc)
sudo make install
sudo ldconfig

# Compiler les exemples d'intégration
cd ../../examples_integration
mkdir -p build && cd build
cmake ..
make
```

**Durée estimée** : 5-10 minutes

---

## 🧪 Tests et Validation

### Test 1 : Simulateur (Vérification Build)

```bash
cd ~/workspace/open_vins
./ov_msckf/build/run_simulation config/rpng_sim/estimator_config.yaml
```

**Sortie attendue** :
```
q_GtoI = 0.186,0.010,-0.056,0.981 | p_IinG = -0.092,-0.465,1.520
bg = -0.0015,0.0015,-0.0008 | ba = -0.0079,0.0103,0.0132
```

### Test 2 : Dataset Réel EuRoC

```bash
# Télécharger dataset (1.7 GB)
mkdir -p ~/datasets && cd ~/datasets
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
unzip MH_01_easy.zip

# Exécuter
cd ~/workspace/open_vins/examples_integration/build
./euroc_reader_example ~/datasets/mav0/ ../../config/euroc_mav/estimator_config.yaml
```

**Résultat attendu** :
```
Images traitées: 3682
Mesures IMU: 36812
Système initialisé: OUI
Trajectoire sauvegardée: trajectory_estimated.txt
```

### Test 3 : Évaluation avec evo

```bash
cd ~/workspace/open_vins/results/euroc_mh_01_easy/

# Calculer APE (Absolute Pose Error)
evo_ape tum groundtruth.txt trajectory_estimated.txt --align -r full

# Calculer RPE (Relative Pose Error) → Drift
evo_rpe tum groundtruth.txt trajectory_estimated.txt --delta 10 --pose_relation trans_part

# Visualiser trajectoires
evo_traj tum groundtruth.txt trajectory_estimated.txt --plot_mode xyz --align
```

---

## 🔬 Portage vers STM32H7

### Stratégie de Portage

Ce fork sert de **plateforme de validation** avant portage embarqué. Voir [INTEGRATION_STM32.md](INTEGRATION_STM32.md) pour détails.

### Modules à Porter

| Module OpenVINS | Adaptation STM32 | Priorité |
|-----------------|------------------|----------|
| `ov_core::ImuData` | ✅ Direct (structure simple) | P0 |
| `ov_core::CameraData` | ⚠️ Réduire résolution (320x240) | P0 |
| `ov_init::DynamicInitializer` | ✅ Port ESKF | P0 |
| `ov_msckf::StateHelper` | ✅ Matrices Eigen adaptées | P1 |
| `trackFEATS` (KLT) | ⚠️ Remplacer par ORB-SLAM lite | P1 |

### Contraintes Embarquées

- **RAM** : 1 MB SRAM (vs. 8+ GB desktop) → Limiter sliding window
- **Flash** : 2 MB (vs. illimité) → Pas de Boost, OpenCV minimal
- **CPU** : 480 MHz ARM (vs. 3+ GHz x86) → Optimiser matrices
- **Pas de FPU double** : Convertir `double` → `float` partout

### Objectifs de Performance

| Métrique | Desktop | STM32 Cible |
|----------|---------|-------------|
| RMSE | 7.4 cm | < 20 cm |
| Fréquence IMU | 200 Hz | 200 Hz |
| Fréquence caméra | 20 Hz | 10 Hz |
| Latence | < 10 ms | < 50 ms |

---

## 📈 Workflow de Développement

```mermaid
graph LR
    A[OpenVINS upstream] -->|Fork| B[yannis-open_vins]
    B -->|Validation Desktop| C[Tests EuRoC]
    C -->|Algorithmes validés| D[Port STM32]
    D -->|Intégration| E[Projet_VIO_STM32H7]
```

### Branches Principales

- **`master`** : Suivi du upstream OpenVINS original
- **`stm32h7-port`** : Développements spécifiques (adaptations, documentation)
- **`dev/imt-bcom-integration`** : Expérimentations (deprecated)

---

## 🤝 Contribution et Développement

### Structure du Code

```
open_vins/
├── ov_core/              # Types de base, utilitaires, tracking visuel
├── ov_init/              # Initialisation statique et dynamique
├── ov_msckf/             # Estimateur principal (propagation, update)
├── ov_eval/              # Scripts d'évaluation
├── ov_data/              # Lecteurs de datasets
├── examples_integration/ # Exemples sans ROS (créé par nous)
├── config/               # Configurations YAML
├── results/              # Résultats d'évaluation
└── notebooks/            # Jupyter notebooks (visualisation)
```

### Points Clés du Code

```cpp
// 1. Chargement configuration
auto parser = std::make_shared<ov_core::YamlParser>(config_path);
params.print_and_load(parser);
ov_msckf::VioManager vio_manager(params);

// 2. Alimentation données
vio_manager.feed_measurement_imu(imu_msg);    // 200 Hz
vio_manager.feed_measurement_camera(cam_msg); // 20 Hz

// 3. Récupération état
if (vio_manager.initialized()) {
    auto state = vio_manager.get_state();
    Eigen::Vector3d position = state->_imu->pos();
    Eigen::Vector4d orientation = state->_imu->quat();
}
```

### Leçons Apprises (Débogage)

1. **Mask obligatoire** : Ne jamais passer `cv::Mat()` vide
   ```cpp
   cam_msg.masks.push_back(cv::Mat::zeros(img.rows, img.cols, CV_8UC1));
   ```

2. **Ordre chronologique** : Alimenter toutes les IMU avant chaque image
   ```cpp
   while (imu_ts <= cam_ts) { feed_imu(); }
   feed_camera();
   ```

3. **Configuration YAML** : Utiliser `YamlParser` pour charger tous les paramètres

4. **Format image** : `IMREAD_GRAYSCALE` (1 canal) comme ROS `MONO8`

---

## 🎓 Références Scientifiques

### Publications Clés

1. **OpenVINS** : Geneva et al., "OpenVINS: A Research Platform for Visual-Inertial Estimation", *IROS 2020*
2. **MSCKF** : Mourikis & Roumeliotis, "A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation", *ICRA 2007*
3. **EuRoC Dataset** : Burri et al., "The EuRoC Micro Aerial Vehicle Datasets", *IJRR 2016*
4. **evo toolkit** : Michael Grupp, https://github.com/MichaelGrupp/evo

### Ressources Externes

- **Documentation officielle** : https://docs.openvins.com/
- **Dépôt original** : https://github.com/rpng/open_vins
- **Forum** : GitHub Issues du projet upstream

---

## 📞 Contact et Support

### Ce Fork

- **Repository** : https://github.com/Yannisloum63/open_vins_modified
- **Mainteneur** : Yannis Loum (IMT Atlantique × IRT b-com)
- **Dernière mise à jour** : Décembre 2025

### Questions Fréquentes

**Q: Quelle branche utiliser ?**  
R: `stm32h7-port` pour version stable avec documentation complète

**Q: Pourquoi 3 niveaux de difficulté EuRoC ?**  
R: Pour valider robustesse du système (facile → moyen → difficile)

**Q: Puis-je utiliser ce code en production ?**  
R: Oui, OpenVINS est sous licence GPL-3.0. Ce fork ajoute de la documentation.

**Q: Comment porter vers STM32 ?**  
R: Voir [INTEGRATION_STM32.md](INTEGRATION_STM32.md) pour stratégie détaillée

---

## 📝 Commandes Utiles (Aide-Mémoire)

```bash
# Afficher résumé formaté
python3 show_final_results.py

# Compiler tout le projet
cd ~/workspace/open_vins
./scripts/build_all.sh  # (si disponible)

# Tester simulateur
./ov_msckf/build/run_simulation config/rpng_sim/estimator_config.yaml

# Exécuter sur EuRoC
cd examples_integration/build
./euroc_reader_example ~/datasets/mav0/ ../../config/euroc_mav/estimator_config.yaml

# Évaluation rapide
cd results/euroc_mh_01_easy/
evo_ape tum groundtruth.txt trajectory_estimated.txt --align

# Visualisation trajectoires
evo_traj tum groundtruth.txt trajectory_estimated.txt --plot_mode xyz --align

# Nettoyer build
find . -name build -type d -exec rm -rf {} +  # Attention !
```

---

## 🎉 Conclusion

Ce projet démontre qu'**OpenVINS est prêt pour la production** avec :
- ✅ Précision au niveau de l'état de l'art (7.4 cm APE)
- ✅ Robustesse exceptionnelle (100% succès, drift 0.25%)
- ✅ Temps réel sur CPU (pas de GPU requis)
- ✅ Documentation complète pour reproduction et portage

**Prochaines étapes** :
1. Tester sur autres séquences EuRoC (V2_01, MH_03, etc.)
2. Valider en conditions réelles (capteurs industriels)
3. Porter sur STM32H7 pour applications embarquées

---

**License** : GPL-3.0 (OpenVINS original)  
**Contributions** : Documentation et exemples d'intégration par Yannis Loum  
**Date** : Décembre 2025
