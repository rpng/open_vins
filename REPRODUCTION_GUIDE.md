# Guide de reproduction complet - Projet VIO IMT-bcom

Ce document liste **toutes les commandes** utilisées pour configurer, compiler et tester OpenVINS en mode ROS-free.

## 🔧 Environnement

- **OS :** Ubuntu 24.04 LTS
- **Compilateur :** GCC/G++
- **Dépendances :** Eigen3, Boost, Ceres 2.2.0, OpenCV 4.6.0

---

## 📦 Étape 1 : Installation des dépendances

```bash
sudo apt update
sudo apt install -y \
    libeigen3-dev \
    libboost-all-dev \
    libceres-dev \
    libopencv-dev \
    libopencv-contrib-dev \
    git \
    cmake \
    build-essential \
    wget \
    unzip

# Vérification des versions installées
dpkg -l | grep -E "(libeigen|libceres|libopencv|libboost)"
```

---

## 📥 Étape 2 : Cloner le fork du projet

```bash
# Créer le dossier de travail
mkdir -p ~/workspace
cd ~/workspace

# Cloner VOTRE fork (remplacer par votre username)
git clone https://github.com/Yannisloum63/yannis-open_vins.git open_vins
cd open_vins

# Basculer sur la branche de développement
git checkout dev/imt-bcom-integration

# Configurer Git (si pas encore fait)
git config user.name "Votre Nom"
git config user.email "votre.email@example.com"
```

---

## ⚙️ Étape 3 : Compilation OpenVINS (mode ROS-free)

```bash
cd ~/workspace/open_vins/ov_msckf

# Créer dossier build
mkdir -p build
cd build

# Configurer CMake sans ROS
cmake -DENABLE_ROS=OFF ..

# Compiler (utilise tous les cœurs CPU disponibles)
make -j$(nproc)

# Installer les bibliothèques globalement (nécessite sudo)
sudo make install

# Vérifier l'installation
ls -lh /usr/local/lib/libov_msckf_lib.so
ls -lh /usr/local/bin/run_simulation

# Mettre à jour le cache des bibliothèques partagées
sudo ldconfig
```

**Durée estimée :** 5-10 minutes selon votre machine.

---

## 🧪 Étape 4 : Tester le simulateur

```bash
cd ~/workspace/open_vins

# Lancer le simulateur (Ctrl+C pour arrêter)
./ov_msckf/build/run_simulation config/rpng_sim/estimator_config.yaml
```

### Sortie attendue

Vous devriez voir défiler en continu :

```
q_GtoI = 0.186,0.010,-0.056,0.981 | p_IinG = -0.092,-0.465,1.520 | dist = 8.69 (meters)
bg = -0.0015,0.0015,-0.0008 | ba = -0.0079,0.0103,0.0132
camera-imu timeoffset = -0.00006
cam0 intrinsics = 458.663,457.212,366.834,248.029 | -0.284,0.074,0.000,-0.000
...
```

**Interprétation :**
- `q_GtoI` : orientation (quaternion)
- `p_IinG` : position (x, y, z en mètres)
- `bg/ba` : biais gyroscope/accéléromètre
- `cam0/1 intrinsics` : calibration caméra

**✅ Si vous voyez ces sorties, OpenVINS fonctionne correctement !**

---

## 📚 Étape 5 : Compiler l'exemple d'intégration

```bash
cd ~/workspace/open_vins/examples_integration

# Créer dossier build
mkdir -p build
cd build

# Configurer et compiler
cmake ..
make

# Vérifier la compilation
ls -lh minimal_vio_example
```

### Exécuter l'exemple

```bash
# Depuis examples_integration/build/
./minimal_vio_example
```

### Sortie attendue

```
=== Exemple minimal OpenVINS ROS-free ===
[INFO] Configuration par défaut (monocular)
[OK] VioManager initialisé

--- Simulation d'acquisition de données ---
[t=0.1s] Pos: [0.0 0.0 0.0] | Ori: [1.0 0.0 0.0 0.0]
[t=0.3s] Pos: [0.0 0.0 0.0] | Ori: [1.0 0.0 0.0 0.0]
...

=== Résumé ===
Mesures IMU traitées: 100
Images traitées: 20
Système initialisé: NON ou OUI
```

**Note :** Le système peut ne pas s'initialiser avec des données simulées vides, c'est normal.

---

## 📥 Étape 6 : Télécharger dataset EuRoC (optionnel)

**⚠️ Nécessite ~1.7 GB de téléchargement et connexion WiFi stable**

```bash
# Créer dossier datasets
mkdir -p ~/datasets
cd ~/datasets

# Télécharger EuRoC MH_01_easy
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip

# Extraire (crée dossier mav0/)
unzip MH_01_easy.zip

# Vérifier la structure
ls -lh mav0/
# Devrait montrer : cam0/, cam1/, imu0/, state_groundtruth_estimate0/, etc.
```

**Durée téléchargement :** 3-10 minutes selon votre connexion.

---

## 🎯 Étape 7 : Exécuter euroc_reader_example (validation réelle)

### Compiler l'exemple EuRoC

```bash
cd ~/workspace/open_vins/examples_integration/build

# Recompiler si nécessaire
cmake .. && make

# Vérifier que l'exécutable existe
ls -lh euroc_reader_example
```

### Exécuter avec le dataset EuRoC

```bash
# Lancer le traitement (prend ~3-5 minutes)
./euroc_reader_example ~/datasets/mav0/ ../../config/euroc_mav/estimator_config.yaml
```

### Sortie attendue

```
========================================
  Lecteur Dataset EuRoC pour OpenVINS  
========================================
[INFO] Dataset: /home/yannis/datasets/mav0/
[INFO] Chargement configuration: ../../config/euroc_mav/estimator_config.yaml
[OK] Configuration chargée avec succès
[OK] VioManager initialisé
[INFO] Chargé 36820 mesures IMU
[INFO] Chargé 3682 timestamps d'images

[INFO] Démarrage du traitement...
---------------------------------------
[Frame 10] t=1.4e+09s | Pos: [-0.06, -0.01, 0.02] | Vel: 0.12 m/s
[Frame 20] t=1.4e+09s | Pos: [-0.12, -0.01, 0.02] | Vel: 0.13 m/s
...
[Frame 3680] t=1.4e+09s | Pos: [-0.46, 0.30, -0.06] | Vel: 0.13 m/s

========================================
  Traitement terminé
========================================
Images traitées: 3682
Mesures IMU: 36812
Système initialisé: OUI
Trajectoire sauvegardée: trajectory_estimated.txt
```

**✅ Résultats attendus :**
- **3682 images** traitées
- **36812 mesures IMU** synchronisées
- Fichier `trajectory_estimated.txt` créé avec ~2263 poses

## ⚡ Script tout-en-un (démo rapide EuRoC)

Pour une démo en 1 commande (run + évaluation APE/RPE), utilisez :

```bash
cd ~/workspace/open_vins
./scripts/run_euroc_all_in_one.sh \
    ~/datasets/mav0 \
    ~/workspace/open_vins/config/euroc_mav/estimator_config.yaml \
    ~/workspace/open_vins/results/euroc_mh_01_easy
```

Ce que fait le script :
1) Convertit le ground truth CSV en format TUM (`groundtruth.txt`)
2) Lance `euroc_reader_example` et logge dans `vio_output.log`
3) Exécute `evo_ape --align` et `evo_rpe --delta 10`
4) Regroupe tout dans le dossier de résultats passé en argument

**Pré-requis**
- `examples_integration/build/euroc_reader_example` compilé
- `evo` installé (`pip install evo --upgrade`)
- Dataset EuRoC avec `state_groundtruth_estimate0/data.csv` présent dans `~/datasets/mav0`

---

## 📊 Étape 8 : Analyser les résultats avec evo

### Installer evo (outil d'évaluation)

```bash
# Installer pipx (gestionnaire d'environnements Python)
sudo apt install -y pipx
pipx ensurepath

# Installer evo
pipx install evo

# Installer tkinter pour visualisations
sudo apt install -y python3-tk

# Vérifier installation
export PATH="$HOME/.local/bin:$PATH"
evo_traj --help
```

### Convertir le ground truth EuRoC

```bash
# Créer script de conversion
cat > ~/convert_euroc_gt.py << 'EOF'
#!/usr/bin/env python3
import sys

input_file = sys.argv[1]
output_file = sys.argv[2]

with open(input_file, 'r') as fin, open(output_file, 'w') as fout:
    fout.write("# timestamp tx ty tz qx qy qz qw\n")
    for line in fin:
        if line.startswith('#'):
            continue
        parts = line.strip().split(',')
        if len(parts) < 8:
            continue
        
        timestamp = float(parts[0]) * 1e-9  # ns -> s
        px, py, pz = parts[1], parts[2], parts[3]
        qw, qx, qy, qz = parts[4], parts[5], parts[6], parts[7]
        
        # Format TUM: timestamp tx ty tz qx qy qz qw
        fout.write(f"{timestamp:.9f} {px} {py} {pz} {qx} {qy} {qz} {qw}\n")

print(f"Conversion terminée : {output_file}")
EOF

# Convertir le ground truth
python3 ~/convert_euroc_gt.py \
    ~/datasets/mav0/state_groundtruth_estimate0/data.csv \
    ~/workspace/open_vins/results/euroc_mh01_validation/groundtruth.txt
```

### Calculer l'erreur (APE)

```bash
cd ~/workspace/open_vins/examples_integration/build

# Calculer APE avec alignement
export PATH="$HOME/.local/bin:$PATH"
evo_ape tum \
    ~/workspace/open_vins/results/euroc_mh01_validation/groundtruth.txt \
    trajectory_estimated.txt \
    --align \
    --save_plot ~/workspace/open_vins/results/euroc_mh01_validation/ape_plot.pdf
```

### Résultats attendus

```
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max      0.294828
      mean      0.073918
    median      0.066045
       min      0.015840
      rmse      0.086432    ← **8.6 cm RMSE** ✅
       sse      16.905687
       std      0.044796
```

**🎯 Objectif atteint : RMSE < 10 cm (excellent pour VIO)**

### Visualiser la trajectoire 3D

```bash
# Créer visualisation 3D
evo_traj tum trajectory_estimated.txt \
    --plot_mode xyz \
    --save_plot ~/workspace/open_vins/results/euroc_mh01_validation/trajectory_3d.pdf

# Voir les statistiques
evo_traj tum trajectory_estimated.txt
```

**Statistiques attendues :**
```
name:   trajectory_estimated
infos:  2263 poses, 62.756m path length, 113.100s duration
```

---

## 🔄 Étape 9 : Workflow Git pour vos modifications

```bash
cd ~/workspace/open_vins

# Créer une nouvelle branche pour votre fonctionnalité
git checkout -b feature/ma-nouvelle-fonctionnalite

# Faire vos modifications...
# (éditer des fichiers)

# Voir les modifications
git status

# Ajouter les fichiers modifiés
git add fichier1.cpp fichier2.h

# Commiter avec message descriptif
git commit -m "feat: description de la fonctionnalité"

# Pousser sur votre fork
git push origin feature/ma-nouvelle-fonctionnalite
```

---

## 📊 Récapitulatif des commandes de test

```bash
# Test simulateur
cd ~/workspace/open_vins
timeout 10 ./ov_msckf/build/run_simulation config/rpng_sim/estimator_config.yaml

# Test exemple d'intégration
cd ~/workspace/open_vins/examples_integration/build
./minimal_vio_example

# Vérifier bibliothèques installées
ldconfig -p | grep ov_msckf
ls -l /usr/local/lib/libov_msckf_lib.so
ls -l /usr/local/bin/run_simulation
```

---

## ❓ Dépannage

### Erreur : `libov_msckf_lib.so: cannot open shared object file`

```bash
sudo ldconfig
# Vérifier que /usr/local/lib est dans le path
echo $LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

### Erreur de compilation CMake

```bash
# Nettoyer et recommencer
cd ~/workspace/open_vins/ov_msckf/build
rm -rf *
cmake -DENABLE_ROS=OFF ..
make -j$(nproc)
```

### Problème de version Ceres

Ce projet nécessite **Ceres 2.2.0**. Le patch pour Manifold API est déjà appliqué dans cette branche.

```bash
# Vérifier version Ceres
dpkg -l | grep libceres
# Devrait afficher 2.2.0 ou supérieur
```

---

## 📞 Support

- **Issues GitHub :** https://github.com/Yannisloum63/yannis-open_vins/issues
- **Documentation OpenVINS :** https://docs.openvins.com/
- **Projet original :** https://github.com/rpng/open_vins

---

## 🎓 Points clés découverts durant le développement

### Problème 1 : Mask obligatoire
**Erreur :** `cv::Exception: !ssize.empty() in function 'resize'`  
**Solution :** Utiliser `cv::Mat::zeros(rows, cols, CV_8UC1)` au lieu de `cv::Mat()` vide

```cpp
// ❌ ERREUR
cam_msg.masks.push_back(cv::Mat());

// ✅ CORRECT
cam_msg.masks.push_back(cv::Mat::zeros(img.rows, img.cols, CV_8UC1));
```

### Problème 2 : Parsing CSV avec whitespace
**Erreur :** Noms de fichiers avec retours à la ligne  
**Solution :** Supprimer les espaces/retours à la ligne

```cpp
// Nettoyer le filename
filename.erase(filename.find_last_not_of(" \n\r\t") + 1);
```

### Problème 3 : Configuration YAML
**Erreur :** `got 0 but expected 1 max cameras`  
**Solution :** Utiliser `YamlParser` et `params.print_and_load(parser)` au lieu de setter manuellement

```cpp
auto parser = std::make_shared<ov_core::YamlParser>(config_path);
params.print_and_load(parser);
```

### Problème 4 : Format image
**Choix :** `IMREAD_GRAYSCALE` (équivalent à ROS `MONO8`)

```cpp
cv::Mat img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);
```

---

## 📁 Structure finale des résultats

```
open_vins/
├── results/
│   └── euroc_mh01_validation/
│       ├── ape_plot.pdf              # Comparaison trajectoires
│       ├── trajectory_3d.pdf         # Visualisation 3D
│       ├── groundtruth.txt           # Ground truth converti
│       └── README.md                 # Métriques détaillées
│
└── examples_integration/
    └── build/
        └── trajectory_estimated.txt  # Résultat de votre run
```

---

## 🏆 Résultats finaux

| Métrique | Valeur | Interprétation |
|----------|--------|----------------|
| **RMSE** | **8.6 cm** | ✅ Excellent |
| Erreur moyenne | 7.4 cm | Précision constante |
| Erreur médiane | 6.6 cm | Majorité < 7cm |
| Trajectoire | 62.8 m | Distance parcourue |
| Durée | 113 s | Temps traitement |

**Comparaison benchmarks :**
- VINS-Mono : ~10-15 cm
- ORB-SLAM3 : ~5-10 cm
- **Notre OpenVINS** : **8.6 cm** ← Comparable aux meilleurs !

---

**Dernière mise à jour :** 30 novembre 2025
