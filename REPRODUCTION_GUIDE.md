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

## 🔄 Étape 7 : Workflow Git pour vos modifications

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

**Dernière mise à jour :** 26 novembre 2025
