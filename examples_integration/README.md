# Exemples d'intégration OpenVINS - Projet IMT-bcom

## 📁 Contenu

### `minimal_vio_example.cpp`
Exemple minimal montrant comment utiliser OpenVINS en mode ROS-free.

**Démontre :**
- ✅ Initialisation de `VioManager`
- ✅ Alimentation avec données IMU (`feed_measurement_imu`)
- ✅ Alimentation avec images caméra (`feed_measurement_camera`)
- ✅ Récupération de la pose estimée (`get_state()`)
- 📡 Point d'intégration avec Overview (TODO)

## 🔧 Compilation

### Prérequis
OpenVINS doit être compilé et installé (voir README principal du projet).

```bash
# Étape 1 : Compiler OpenVINS en mode ROS-free
cd ~/workspace/open_vins/ov_msckf
mkdir -p build && cd build
cmake -DENABLE_ROS=OFF ..
make -j$(nproc)
sudo make install

# Étape 2 : Compiler l'exemple d'intégration
cd ~/workspace/open_vins/examples_integration
mkdir -p build && cd build
cmake ..
make
```

## ▶️ Exécution

### Test du simulateur OpenVINS

Avant de tester l'exemple d'intégration, validez que OpenVINS fonctionne :

```bash
cd ~/workspace/open_vins
./ov_msckf/build/run_simulation config/rpng_sim/estimator_config.yaml
```

**Sortie attendue :** Affichage de la pose estimée en temps réel (position, orientation, calibration).

### Test de l'exemple d'intégration

```bash
# Depuis le dossier build de l'exemple
cd ~/workspace/open_vins/examples_integration/build

# Sans configuration (paramètres par défaut)
./minimal_vio_example

# Avec fichier de configuration YAML
./minimal_vio_example ../../config/euroc_mav/estimator_config.yaml
```

## 📊 Sortie attendue

```
=== Exemple minimal OpenVINS ROS-free ===
[INFO] Configuration par défaut (monocular)
[OK] VioManager initialisé

--- Simulation d'acquisition de données ---
[t=0.1s] Pos: [0.0 0.0 0.0] | Ori: [1.0 0.0 0.0 0.0]
...
=== Résumé ===
Mesures IMU traitées: 100
Images traitées: 20
Système initialisé: OUI
```

## 🔗 Intégration avec Overview

Le point marqué `// ICI : Envoi vers Overview` dans le code montre où transmettre les données au serveur b-com.

**Format proposé (JSON):**
```json
{
  "timestamp": 1234567890.123,
  "position": {"x": 0.5, "y": -1.2, "z": 0.0},
  "orientation": {"qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0},
  "velocity": {"vx": 0.1, "vy": 0.0, "vz": 0.0},
  "covariance": [...]  // Optionnel
}
```

## 📋 TODO

- [ ] Parser YAML pour configuration
- [ ] Implémenter classe `OverviewClient`
- [ ] Gérer reconnexion réseau
- [ ] Ajouter logs de debug
- [ ] Tester avec vraies données caméra/IMU

---

## 🧪 Validation du système

### Étapes de test complètes

```bash
# 1. Télécharger dataset EuRoC (optionnel, ~1.7GB)
mkdir -p ~/datasets && cd ~/datasets
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
unzip MH_01_easy.zip

# 2. Vérifier l'installation des bibliothèques
ldconfig -p | grep -E "(opencv|eigen|ceres|ov_msckf)"

# 3. Tester le simulateur
cd ~/workspace/open_vins
timeout 10 ./ov_msckf/build/run_simulation config/rpng_sim/estimator_config.yaml

# 4. Compiler et tester l'exemple
cd examples_integration
mkdir -p build && cd build
cmake .. && make
./minimal_vio_example
```

### Vérification de la sortie

Le simulateur doit afficher périodiquement :
```
q_GtoI = [orientation quaternion] | p_IinG = [position x,y,z] | dist = [distance]
bg = [gyro bias] | ba = [accel bias]
camera-imu timeoffset = [offset]
```

L'exemple d'intégration doit afficher :
```
[t=X.XXs] Pos: [x y z] | Ori: [qx qy qz qw]
```
