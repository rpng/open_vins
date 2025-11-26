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

```bash
cd ~/workspace/open_vins/examples_integration
mkdir build && cd build
cmake ..
make
```

## ▶️ Exécution

```bash
# Sans configuration (paramètres par défaut)
./build/minimal_vio_example

# Avec fichier de configuration YAML
./build/minimal_vio_example ../config/euroc_mav/estimator_config.yaml
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
