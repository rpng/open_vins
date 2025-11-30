# Résultats validation EuRoC MH_01_easy

## 📊 Métriques de performance

Date de test : 27-28 Novembre 2025  
Dataset : EuRoC Machine Hall 01 (easy)  
Configuration : Stereo (cam0 + cam1), IMU 200Hz, Camera 20Hz

### Erreur Absolue de Trajectoire (APE)

| Métrique | Valeur |
|----------|--------|
| **RMSE** | **8.6 cm** |
| Erreur moyenne | 7.4 cm |
| Erreur médiane | 6.6 cm |
| Erreur max | 29.5 cm |
| Erreur min | 1.6 cm |

### Statistiques de trajectoire

- **Poses estimées** : 2263
- **Longueur parcourue** : 62.756 mètres
- **Durée** : 113.1 secondes
- **Vitesse moyenne** : 0.55 m/s

## 📁 Fichiers

- `ape_plot.pdf` - Comparaison trajectoire estimée vs ground truth
- `ape_results.zip` - Résultats numériques détaillés (format evo)
- `groundtruth.txt` - Ground truth EuRoC converti au format TUM
- `trajectory_3d.pdf` - Visualisation 3D de la trajectoire
- `trajectory_xyz_components.pdf` - Graphiques des composantes x,y,z

## 🎯 Interprétation

**Précision relative** : 8.6cm / 62.8m = **0.14%** d'erreur

**Comparaison benchmarks** :
- VINS-Mono : ~10-15 cm RMSE
- ORB-SLAM3 : ~5-10 cm RMSE
- **Notre OpenVINS** : **8.6 cm RMSE** ← Excellent résultat

## 🔗 Références

- Code utilisé : `examples_integration/euroc_reader_example.cpp`
- Configuration : `config/euroc_mav/estimator_config.yaml`
- Dataset : http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/
- Documentation complète : [SESSION_SUMMARY.md](../../SESSION_SUMMARY.md)
