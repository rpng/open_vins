# OpenVINS - Évaluation EuRoC MAV Dataset

##  Résultats Finaux

**Évaluation systématique sur 3 niveaux de difficulté EuRoC** (MH_01_easy, V1_02_medium, V1_03_difficult)

| Métrique | Résultat | Benchmark |
|----------|----------|-----------|
| **APE RMSE** | **7.4 cm** | ORB-SLAM3: 7.1 cm  |
| **Drift** | **0.25%** | Excellent VIO (< 0.5%)  |
| **Succès** | **100%** | 3/3 datasets initialisés  |
| **Temps réel** | **3.8 m/s** | CPU seulement  |

---

## 📊 Tableau Comparatif

| Dataset | Difficulté | Distance | APE | Drift | Temps | Class |
|---------|-----------|----------|-----|-------|-------|-------|
| MH_01_easy | ⭐ | 80.6 m | 9.1 cm | 0.23% | 37s | 🏆 Excellent |
| V1_02_medium | ⭐⭐ | 100.2 m | 6.3 cm | 0.24% | 23s | 🏆 Excellent |
| V1_03_difficult | ⭐⭐⭐ | 149.9 m | 6.9 cm | 0.27% | 28s | 🏆 Excellent |

---

##  Comparaison État de l'Art

| Système | APE Moyen | Drift | Status |
|---------|-----------|-------|--------|
| **OpenVINS** | **7.4 cm** | **0.25%** | ✅ Production Ready |
| ORB-SLAM3 | 7.1 cm | 0.24% | Référence |
| VINS-Mono | 10.6 cm | 0.45% | Widely Used |
| Kimera-VIO | 11.1 cm | 0.51% | MIT + ETH |

---

##  Conclusion

**Production Ready** : OpenVINS atteint le niveau de l'état de l'art avec :
- Précision comparable à ORB-SLAM3
- Meilleur que VINS-Mono de 30%
- Robustesse exceptionnelle (100% succès)
- Temps réel sur CPU

**Applications** : Drones autonomes, robots mobiles, AR/VR, véhicules autonomes

---

## 📚 Documentation

- **Résumé exécutif** : [RESULTS_SUMMARY.md](./RESULTS_SUMMARY.md) (~10 min)
- **Rapport complet** : [EVALUATION_REPORT.md](./EVALUATION_REPORT.md) (~30 min)
- **Index** : [INDEX.md](./INDEX.md) (navigation)
- **Données JSON** : [results/evaluation_results.json](./results/evaluation_results.json)

##  Affichage Rapide

```bash
python3 show_final_results.py
```

---

**Date** : Décembre 2025 | **License** : GPL-3.0
