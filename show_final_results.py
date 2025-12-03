#!/usr/bin/env python3
"""
Script d'affichage des résultats finaux OpenVINS sur EuRoC
"""

import json

def print_header():
    print('\n' + '='*100)
    print(' '*25 + '🏆 OPENVINS - ÉVALUATION FINALE SUR EUROC MAV DATASET 🏆')
    print('='*100 + '\n')

def print_summary(data):
    summary = data['summary_statistics']
    
    print('📊 SYNTHÈSE GLOBALE')
    print('─'*100)
    print(f"   Datasets évalués        : {summary['total_datasets']}")
    print(f"   Taux de succès          : {summary['success_rate']*100:.0f}% ({summary['total_datasets']}/{summary['total_datasets']})")
    print(f"   Distance totale         : {summary['total_distance_m']:.1f} m")
    print(f"   Temps de traitement     : {summary['total_processing_time_s']} secondes")
    print(f"   Vitesse de traitement   : {summary['processing_speed_ms']:.1f} m/s")
    print(f"   Images traitées         : {summary['total_images_processed']:,}")
    print(f"   Poses estimées          : {summary['total_poses_estimated']:,}")
    print('─'*100)
    print(f"   🎯 APE RMSE moyen        : {summary['average_ape_rmse_cm']:.2f} cm")
    print(f"   🎯 Drift moyen           : {summary['average_drift_percent']:.2f}%")
    print(f"   🎯 Classification        : {summary['classification']}")
    print(f"   🎯 Robustesse            : Dégradation = {summary['degradation_easy_to_difficult_cm']:.1f} cm")
    print('─'*100 + '\n')

def print_datasets_comparison(data):
    print('📈 COMPARAISON PAR DATASET')
    print('─'*100)
    
    # Header
    header = f"{'Dataset':<20} {'Difficulté':<12} {'Distance':>10} {'APE':>8} {'Drift':>8} {'Temps':>8} {'Class':<15}"
    print(header)
    print('─'*100)
    
    # Data rows
    for ds in data['datasets']:
        name = ds['name']
        diff = '⭐' * ds['difficulty_stars']
        dist = f"{ds['dataset_info']['distance_m']:.1f} m"
        ape = f"{ds['metrics']['ape']['rmse_cm']:.1f} cm"
        drift = f"{ds['metrics']['rpe_10m']['drift_percent']:.2f}%"
        time = f"{ds['dataset_info']['processing_time_s']} s"
        classif = ds['classification']
        
        print(f"{name:<20} {diff:<12} {dist:>10} {ape:>8} {drift:>8} {time:>8} {classif:<15}")
    
    print('─'*100 + '\n')

def print_state_of_art(data):
    print('🏆 COMPARAISON AVEC L\'ÉTAT DE L\'ART')
    print('─'*100)
    
    print(f"{'Système':<15} {'APE Moyen':>12} {'Drift Moyen':>14} {'Type':>20} {'Status':<20}")
    print('─'*100)
    
    sota = data['state_of_the_art_comparison']
    for system, metrics in sota.items():
        name = system
        ape = f"{metrics['ape_mean_cm']:.2f} cm"
        drift = f"{metrics['drift_mean_percent']:.2f}%"
        typ = metrics['type']
        status = metrics['status']
        
        marker = '✅' if system == 'OpenVINS' else '  '
        print(f"{marker} {name:<13} {ape:>12} {drift:>14} {typ:>20} {status:<20}")
    
    print('─'*100 + '\n')

def print_key_findings(data):
    print('💡 OBSERVATIONS CLÉS')
    print('─'*100)
    
    obs = data['key_observations']
    
    print('1️⃣  RÉSULTAT CONTRE-INTUITIF:')
    print(f"    {obs['counter_intuitive_result']['observation']}")
    print('    Hypothèses explicatives:')
    for i, hyp in enumerate(obs['counter_intuitive_result']['hypotheses'], 1):
        print(f"      • {hyp}")
    print()
    
    print('2️⃣  ROBUSTESSE:')
    print(f"    • Dégradation facile→difficile: {obs['robustness']['degradation_easy_to_difficult']}")
    print(f"    • Taux de succès: {obs['robustness']['success_rate']}")
    print(f"    • Consistance: {obs['robustness']['consistency']}")
    print()
    
    print('3️⃣  PERFORMANCE:')
    print(f"    • Comparable à: {obs['performance']['comparable_to']}")
    print(f"    • Meilleur que: {obs['performance']['better_than']}")
    print(f"    • Temps réel: {obs['performance']['real_time']}")
    print('─'*100 + '\n')

def print_applications(data):
    print('🚀 APPLICATIONS RECOMMANDÉES')
    print('─'*100)
    
    for i, app in enumerate(data['recommended_applications'], 1):
        print(f"{i}. {app['domain']}")
        print(f"   Scénario: {app['scenario']}")
        print(f"   Budget d'erreur: {app['error_budget']}")
        print()
    
    print('─'*100 + '\n')

def print_conclusion(data):
    conclusion = data['conclusion']
    
    print('✅ CONCLUSION FINALE')
    print('─'*100)
    print(f"Rating global: {conclusion['overall_rating']}")
    print(f"Production Ready: {'✅ OUI' if conclusion['production_ready'] else '❌ NON'}")
    print()
    print("Points forts:")
    for strength in conclusion['key_strengths']:
        print(f"  ✓ {strength}")
    print()
    print(f"Recommandation: {conclusion['recommendation']}")
    print('─'*100 + '\n')

def print_files_location():
    print('📁 FICHIERS GÉNÉRÉS')
    print('─'*100)
    print('  📄 EVALUATION_REPORT.md        - Rapport détaillé complet (335 lignes)')
    print('  📄 RESULTS_SUMMARY.md          - Résumé exécutif (232 lignes)')
    print('  📄 results/evaluation_results.json - Données machine-readable')
    print('  📂 results/euroc_mh_01_easy/   - Résultats MH_01_easy')
    print('  📂 results/euroc_v1_02_medium/ - Résultats V1_02_medium')
    print('  📂 results/euroc_v1_03_difficult/ - Résultats V1_03_difficult')
    print('─'*100 + '\n')

def main():
    # Charger les données JSON
    with open('results/evaluation_results.json', 'r') as f:
        data = json.load(f)
    
    # Afficher tous les résultats
    print_header()
    print_summary(data)
    print_datasets_comparison(data)
    print_state_of_art(data)
    print_key_findings(data)
    print_applications(data)
    print_conclusion(data)
    print_files_location()
    
    print('='*100)
    print(' '*20 + '🎉 Évaluation terminée avec succès - Tous les résultats sont disponibles 🎉')
    print('='*100 + '\n')

if __name__ == '__main__':
    main()
