#!/usr/bin/env python3
"""
Génération d'un PDF avec un graphique 3D zoomé pour poster
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

def read_tum_format(filename):
    """Lit un fichier TUM: timestamp tx ty tz qx qy qz qw"""
    data = np.loadtxt(filename)
    return data[:, 0], data[:, 1:4], data[:, 4:8]

def match_by_time(ts_a, ts_b, max_dt=0.01):
    """
    Apparier deux séquences par plus proche voisin temporel (max_dt en secondes).
    Retourne des index i (dans A) et j (dans B) appariés.
    """
    i = j = 0
    idx_a, idx_b = [], []
    while i < len(ts_a) and j < len(ts_b):
        dt = ts_a[i] - ts_b[j]
        if abs(dt) <= max_dt:
            idx_a.append(i)
            idx_b.append(j)
            i += 1
            j += 1
        elif dt > 0:
            j += 1
        else:
            i += 1
    return np.array(idx_a, dtype=int), np.array(idx_b, dtype=int)

def umeyama_se3(X, Y):
    """
    Estime R,t (SE3) tel que Y ≈ R X + t (sans échelle).
    X, Y: (N,3) points appariés.
    """
    Xm = X.mean(axis=0)
    Ym = Y.mean(axis=0)
    Xc = X - Xm
    Yc = Y - Ym
    H = Xc.T @ Yc
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = Ym - R @ Xm
    return R, t

def compute_metrics(est_pos, gt_pos):
    errors = np.linalg.norm(est_pos - gt_pos, axis=1)
    metrics = {
        'mean': float(np.mean(errors)),
        'median': float(np.median(errors)),
        'std': float(np.std(errors)),
        'rmse': float(np.sqrt(np.mean(errors**2))),
        'max': float(np.max(errors)),
        'min': float(np.min(errors))
    }
    return errors, metrics

def create_pdf_plot(est_file, gt_file, output_pdf):
    print(f"📂 Lecture trajectoire estimée : {est_file}")
    ts_est, pos_est_full, _ = read_tum_format(est_file)
    print(f"📂 Lecture ground truth : {gt_file}")
    ts_gt, pos_gt_full, _ = read_tum_format(gt_file)

    print(f"✅ Estimée (total): {len(pos_est_full)} poses")
    print(f"✅ Ground truth (total): {len(pos_gt_full)} poses")

    # Apparier temporellement (10 ms par défaut)
    idx_e, idx_g = match_by_time(ts_est, ts_gt, max_dt=0.01)
    if len(idx_e) < 10:
        print("❌ Trop peu de correspondances temporelles (<10). Augmentez max_dt.")
        sys.exit(1)
    pos_est = pos_est_full[idx_e]
    pos_gt = pos_gt_full[idx_g]
    ts_rel = ts_est[idx_e] - ts_est[idx_e][0]
    print(f"🔗 Correspondances temporelles: {len(idx_e)} paires")

    # Alignement SE(3)
    R, t = umeyama_se3(pos_est, pos_gt)
    pos_est_aligned = (pos_est @ R.T) + t
    print("📐 Alignement SE(3) (sans échelle) calculé.")

    # Métriques
    errors, metrics = compute_metrics(pos_est_aligned, pos_gt)
    print(f"\n📊 Métriques (après alignement SE3 + sync):")
    print(f"  - RMSE    : {metrics['rmse']*100:.2f} cm")
    print(f"  - Mean    : {metrics['mean']*100:.2f} cm")
    print(f"  - Median  : {metrics['median']*100:.2f} cm")
    print(f"  - Std Dev : {metrics['std']*100:.2f} cm")
    print(f"  - Max     : {metrics['max']*100:.2f} cm")
    print(f"  - Min     : {metrics['min']*100:.2f} cm")

    # Sous-échantillonner pour affichage
    ds_est = max(1, len(pos_est_full)//2000)
    ds_gt = max(1, len(pos_gt_full)//3000)
    pos_est_vis = (pos_est_full[::ds_est] @ R.T) + t
    pos_gt_vis = pos_gt_full[::ds_gt]

    # Créer figure avec 4 sous-graphiques
    fig = plt.figure(figsize=(16, 10))
    
    # 1. Graphique 3D principal (grand, zoomé)
    ax1 = fig.add_subplot(2, 2, (1, 3), projection='3d')
    
    # Tracer les trajectoires
    ax1.plot(pos_gt_vis[:, 0], pos_gt_vis[:, 1], pos_gt_vis[:, 2], 
             'r-', linewidth=3, label='Ground Truth', alpha=0.8)
    ax1.plot(pos_est_vis[:, 0], pos_est_vis[:, 1], pos_est_vis[:, 2], 
             'b-', linewidth=2.5, label='Estimée (alignée)', alpha=0.9)
    
    # Points de départ et fin
    ax1.scatter([pos_est_vis[0, 0]], [pos_est_vis[0, 1]], [pos_est_vis[0, 2]], 
                c='green', s=200, marker='o', label='Départ', edgecolors='black', linewidth=2)
    ax1.scatter([pos_est_vis[-1, 0]], [pos_est_vis[-1, 1]], [pos_est_vis[-1, 2]], 
                c='orange', s=200, marker='s', label='Arrivée', edgecolors='black', linewidth=2)
    
    ax1.set_xlabel('X (m)', fontsize=14, fontweight='bold')
    ax1.set_ylabel('Y (m)', fontsize=14, fontweight='bold')
    ax1.set_zlabel('Z (m)', fontsize=14, fontweight='bold')
    ax1.set_title('Trajectoires 3D Alignées', fontsize=16, fontweight='bold', pad=20)
    ax1.legend(fontsize=12, loc='upper right')
    ax1.grid(True, alpha=0.3)
    
    # Ajuster la vue (angle d'élévation et azimut)
    ax1.view_init(elev=25, azim=45)
    
    # 2. Erreur vs Temps
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(ts_rel, errors * 100, 'purple', linewidth=2, label='Erreur')
    ax2.axhline(y=metrics['rmse'] * 100, color='r', linestyle='--', 
                linewidth=2, label=f"RMSE: {metrics['rmse']*100:.2f} cm")
    ax2.set_xlabel('Temps (s)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Erreur (cm)', fontsize=12, fontweight='bold')
    ax2.set_title('Erreur vs Temps', fontsize=14, fontweight='bold')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)
    
    # 3. Histogramme des erreurs
    ax3 = fig.add_subplot(2, 2, 4)
    ax3.hist(errors * 100, bins=40, color='steelblue', edgecolor='black', alpha=0.7)
    ax3.axvline(x=metrics['rmse'] * 100, color='r', linestyle='--', 
                linewidth=2, label=f"RMSE: {metrics['rmse']*100:.2f} cm")
    ax3.set_xlabel('Erreur (cm)', fontsize=12, fontweight='bold')
    ax3.set_ylabel('Fréquence', fontsize=12, fontweight='bold')
    ax3.set_title('Distribution des Erreurs', fontsize=14, fontweight='bold')
    ax3.legend(fontsize=10)
    ax3.grid(True, alpha=0.3, axis='y')
    
    # Titre global
    fig.suptitle(
        f'Analyse Trajectoire VIO - Alignement SE(3)\n'
        f'RMSE: {metrics["rmse"]*100:.2f} cm | Mean: {metrics["mean"]*100:.2f} cm | '
        f'Median: {metrics["median"]*100:.2f} cm | Paires: {len(idx_e)}',
        fontsize=16, fontweight='bold', y=0.98
    )
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    # Sauvegarder les graphiques individuels
    output_dir = os.path.dirname(output_pdf) or '.'
    base_name = os.path.splitext(os.path.basename(output_pdf))[0]
    
    # Graphique 3D seul
    print(f"\n💾 Sauvegarde des graphiques individuels...")
    fig_3d = plt.figure(figsize=(12, 10))
    ax_3d = fig_3d.add_subplot(111, projection='3d')
    ax_3d.plot(pos_gt_vis[:, 0], pos_gt_vis[:, 1], pos_gt_vis[:, 2], 
               'r-', linewidth=3, label='Ground Truth', alpha=0.8)
    ax_3d.plot(pos_est_vis[:, 0], pos_est_vis[:, 1], pos_est_vis[:, 2], 
               'b-', linewidth=2.5, label='Estimée (alignée)', alpha=0.9)
    ax_3d.scatter([pos_est_vis[0, 0]], [pos_est_vis[0, 1]], [pos_est_vis[0, 2]], 
                  c='green', s=200, marker='o', label='Départ', edgecolors='black', linewidth=2)
    ax_3d.scatter([pos_est_vis[-1, 0]], [pos_est_vis[-1, 1]], [pos_est_vis[-1, 2]], 
                  c='orange', s=200, marker='s', label='Arrivée', edgecolors='black', linewidth=2)
    ax_3d.set_xlabel('X (m)', fontsize=16, fontweight='bold')
    ax_3d.set_ylabel('Y (m)', fontsize=16, fontweight='bold')
    ax_3d.set_zlabel('Z (m)', fontsize=16, fontweight='bold')
    ax_3d.set_title('Trajectoires 3D Alignées', fontsize=18, fontweight='bold', pad=20)
    ax_3d.legend(fontsize=14, loc='upper right')
    ax_3d.grid(True, alpha=0.3)
    ax_3d.view_init(elev=25, azim=45)
    plt.tight_layout()
    fig_3d.savefig(f'{output_dir}/{base_name}_3d.png', dpi=300, bbox_inches='tight')
    plt.close(fig_3d)
    print(f"   ✅ {base_name}_3d.png")
    
    # Erreur vs Temps seul
    fig_err = plt.figure(figsize=(10, 6))
    ax_err = fig_err.add_subplot(111)
    ax_err.plot(ts_rel, errors * 100, 'purple', linewidth=2.5, label='Erreur')
    ax_err.axhline(y=metrics['rmse'] * 100, color='r', linestyle='--', 
                   linewidth=2.5, label=f"RMSE: {metrics['rmse']*100:.2f} cm")
    ax_err.set_xlabel('Temps (s)', fontsize=14, fontweight='bold')
    ax_err.set_ylabel('Erreur (cm)', fontsize=14, fontweight='bold')
    ax_err.set_title('Erreur vs Temps', fontsize=16, fontweight='bold')
    ax_err.legend(fontsize=12)
    ax_err.grid(True, alpha=0.3)
    plt.tight_layout()
    fig_err.savefig(f'{output_dir}/{base_name}_error_time.png', dpi=300, bbox_inches='tight')
    plt.close(fig_err)
    print(f"   ✅ {base_name}_error_time.png")
    
    # Histogramme seul
    fig_hist = plt.figure(figsize=(10, 6))
    ax_hist = fig_hist.add_subplot(111)
    ax_hist.hist(errors * 100, bins=40, color='steelblue', edgecolor='black', alpha=0.7)
    ax_hist.axvline(x=metrics['rmse'] * 100, color='r', linestyle='--', 
                    linewidth=2.5, label=f"RMSE: {metrics['rmse']*100:.2f} cm")
    ax_hist.set_xlabel('Erreur (cm)', fontsize=14, fontweight='bold')
    ax_hist.set_ylabel('Fréquence', fontsize=14, fontweight='bold')
    ax_hist.set_title('Distribution des Erreurs', fontsize=16, fontweight='bold')
    ax_hist.legend(fontsize=12)
    ax_hist.grid(True, alpha=0.3, axis='y')
    plt.tight_layout()
    fig_hist.savefig(f'{output_dir}/{base_name}_histogram.png', dpi=300, bbox_inches='tight')
    plt.close(fig_hist)
    print(f"   ✅ {base_name}_histogram.png")
    
    print(f"\n💾 Sauvegarde {output_pdf}...")
    plt.savefig(output_pdf, dpi=300, bbox_inches='tight', format='pdf')
    print(f"✅ Fichier PDF généré : {output_pdf}")
    plt.close()

if __name__ == "__main__":
    est_file = sys.argv[1] if len(sys.argv) >= 2 else "trajectory_estimated.txt"
    gt_file = sys.argv[2] if len(sys.argv) >= 3 else "groundtruth.txt"
    output_pdf = sys.argv[3] if len(sys.argv) >= 4 else "trajectory_comparison.pdf"

    if not os.path.exists(est_file):
        print(f"❌ {est_file} introuvable")
        sys.exit(1)
    if not os.path.exists(gt_file):
        print(f"❌ {gt_file} introuvable")
        sys.exit(1)

    create_pdf_plot(est_file, gt_file, output_pdf)
