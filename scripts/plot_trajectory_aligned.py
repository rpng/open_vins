#!/usr/bin/env python3
"""
Visualisation 3D interactive de trajectoires VIO avec ALIGNEMENT et SYNC TEMPOREL
Alignement SE(3) (rotation + translation, sans échelle) après appariement temporel.
"""

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
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

def create_interactive_plot(est_file, gt_file, output_html):
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

    # Pour le tracé 3D, on peut sous-échantillonner visuellement les courbes complètes
    ds_est = max(1, len(pos_est_full)//3000)
    ds_gt = max(1, len(pos_gt_full)//4000)
    pos_est_vis = (pos_est_full[::ds_est] @ R.T) + t  # appliquer alignement à toute la courbe
    pos_gt_vis = pos_gt_full[::ds_gt]

    fig = make_subplots(
        rows=2, cols=2,
        specs=[[{'type': 'scatter3d', 'rowspan': 2}, {'type': 'xy'}],
               [None, {'type': 'xy'}]],
        subplot_titles=(
            '🎯 Trajectoires 3D Alignées (Interactive)',
            '📈 Erreur vs Temps (appairé)',
            '📊 Distribution des erreurs'
        ),
        column_widths=[0.65, 0.35],
        row_heights=[0.5, 0.5]
    )

    # Ground truth (rouge)
    fig.add_trace(
        go.Scatter3d(x=pos_gt_vis[:,0], y=pos_gt_vis[:,1], z=pos_gt_vis[:,2],
                      mode='lines', name='Ground Truth',
                      line=dict(color='rgba(255,0,0,0.8)', width=10)),
        row=1, col=1)

    # Estimée alignée (bleu)
    fig.add_trace(
        go.Scatter3d(x=pos_est_vis[:,0], y=pos_est_vis[:,1], z=pos_est_vis[:,2],
                      mode='lines', name='Estimée (alignée)',
                      line=dict(color='rgba(0,100,255,1.0)', width=8)),
        row=1, col=1)

    # Erreur temporelle
    fig.add_trace(
        go.Scatter(x=ts_rel, y=errors*100, mode='lines', name='Erreur (cm)',
                   line=dict(color='purple', width=2)),
        row=1, col=2)

    # Ligne RMSE
    fig.add_trace(
        go.Scatter(x=[ts_rel[0], ts_rel[-1]], y=[metrics['rmse']*100, metrics['rmse']*100],
                   mode='lines', name=f"RMSE {metrics['rmse']*100:.1f} cm",
                   line=dict(color='red', width=2, dash='dash')),
        row=1, col=2)

    # Histogramme
    fig.add_trace(
        go.Histogram(x=errors*100, nbinsx=40, name='Distribution',
                     marker=dict(color='steelblue')),
        row=2, col=2)

    # Calculer le centre et l'étendue de la trajectoire pour le zoom
    all_points = np.vstack([pos_gt_vis, pos_est_vis])
    center = all_points.mean(axis=0)
    ranges = all_points.max(axis=0) - all_points.min(axis=0)
    max_range = ranges.max()
    
    # Caméra zoomée (eye plus proche = plus de zoom)
    camera = dict(
        eye=dict(x=0.8, y=0.8, z=0.7),  # Position de la caméra (plus proche = plus zoomé)
        center=dict(x=0, y=0, z=0),
        up=dict(x=0, y=0, z=1)
    )

    fig.update_layout(
        title=dict(text=(
            f"<b>🚁 Analyse Trajectoire VIO - Alignement SE3 + Sync</b><br>"
            f"<sub>RMSE: {metrics['rmse']*100:.2f} cm | Mean: {metrics['mean']*100:.2f} cm | "
            f"Median: {metrics['median']*100:.2f} cm | Paires: {len(idx_e)}</sub>"
        ), x=0.5, xanchor='center', font=dict(size=18)),
        height=900, showlegend=True, template='plotly_white',
        font=dict(size=14))

    fig.update_scenes(
        xaxis_title='X (m)', 
        yaxis_title='Y (m)', 
        zaxis_title='Z (m)', 
        aspectmode='data',
        camera=camera,
        xaxis=dict(title_font=dict(size=16)),
        yaxis=dict(title_font=dict(size=16)),
        zaxis=dict(title_font=dict(size=16))
    )
    fig.update_xaxes(title_text='Temps (s)', row=1, col=2)
    fig.update_yaxes(title_text='Erreur (cm)', row=1, col=2)
    fig.update_xaxes(title_text='Erreur (cm)', row=2, col=2)
    fig.update_yaxes(title_text='Fréquence', row=2, col=2)

    print(f"\n💾 Sauvegarde {output_html}...")
    fig.write_html(output_html, config={'displayModeBar': True, 'displaylogo': False})
    print(f"✅ Fichier HTML généré : {output_html}")

if __name__ == "__main__":
    est_file = sys.argv[1] if len(sys.argv) >= 2 else "trajectory_estimated.txt"
    gt_file = sys.argv[2] if len(sys.argv) >= 3 else "groundtruth.txt"
    output_html = sys.argv[3] if len(sys.argv) >= 4 else "trajectory_3d_aligned.html"

    if not os.path.exists(est_file):
        print(f"❌ {est_file} introuvable")
        sys.exit(1)
    if not os.path.exists(gt_file):
        print(f"❌ {gt_file} introuvable")
        sys.exit(1)

    create_interactive_plot(est_file, gt_file, output_html)
