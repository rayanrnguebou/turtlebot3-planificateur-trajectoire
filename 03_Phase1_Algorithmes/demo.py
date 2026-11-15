"""Affiche A*, Dijkstra et RRT côte à côte sur une carte.

Exemples (dans PowerShell, depuis ce dossier) :
    python demo.py
    python demo.py --carte labyrinthe
    python demo.py --carte aleatoire --graine 3 --sauver resultat.png
"""

from __future__ import annotations

import argparse

import matplotlib.pyplot as plt
import numpy as np

from algos import CARTES, PLANIFICATEURS, lisser
from algos.grille import extremites


def dessiner(ax, grille, res, depart, arrivee, chemin_lisse=None):
    ax.imshow(grille.donnees != 0, cmap="Greys", origin="lower", interpolation="nearest")
    if res.explores:
        e = np.array(res.explores)
        ax.scatter(e[:, 1], e[:, 0], s=2, c="#9ecae1", label="cases explorées")
    for a, b in res.arbre:
        ax.plot([a[1], b[1]], [a[0], b[0]], c="#9ecae1", lw=0.6)
    if res.trouve:
        c = np.array(res.chemin)
        ax.plot(c[:, 1], c[:, 0], c="#d62728", lw=2, label="chemin")
    if chemin_lisse:
        c = np.array(chemin_lisse)
        ax.plot(c[:, 1], c[:, 0], c="#2ca02c", lw=2, ls="--", label="chemin lissé")
    ax.plot(depart[1], depart[0], "o", c="#2ca02c", ms=8)
    ax.plot(arrivee[1], arrivee[0], "*", c="#ff7f0e", ms=12)
    statut = (f"longueur {res.cout:.1f} | {res.noeuds_explores} nœuds | {res.temps_s * 1000:.1f} ms"
              if res.trouve else "aucun chemin trouvé")
    ax.set_title(f"{res.algo}\n{statut}", fontsize=10)
    ax.set_xticks([])
    ax.set_yticks([])


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--carte", choices=CARTES, default="murs")
    p.add_argument("--graine", type=int, default=0, help="graine de la carte aléatoire et de RRT")
    p.add_argument("--rayon", type=float, default=0.0,
                   help="gonfle les obstacles de ce rayon en mètres (Burger : 0.105)")
    p.add_argument("--sauver", help="enregistre l'image au lieu de l'afficher")
    args = p.parse_args()

    fabrique = CARTES[args.carte]
    grille = fabrique(graine=args.graine) if args.carte in ("labyrinthe", "aleatoire") else fabrique()
    if args.rayon:
        grille = grille.gonfler(args.rayon)
    depart, arrivee = extremites(grille)

    fig, axes = plt.subplots(1, len(PLANIFICATEURS), figsize=(5 * len(PLANIFICATEURS), 4.5))
    for ax, (nom, planif) in zip(axes, PLANIFICATEURS.items()):
        res = planif(grille, depart, arrivee, graine=args.graine) if nom == "RRT" else planif(grille, depart, arrivee)
        lisse = lisser(grille, res.chemin) if res.trouve else None
        dessiner(ax, grille, res, depart, arrivee, lisse)
    for ax in axes:
        if ax.get_legend_handles_labels()[0]:
            ax.legend(loc="upper left", fontsize=8)
    fig.suptitle(f"Carte « {args.carte} » ({grille.hauteur}×{grille.largeur} cases)")
    fig.tight_layout(rect=(0, 0, 1, 0.92))

    if args.sauver:
        fig.savefig(args.sauver, dpi=120)
        print(f"Image enregistrée : {args.sauver}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
