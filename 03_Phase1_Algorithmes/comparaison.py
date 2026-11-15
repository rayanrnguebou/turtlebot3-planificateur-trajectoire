"""Benchmark des planificateurs sur toutes les cartes, pour le rapport.

Chaque algorithme est lancé plusieurs fois par carte (RRT avec une graine
différente à chaque fois, car il est aléatoire). On mesure :
    - taux de succès
    - longueur du chemin (en cases), brute et après lissage
    - nombre de nœuds explorés
    - temps de calcul moyen

Usage :
    python comparaison.py                 # tableau dans la console
    python comparaison.py --csv res.csv   # + export pour Excel / le rapport
"""

from __future__ import annotations

import argparse
import csv
import statistics

from algos import CARTES, PLANIFICATEURS, lisser
from algos.grille import extremites
from algos.rrt import longueur


def lancer(repetitions: int):
    lignes = []
    for nom_carte, fabrique in CARTES.items():
        for nom_algo, planif in PLANIFICATEURS.items():
            mesures = []
            for essai in range(repetitions):
                grille = fabrique(graine=essai) if nom_carte in ("labyrinthe", "aleatoire") else fabrique()
                depart, arrivee = extremites(grille)
                if nom_algo == "RRT":
                    res = planif(grille, depart, arrivee, graine=essai)
                else:
                    res = planif(grille, depart, arrivee)
                lisse = longueur(lisser(grille, res.chemin)) if res.trouve else None
                mesures.append((res, lisse))

            reussis = [(r, l) for r, l in mesures if r.trouve]
            moy = lambda xs: statistics.mean(xs) if xs else float("nan")
            lignes.append({
                "carte": nom_carte,
                "algo": nom_algo,
                "succes_%": 100 * len(reussis) / repetitions,
                "longueur": moy([r.cout for r, _ in reussis]),
                "longueur_lissee": moy([l for _, l in reussis]),
                "noeuds": moy([r.noeuds_explores for r, _ in mesures]),
                "temps_ms": moy([r.temps_s * 1000 for r, _ in mesures]),
            })
    return lignes


def afficher(lignes):
    entetes = ["carte", "algo", "succes_%", "longueur", "longueur_lissee", "noeuds", "temps_ms"]
    print(" | ".join(f"{e:>15}" for e in entetes))
    print("-" * (18 * len(entetes)))
    for l in lignes:
        print(" | ".join(f"{l[e]:>15.1f}" if isinstance(l[e], float) else f"{l[e]:>15}" for e in entetes))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--repetitions", type=int, default=5)
    p.add_argument("--csv", help="fichier CSV de sortie")
    args = p.parse_args()

    lignes = lancer(args.repetitions)
    afficher(lignes)
    if args.csv:
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=lignes[0].keys(), delimiter=";")
            w.writeheader()
            w.writerows(lignes)
        print(f"\nRésultats exportés : {args.csv}")


if __name__ == "__main__":
    main()
