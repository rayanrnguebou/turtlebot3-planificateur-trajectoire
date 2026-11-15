# Planificateur de trajectoire TurtleBot3 : phase 1 (algorithmes)

Python pur, sans ROS : tout fonctionne directement sous Windows.
Ces modules seront réutilisés tels quels dans les nœuds ROS 2 des phases 3 et 4.

## Installation (PowerShell, Windows)

1. Installe Python 3.11 ou 3.12 depuis python.org (coche « Add python.exe to PATH »).
2. Dans le dossier du projet :

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

Si PowerShell refuse d'activer l'environnement :
`Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`, puis recommence.

## Utilisation

```powershell
python demo.py                                  # A*, Dijkstra, RRT sur la carte « murs »
python demo.py --carte labyrinthe
python demo.py --carte aleatoire --graine 3
python demo.py --carte murs --rayon 0.105       # obstacles gonflés du rayon du Burger
python comparaison.py --csv resultats.csv       # benchmark pour le rapport
python -m pytest                                # 18 tests
```

Cartes disponibles : `vide`, `murs`, `labyrinthe`, `aleatoire`.
Des images d'exemple et un CSV de comparaison sont dans `exemples/`.

## Organisation

| Fichier | Contenu |
| --- | --- |
| `algos/grille.py` | Carte d'occupation (format ROS : 0 libre, 100 obstacle, -1 inconnu), voisins 8-connexes, gonflement, conversion cases ↔ mètres, cartes de test |
| `algos/recherche.py` | Recherche du meilleur d'abord, commune à A* et Dijkstra, et heuristiques |
| `algos/astar.py`, `algos/dijkstra.py` | Les deux algorithmes (seule l'heuristique change) |
| `algos/rrt.py` | RRT avec biais vers le but, et lissage de chemin par raccourcis |
| `demo.py` | Visualisation côte à côte |
| `comparaison.py` | Benchmark : succès, longueur, longueur lissée, nœuds, temps |
| `tests/` | Tests pytest |

## Ce que montrent les premiers résultats (5 essais par carte)

- **A\* et Dijkstra trouvent toujours le même chemin optimal**, mais A\* explore
  2 à 14 fois moins de cases sur les cartes ouvertes (vide : 328 contre 4 524).
  Dans un labyrinthe, l'écart disparaît : l'heuristique ne peut pas « voir » les murs.
- **RRT est rapide sur les cartes ouvertes mais donne des chemins plus longs**
  (+11 à +42 %). Le lissage rattrape l'essentiel de l'écart.
- **RRT échoue souvent dans les passages étroits** (labyrinthe : 20 % de succès,
  0 % avec des couloirs de 1 case). C'est une limite connue de RRT, à discuter
  dans le rapport.

Pour le rapport, compare toujours les longueurs sur les mêmes cartes : la
moyenne RRT du labyrinthe ne porte que sur les essais réussis.

## Pistes pour la suite

- RRT* (version optimale de RRT) pour comparer avec A\*.
- Charger la carte produite par le SLAM (fichiers `.pgm` + `.yaml` de ROS).
- Brancher `astar()` dans un nœud ROS 2 qui lit `/map` et publie un `nav_msgs/Path`.
