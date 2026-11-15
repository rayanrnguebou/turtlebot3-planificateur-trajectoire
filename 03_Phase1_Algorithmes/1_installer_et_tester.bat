@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo === Phase 1 : installation de l'environnement Python ===
if not exist .venv (
    python -m venv .venv || goto erreur
)
call .venv\Scripts\activate.bat
python -m pip install --upgrade pip >nul
pip install -r requirements.txt || goto erreur
echo.
echo === Tests automatiques ===
python -m pytest -q || goto erreur
echo.
echo === Benchmark des algorithmes (resultats dans resultats.csv) ===
python comparaison.py --csv resultats.csv
echo.
echo Tout est OK. Lance maintenant 2_demo.bat pour voir les algorithmes.
pause
exit /b 0

:erreur
echo.
echo ERREUR : lis le message ci-dessus (souvent : Python absent du PATH ou pas d acces Internet).
echo Si "python" est introuvable : installe Python 3.12 depuis python.org
echo en cochant "Add python.exe to PATH".
pause
exit /b 1
