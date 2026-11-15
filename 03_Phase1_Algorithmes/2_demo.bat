@echo off
chcp 65001 >nul
cd /d "%~dp0"
if not exist .venv (
    echo Lance d'abord 1_installer_et_tester.bat
    pause
    exit /b 1
)
call .venv\Scripts\activate.bat
echo Ferme chaque fenetre pour passer a la carte suivante.
python demo.py --carte murs
python demo.py --carte aleatoire
python demo.py --carte labyrinthe
python demo.py --carte murs --rayon 0.105
pause
