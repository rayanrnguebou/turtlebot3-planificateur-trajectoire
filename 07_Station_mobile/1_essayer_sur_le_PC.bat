@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo === Station de controle TurtleBot3 : mode SIMULATION (robot virtuel) ===
set PY=python
if exist "..\03_Phase1_Algorithmes\.venv\Scripts\python.exe" set PY="..\03_Phase1_Algorithmes\.venv\Scripts\python.exe"
%PY% -c "import numpy" 2>nul || %PY% -m pip install numpy
echo Si Windows demande l'autorisation du pare-feu pour Python : clique sur Autoriser.
start "" /min cmd /c "timeout /t 3 >nul & start http://localhost:8080"
%PY% station_mobile.py --simulation
pause
