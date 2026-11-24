@echo off
setlocal
cd /d "%~dp0"
echo ===========================================
echo BRAZO ROBOTICO V6 - MODELO XY NEGATIVO 56mm
echo ===========================================
echo Usaremos puerto 5517 para NO abrir el servidor antiguo del 5500.
echo La carpeta en pantalla debe ser la interfaz de ESTE proyecto.
echo.
where py >nul 2>nul
if %ERRORLEVEL% EQU 0 goto usar_py
where python >nul 2>nul
if %ERRORLEVEL% EQU 0 goto usar_python
echo ERROR: No se encontro Python. Instala Python 3 o usa Live Server.
pause
exit /b 1
:usar_py
py -3 "%~dp0iniciar_interfaz.py"
goto terminar
:usar_python
python "%~dp0iniciar_interfaz.py"
:terminar
pause
endlocal
