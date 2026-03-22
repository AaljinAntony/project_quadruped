@echo off
echo ============================================================
echo Starting SpotMicro Docker Environment (Windows)
echo ============================================================

:: Check for Docker
docker --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Docker is not installed or not in PATH.
    echo Please install Docker Desktop and ensure it is running.
    pause
    exit /b 1
)

:: Start the docker containers in the background
echo --^> Starting containers via Docker Compose...
docker compose up -d

echo.
echo ✅ Docker environment started.
echo You can now enter the container with:
echo    docker exec -it spotmicro_container bash
echo.
echo NOTE: If you are on Windows 11 or Windows 10 (21H2+), 
echo GUI applications (Gazebo/RViz) will work automatically via WSLg.
echo.
pause
