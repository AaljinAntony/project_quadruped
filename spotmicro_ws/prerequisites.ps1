# Prerequisites Check for SpotMicro (Windows PowerShell)
# Run this script as Administrator

Write-Host "============================================================" -ForegroundColor Cyan
Write-Host "Checking ROS 2 Docker Simulation Prerequisites (Windows)..." -ForegroundColor Cyan
Write-Host "============================================================" -ForegroundColor Cyan

# 1. Check for Administrator Privileges
$currentPrincipal = New-Object Security.Principal.WindowsPrincipal([Security.Principal.WindowsIdentity]::GetCurrent())
if (-not $currentPrincipal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    Write-Host "[ERROR] Please run this script as Administrator!" -ForegroundColor Red
    exit
}

# 2. Check for WSL 2
Write-Host "--> Checking for WSL 2..."
$wslStatus = wsl --status 2>$null
if ($null -eq $wslStatus) {
    Write-Host "[WARNING] WSL is not installed. WSL 2 is required for Docker Desktop." -ForegroundColor Yellow
    Write-Host "Action: Run 'wsl --install' in a new Admin terminal and restart." -ForegroundColor Gray
} else {
    Write-Host "--> WSL is installed."
}

# 3. Check for Docker Desktop
Write-Host "--> Checking for Docker Desktop..."
if (Get-Command "docker" -ErrorAction SilentlyContinue) {
    Write-Host "--> Docker is already installed. (Version: $((docker --version)))"
} else {
    Write-Host "[WARNING] Docker is not found." -ForegroundColor Yellow
    Write-Host "Action: Install Docker Desktop from https://www.docker.com/products/docker-desktop/" -ForegroundColor Gray
    Write-Host "Alternatively, run: winget install Docker.DockerDesktop" -ForegroundColor Gray
}

# 4. Check for NVIDIA Driver (for GPU support)
Write-Host "--> Checking for NVIDIA Drivers..."
if (Get-Command "nvidia-smi" -ErrorAction SilentlyContinue) {
    Write-Host "--> NVIDIA Driver found."
} else {
    Write-Host "[INFO] NVIDIA Driver not found. This is optional unless using GPU acceleration." -ForegroundColor Cyan
}

# 5. Summary
Write-Host "============================================================" -ForegroundColor Green
Write-Host "Checks complete! If all core components (WSL 2, Docker) are ready," -ForegroundColor Green
Write-Host "you can now run: .\start.bat" -ForegroundColor Green
Write-Host "============================================================" -ForegroundColor Green
