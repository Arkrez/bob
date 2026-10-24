# One-time setup of Bob's simulation on Windows 10/11.
#
#   git clone -b feat/rocky-bob https://github.com/Arkrez/bob.git
#   cd bob
#   powershell -ExecutionPolicy Bypass -File scripts\setup_windows.ps1
#
# Installs (with winget, if missing): Git, Python 3.12, Docker Desktop, Webots R2025a.
# Then: the Python environment for MuJoCo training (rl\.venv) and the ROS 2 sim container image.
# Docker Desktop needs WSL 2; if it asks you to restart or sign in, do that and run this again.
$ErrorActionPreference = "Stop"
$Repo = Split-Path -Parent $PSScriptRoot

function Have($cmd) { [bool](Get-Command $cmd -ErrorAction SilentlyContinue) }
function Install($id, $what) {
    Write-Host "Installing $what ($id)..."
    winget install --id $id -e --accept-source-agreements --accept-package-agreements
}

if (-not (Have winget)) { throw "winget not found: install 'App Installer' from the Microsoft Store first." }
if (-not (Have git)) { Install "Git.Git" "Git" }
if (-not (Have py)) { Install "Python.Python.3.12" "Python 3.12" }
if (-not (Have docker)) { Install "Docker.DockerDesktop" "Docker Desktop" }
$Webots = if ($env:WEBOTS_HOME) { $env:WEBOTS_HOME } else { "C:\Program Files\Webots" }
if (-not (Test-Path "$Webots\msys64\mingw64\bin\webotsw.exe")) {
    Install "Cyberbotics.Webots" "Webots"
    if (-not (Test-Path "$Webots\msys64\mingw64\bin\webotsw.exe")) {
        Write-Warning "Webots not found at $Webots. Install R2025a from https://cyberbotics.com/#download (or set WEBOTS_HOME), then run this again."
    }
}
# fresh installs aren't on PATH in this window yet
$env:Path = [Environment]::GetEnvironmentVariable("Path", "Machine") + ";" + [Environment]::GetEnvironmentVariable("Path", "User")

Write-Host "Python environment for training (rl\.venv)..."
Push-Location "$Repo\rl"
if (-not (Test-Path ".venv")) { py -3.12 -m venv .venv }
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -c "import mujoco, stable_baselines3, torch; print('MuJoCo', mujoco.__version__, '| SB3', stable_baselines3.__version__, '| torch', torch.__version__)"
Pop-Location

Write-Host "Sim container (first build takes a while)..."
if (-not (docker info 2>$null)) {
    Start-Process "C:\Program Files\Docker\Docker\Docker Desktop.exe"
    Write-Host "Waiting for Docker Desktop..."
    for ($i = 0; $i -lt 90 -and -not (docker info 2>$null); $i++) { Start-Sleep 2 }
}
docker compose -f "$Repo\docker\compose.yaml" build

Write-Host ""
Write-Host "Done. Start the sim with:   powershell -ExecutionPolicy Bypass -File scripts\start_sim.ps1 -Nav planner"
Write-Host "If Windows Firewall asks about Webots, allow it on private networks (the sim container connects to it)."
