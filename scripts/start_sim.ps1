# Start Bob's simulation on Windows: Webots here + ROS 2 (controller and bridge) in Docker.
# The PowerShell twin of scripts/start_sim.sh (macOS/Linux).
#
#   scripts\start_sim.ps1 -Nav planner        walk to the orange target (drag it in Webots), round walls
#   scripts\start_sim.ps1 -Nav scripted       the simpler scripted navigator
#   scripts\start_sim.ps1 -Policy rl\policies\rocky_walk.npz     walk with a policy, drive it with /cmd_vel
#   scripts\start_sim.ps1 -World open         the empty arena (speed tests); -World cad: the old CAD Bob
#   scripts\start_sim.ps1 -Rebuild            rebuild the Docker image first
#   scripts\start_sim.ps1 -Stop               stop the container (and Webots)
# The lidar viewer runs too: http://localhost:8765
param([string]$World = "rocky", [string]$Nav = "", [string]$Policy = "", [switch]$Rebuild, [switch]$Stop)
$ErrorActionPreference = "Stop"
$Repo = Split-Path -Parent $PSScriptRoot
$Compose = "$Repo\docker\compose.yaml"
$Webots = if ($env:WEBOTS_HOME) { $env:WEBOTS_HOME } else { "C:\Program Files\Webots" }
$WebotsExe = "$Webots\msys64\mingw64\bin\webotsw.exe"

if ($Stop) {
    docker compose -f $Compose down
    Get-Process webots*, webotsw -ErrorAction SilentlyContinue | Stop-Process
    Write-Host "Stopped."; exit 0
}
$WorldFile = switch ($World) {
    "rocky" { "$Repo\cad\rocky_bob\sim\worlds\rocky_bob.wbt" }
    "open"  { "$Repo\cad\rocky_bob\sim\worlds\rocky_bob_open.wbt" }
    "cad"   { "$Repo\webots_quadruped\worlds\bob.wbt" }
    default { (Resolve-Path $World).Path }
}
if (-not (Test-Path $WebotsExe)) { throw "Webots not found at $Webots. Run scripts\setup_windows.ps1 or set WEBOTS_HOME." }

function InContainer($path) {   # repo path -> path inside the container
    $full = (Resolve-Path $path).Path
    if (-not $full.StartsWith($Repo)) { throw "Policy must be inside the repo: $path" }
    "/workspace/" + $full.Substring($Repo.Length + 1).Replace("\", "/")
}

# 1. Docker
if (-not (docker info 2>$null)) {
    Write-Host "Starting Docker Desktop..."
    Start-Process "C:\Program Files\Docker\Docker\Docker Desktop.exe"
    for ($i = 0; $i -lt 90 -and -not (docker info 2>$null); $i++) { Start-Sleep 2 }
    if (-not (docker info 2>$null)) { throw "Docker did not start within 3 minutes." }
}
# 2. Webots
if (Get-Process webotsw, webots -ErrorAction SilentlyContinue) {
    Write-Host "Webots is already running; make sure it has $(Split-Path $WorldFile -Leaf) open and is not paused."
} else {
    Write-Host "Opening Webots with $(Split-Path $WorldFile -Leaf)"
    Start-Process $WebotsExe -ArgumentList "--port=1234", "--mode=realtime", "`"$WorldFile`""
}
# 3. ROS 2 container
if ($Nav -and -not $Policy) { $Policy = "$Repo\rl\policies\rocky_walk.npz" }
$env:BOB_POLICY = if ($Policy) { InContainer $Policy } else { "" }
$env:BOB_NAV = if ($Nav -in @("planner", "scripted")) { $Nav } elseif ($Nav) { InContainer $Nav } else { "" }
$since = (Get-Date).ToUniversalTime().ToString("yyyy-MM-ddTHH:mm:ssZ")
if ($Rebuild) { docker compose -f $Compose up -d --build } else { docker compose -f $Compose up -d }

Write-Host "Waiting for the bridge to connect to Webots..."
$ok = $false
for ($i = 0; $i -lt 150; $i++) {
    if ((docker logs --since $since bob-sim 2>&1) -match "Bob Webots bridge ready") { $ok = $true; break }
    Start-Sleep 2
}
if ($ok) { Write-Host "Bridge connected: 12 motors live." }
else { Write-Warning "Bridge has not connected yet. Check 'docker logs bob-sim', that Webots is running, and that Windows Firewall allows Webots." }

docker exec -d bob-sim bash -c "source /opt/ros/humble/setup.bash && exec python3 /workspace/rl/scan_viewer.py > /tmp/scan_viewer.log 2>&1"
Write-Host "Lidar and floor sensor view: http://localhost:8765"
if ($env:BOB_NAV) {
    Write-Host "Navigation: $env:BOB_NAV (walker $env:BOB_POLICY). Drag the orange target post in Webots."
    Write-Host "Demos: scripts\demo.ps1, or click the 3D view and press 1-4 (0 all, S stop)."
}
