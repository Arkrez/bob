# Webots demos of Bob (rl/demo.py), twin of demo.sh. Starts the sim in planner mode if needed.
#   scripts\demo.ps1            all demos: tour, stairs, flip, walls
#   scripts\demo.ps1 stairs     just one
$running = (docker ps --format "{{.Names}}" 2>$null) -contains "bob-sim"
$planner = $running -and ((docker logs bob-sim 2>&1 | Select-Object -Last 50) -match "navigation mode: planner")
if (-not $planner) {
    & "$PSScriptRoot\start_sim.ps1" -Stop | Out-Null
    & "$PSScriptRoot\start_sim.ps1" -Nav planner
    Start-Sleep 8
}
& "$PSScriptRoot\sim_ros.ps1" python3 /workspace/rl/demo.py @args
