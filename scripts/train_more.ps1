# Train Bob some more on Windows, then stop (rl/train_rounds.py; twin of train_more.sh).
#   scripts\train_more.ps1 -Hours 3            rounds in the background for 3 hours (default 2)
#   scripts\train_more.ps1 -Rounds 2 -Tasks walker,recover
#   scripts\train_more.ps1 -Status             champions so far and the recent log
#   scripts\train_more.ps1 -Stop               finish the current round, then stop
# The PC is kept awake while it trains. New champions go into rl\policies\; a running sim picks them up.
param([double]$Hours = 2, [int]$Rounds = 0, [string]$Tasks = "walker,nav,recover", [switch]$Status, [switch]$Stop)
$RL = Join-Path (Split-Path -Parent $PSScriptRoot) "rl"
New-Item -ItemType Directory -Force "$RL\runs\rounds" | Out-Null
$running = Get-CimInstance Win32_Process -Filter "Name='python.exe'" | Where-Object { $_.CommandLine -match "train_rounds.py" }
if ($Status) {
    Write-Host ("training: " + $(if ($running) { "running" } else { "not running" }))
    if (Test-Path "$RL\runs\rounds\state.json") { Get-Content "$RL\runs\rounds\state.json" }
    if (Test-Path "$RL\runs\rounds\log.md") { Get-Content "$RL\runs\rounds\log.md" -Tail 15 }
    exit 0
}
if ($Stop) { New-Item -ItemType File -Force "$RL\runs\rounds\STOP" | Out-Null; Write-Host "Will stop after the current round."; exit 0 }
if ($running) { Write-Host "Already training (-Status to see it)"; exit 0 }
$a = @("train_rounds.py", "--tasks", $Tasks) + $(if ($Rounds -gt 0) { @("--rounds", $Rounds) } else { @("--hours", $Hours) })
Start-Process -FilePath "$RL\.venv\Scripts\python.exe" -ArgumentList $a -WorkingDirectory $RL -WindowStyle Hidden `
    -RedirectStandardOutput "$RL\runs\rounds\console.log" -RedirectStandardError "$RL\runs\rounds\console.err.log"
Write-Host "Training started. Log: rl\runs\rounds\log.md   Status: scripts\train_more.ps1 -Status"
