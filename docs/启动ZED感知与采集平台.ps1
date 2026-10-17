param([switch]$NoBrowser)
$ErrorActionPreference = 'Stop'
$taskPython = 'D:/005_CodeTools/Anaconda/python.exe'
$taskWorkspace = 'C:/Users/yfx33/Documents/Codex/2026-10-06/ssh-172-20-10-3-uavlab'
$taskUrl = 'http://127.0.0.1:8767/'
$taskAvailable = $false
try {
    $taskState = Invoke-RestMethod -Uri ($taskUrl + 'api/status') -TimeoutSec 2
    $taskAvailable = ($taskState.host -eq '192.168.50.100' -and $taskState.api_version -eq 2)
} catch {}
if (-not $taskAvailable) {
    if (-not (Test-Path -LiteralPath $taskPython)) { throw '找不到已配置的 Python。' }
    $taskTarget = Join-Path $taskWorkspace 'work/jetson_dashboard/dashboard.py'
    if (-not (Test-Path -LiteralPath $taskTarget)) { throw '找不到已部署的采集平台。' }
    $taskProcess = Start-Process -FilePath $taskPython -ArgumentList @('-u','work/jetson_dashboard/dashboard.py') -WorkingDirectory $taskWorkspace -WindowStyle Hidden -RedirectStandardOutput (Join-Path $taskWorkspace 'work/jetson_dashboard/stdout.log') -RedirectStandardError (Join-Path $taskWorkspace 'work/jetson_dashboard/stderr.log') -PassThru
    Set-Content -LiteralPath (Join-Path $taskWorkspace 'work/jetson_dashboard/server.pid') -Value $taskProcess.Id
    for ($taskAttempt = 0; $taskAttempt -lt 25; $taskAttempt++) {
        try {
            $taskState = Invoke-RestMethod -Uri ($taskUrl + 'api/status') -TimeoutSec 1
            $taskAvailable = ($taskState.host -eq '192.168.50.100' -and $taskState.api_version -eq 2)
            if ($taskAvailable) { break }
        } catch {}
        Start-Sleep -Milliseconds 200
    }
    if (-not $taskAvailable) { throw '平台未启动，请检查 8767 端口与 work/jetson_dashboard/stderr.log。' }
}
if (-not $NoBrowser) { Start-Process $taskUrl }
