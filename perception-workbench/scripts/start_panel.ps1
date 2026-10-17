param([string]$PythonPath = 'python', [string]$DataDirectory, [switch]$NoBrowser)
$ErrorActionPreference = 'Stop'
$taskRepo = Split-Path -Parent $PSScriptRoot
$taskUrl = 'http://127.0.0.1:8767/'
try {
    $taskState = Invoke-RestMethod -Uri ($taskUrl + 'api/status') -TimeoutSec 2
    if ($taskState.api_version -eq 2) {
        if (-not $NoBrowser) { Start-Process $taskUrl }
        return
    }
    throw '8767 端口由其他服务占用。'
} catch {
    if ($_.Exception.Message -eq '8767 端口由其他服务占用。') { throw }
}
$taskPython = (Get-Command $PythonPath -ErrorAction Stop).Source
if (-not $DataDirectory) { $DataDirectory = Join-Path $taskRepo 'data/perception_data' }
$env:UAVLAB_OUTPUT_DIRECTORY = [IO.Path]::GetFullPath($DataDirectory)
$taskLogs = Join-Path $taskRepo 'data/panel_logs'
New-Item -ItemType Directory -Path $taskLogs -Force | Out-Null
$taskProcess = Start-Process -FilePath $taskPython -ArgumentList @('-u', 'panel/dashboard.py') -WorkingDirectory $taskRepo -WindowStyle Hidden -RedirectStandardOutput (Join-Path $taskLogs 'stdout.log') -RedirectStandardError (Join-Path $taskLogs 'stderr.log') -PassThru
Set-Content -LiteralPath (Join-Path $taskLogs 'server.pid') -Value $taskProcess.Id
for ($taskAttempt = 0; $taskAttempt -lt 40; $taskAttempt++) {
    try {
        $taskState = Invoke-RestMethod -Uri ($taskUrl + 'api/status') -TimeoutSec 1
        if ($taskState.api_version -eq 2) {
            if (-not $NoBrowser) { Start-Process $taskUrl }
            return
        }
    } catch {}
    Start-Sleep -Milliseconds 200
}
throw "服务未启动，请查看 $taskLogs/stderr.log。"
