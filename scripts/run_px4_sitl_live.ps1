param(
    [string]$Image = "px4io/px4-sitl:latest",
    [string]$Connection = "udpout:127.0.0.1:14550",
    [int]$SniffEvents = 8,
    [int]$SniffTimeoutSeconds = 45,
    [int]$VerdictLimit = 5,
    [int]$RunTimeoutSeconds = 60,
    [string]$EvidencePath = "artifacts/px4_sitl_evidence.bin"
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot
$containerName = "rtvlas-px4-sitl"

function Wait-DockerReady {
    if (Get-Command docker -ErrorAction SilentlyContinue) {
        try {
            docker ps | Out-Null
            return
        } catch {
        }
    }

    $desktopPath = "C:\Program Files\Docker\Docker\Docker Desktop.exe"
    if (Test-Path $desktopPath) {
        Start-Process -FilePath $desktopPath | Out-Null
    }

    $deadline = (Get-Date).AddMinutes(2)
    while ((Get-Date) -lt $deadline) {
        try {
            docker ps | Out-Null
            return
        } catch {
            Start-Sleep -Seconds 5
        }
    }

    throw "Docker Desktop did not become ready within 2 minutes."
}

function Stop-ContainerIfPresent {
    try {
        docker rm -f $containerName 2>$null | Out-Null
    } catch {
    }
}

function Invoke-CargoWithTimeout {
    param(
        [string]$ArgumentList,
        [int]$TimeoutSeconds
    )

    $process = Start-Process -FilePath "cargo" `
        -ArgumentList $ArgumentList `
        -WorkingDirectory $repoRoot `
        -NoNewWindow `
        -PassThru

    try {
        Wait-Process -Id $process.Id -Timeout $TimeoutSeconds -ErrorAction Stop
    } catch {
        Stop-Process -Id $process.Id -Force -ErrorAction SilentlyContinue
        throw "Command timed out after ${TimeoutSeconds}s: cargo $ArgumentList"
    }

    if ($process.ExitCode -ne 0) {
        throw "Command failed with exit code $($process.ExitCode): cargo $ArgumentList"
    }
}

Wait-DockerReady
New-Item -ItemType Directory -Force -Path (Join-Path $repoRoot "artifacts") | Out-Null

Write-Host "Pulling $Image"
docker pull $Image | Out-Host

Stop-ContainerIfPresent

Write-Host "Starting PX4 SITL container"
docker run --rm -d `
    --name $containerName `
    -e PX4_SIM_MODEL=sihsim_quadx `
    -p 14550:14550/udp `
    $Image | Out-Host

try {
    Write-Host "Sniffing live telemetry"
    Invoke-CargoWithTimeout `
        -ArgumentList "run --example mavlink_sniff -- --connection $Connection --event-limit $SniffEvents" `
        -TimeoutSeconds $SniffTimeoutSeconds

    Write-Host "Running orchestrator against live PX4 telemetry"
    Invoke-CargoWithTimeout `
        -ArgumentList "run --example px4_sitl_live -- --connection $Connection --verdict-limit $VerdictLimit --evidence $EvidencePath" `
        -TimeoutSeconds $RunTimeoutSeconds
} finally {
    Write-Host "Stopping PX4 SITL container"
    Stop-ContainerIfPresent
}
