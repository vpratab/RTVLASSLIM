$ErrorActionPreference = "Stop"

$workspace = "C:\Users\saanvi\Documents\Codex\2026-05-10-build-what-this-thing-is-suggesting"
$outputDir = Join-Path $workspace "artifacts\texbat"

New-Item -ItemType Directory -Force -Path $outputDir | Out-Null

$files = @(
    "https://rnl-data.ae.utexas.edu/datastore/texbat/processed/cleanStatic/navsol.mat",
    "https://rnl-data.ae.utexas.edu/datastore/texbat/processed/ds2/navsol.mat",
    "https://rnl-data.ae.utexas.edu/datastore/texbat/processed/ds3/navsol.mat",
    "https://rnl-data.ae.utexas.edu/datastore/texbat/processed/ds7/navsol.mat"
)

foreach ($uri in $files) {
    $fileName = ($uri -split "/")[-2] + "_navsol.mat"
    $destination = Join-Path $outputDir $fileName
    Write-Output "Downloading $uri -> $destination"
    Invoke-WebRequest -Uri $uri -OutFile $destination
}

Get-ChildItem $outputDir
