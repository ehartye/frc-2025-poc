param([Parameter(Mandatory=$true)][string]$JavaHome,[Parameter(Mandatory=$true)][string]$GradleHome,[string]$OutputDirectory)
$ErrorActionPreference='Stop'
if (!$OutputDirectory) {
    $OutputDirectory=Join-Path $PSScriptRoot ('artifacts/' + [DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffffffZ'))
}
& (Join-Path $PSScriptRoot 'experiments/six-wheel/run.ps1') -JavaHome $JavaHome -GradleHome $GradleHome -OutputDirectory $OutputDirectory

