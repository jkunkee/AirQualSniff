
Set-StrictMode -Version 2
$ErrorActionPreference = 'Stop'

cd $PSScriptRoot

$propertiesFile = gci $PSScriptRoot\project.properties
$propertiesRaw = gc $propertiesFile
$props = @{}

$propertiesRaw | % {
    if ($_ -imatch 'dependencies\.(.*)=(.*)') {
        $name = $Matches[1]
        $ver = $Matches[2]
        Write-Host "Processing $name, was v$ver"
        Remove-Item -Force $PSScriptRoot\lib\$name -ErrorAction SilentlyContinue -Recurse
        particle library copy $Matches[1]
    }
}