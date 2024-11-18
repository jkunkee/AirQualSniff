
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

git submodule init
git submodule update
pushd lib\mqtt5
git checkout master
git pull
popd
pushd lib\Particle_mDNSResolver
git checkout master
git pull
popd
pushd lib\SparkFun_BMA400_Arduino_Library
git checkout main
git pull
popd
pushd lib\SparkFun_External_EEPROM_Arduino_Library
git checkout main
git pull
popd
