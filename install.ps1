# Copyright (c) 2020 Norwegian University of Science and Engineering.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

Write-Output "Installing CoppeliaSim with PyRep"

$start_directory = Get-Location
$install_folder = "CoppeliaSim_PyRep"

# Set environment variables
$env:COPPELIASIM_ROOT = "$start_directory\$install_folder"
$env:Path = "$env:COPPELIASIM_ROOT\Scripts;$env:COPPELIASIM_ROOT;" + $env:Path

if (!(Test-Path $install_folder)) {
    New-Item -Name $install_folder -ItemType Directory | Out-Null
}

$coppeliasim_url = "https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_0_0_Win.zip"
$coppeliasim_zip = "coppeliasim.zip"
if (!(Test-Path $coppeliasim_zip -PathType Leaf)) {
    Write-Output "Downloading CoppeliaSim Edu 4.0.0 from $coppeliasim_url"
    Invoke-WebRequest $coppeliasim_url -OutFile $coppeliasim_zip | Out-Null
}
if (!(Test-Path "$install_folder\coppeliaSim.dll" -PathType Leaf)) {
    Write-Output "Expanding CoppeliaSim Edu 4.0.0"
    Expand-Archive $coppeliasim_zip -DestinationPath $install_folder | Out-Null
}

$python_url = "https://www.python.org/ftp/python/3.8.2/python-3.8.2-embed-amd64.zip"
$python_zip = "python.zip"
if (!(Test-Path $python_zip -PathType Leaf)) {
    Write-Output "Downloading Python 3.8.2 from $python_url"
    Invoke-WebRequest -Uri $python_url -OutFile $python_zip | Out-Null
}
if (!(Test-Path "$install_folder\python.exe" -PathType Leaf)) {
    Write-Output "Expanding Python 3.8.2"
    Expand-Archive $python_zip -DestinationPath $install_folder -Force | Out-Null
}

$get_pip_url = "https://bootstrap.pypa.io/get-pip.py"
$get_pip_file = "get-pip.py"
if (!(Test-Path $get_pip_file -PathType Leaf)) {
    Write-Output "Downloading 'get-pip.py'"
    Invoke-WebRequest $get_pip_url -OutFile $get_pip_file | Out-Null
}
if (!(Test-Path "$install_folder\$get_pip_file" -PathType Leaf)) {
    Write-Output "Installing pip"
    Copy-Item $get_pip_file -Destination $install_folder
    Set-Content -Path $install_folder\python38._pth -Value "python38.zip`n.`nimport site"
    New-Item -Name $install_folder\sitecustomize.py -ItemType File -Force | Out-Null
    Set-Content -Path $install_folder\sitecustomize.py -Value "import sys`nsys.path.insert(0, '')"
    & python "get-pip.py"
}

$pyrep_url = "https://github.com/tingelst/PyRep/archive/windows-support.zip"
$pyrep_zip = "pyrep.zip"
if (!(Test-Path $pyrep_zip -PathType Leaf)) {
    Write-Output "Downloading PyRep from $pyrep_url"
    Invoke-WebRequest -Uri $pyrep_url -OutFile $pyrep_zip | Out-Null
}
if (!(Test-Path "$install_folder\PyRep")) {
    Write-Output "Expanding and installing PyRep"
    Expand-Archive $pyrep_zip -DestinationPath "$install_folder" -Force | Out-Null
    Rename-Item "$install_folder\PyRep-windows-support" -NewName "PyRep"
    Set-Location "$install_folder\PyRep"
    & python -m pip install numpy cffi
    & python -m pip install .
    Set-Location $start_directory
}

if (!(Test-Path "$install_folder\setup.ps1")) {
    New-Item -Name $install_folder\setup.ps1 -ItemType File 
    Set-Content -Path $install_folder\setup.ps1 -Value "`$env:COPPELIASIM_ROOT = Get-Location"
    Add-Content -Path $install_folder\setup.ps1 "`$env:Path = `"`$env:COPPELIASIM_ROOT\Scripts;`$env:COPPELIASIM_ROOT;`" + `$env:Path"
}


