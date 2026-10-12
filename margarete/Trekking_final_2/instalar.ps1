$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
if (-not (Test-Path -LiteralPath '.venv/Scripts/python.exe')) {
    py -3.11 -m venv .venv
    if ($LASTEXITCODE -ne 0) { throw 'Instale Python 3.11 com o launcher py.' }
}
& '.venv/Scripts/python.exe' -m pip install -r requirements.txt
if ($LASTEXITCODE -ne 0) { throw 'Falha ao instalar dependencias; confira a mensagem do pip.' }
Write-Host 'Instalacao concluida. Execute: .\.venv\Scripts\python.exe main.py'
