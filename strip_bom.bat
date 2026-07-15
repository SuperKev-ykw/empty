@echo off
echo Stripping BOM from all .c and .h files...
powershell -ExecutionPolicy Bypass -File "%~dp0strip_bom.ps1"
echo Done. Press any key to exit.
pause >nul
