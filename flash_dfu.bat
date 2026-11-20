@echo off
setlocal
set PRESET=%1
if "%PRESET%"=="" set PRESET=Debug
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0flash_dfu.ps1" -Preset "%PRESET%"
