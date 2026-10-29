@echo off
setlocal enabledelayedexpansion

echo ========================================================
echo  GPS-Plugin-Plus - Building with MSVC (Visual Studio)
echo ========================================================

cd /d "%~dp0"

:: Auto-detect CMake in PATH, WinGet, or Visual Studio
where cmake >nul 2>nul
if errorlevel 1 (
    for /d %%D in ("%LOCALAPPDATA%\Microsoft\WinGet\Packages\Kitware.CMake_*") do (
        for /d %%C in ("%%D\cmake-*\bin") do (
            if exist "%%C\cmake.exe" set "PATH=%%C;!PATH!"
        )
    )
)
where cmake >nul 2>nul
if errorlevel 1 (
    if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" (
        set "PATH=%ProgramFiles(x86)%\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin;!PATH!"
    ) else if exist "%ProgramFiles%\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" (
        set "PATH=%ProgramFiles%\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin;!PATH!"
    )
)

where cmake >nul 2>nul
if errorlevel 1 (
    echo [ERROR] CMake could not be found. Please ensure CMake or Visual Studio is installed.
    exit /b 1
)

:: Configure
echo [1/3] Configuring CMake (Win32)...
cmake -B "build-msvc" -G "Visual Studio 17 2022" -A Win32
if errorlevel 1 (
    echo [ERROR] CMake configuration failed.
    exit /b %errorlevel%
)

:: Build
echo [2/3] Compiling Release binaries...
cmake --build "build-msvc" --config Release
if errorlevel 1 (
    echo [ERROR] Compilation failed.
    exit /b %errorlevel%
)

:: Package and create Zip archive
echo [3/3] Creating zip package...
set "STAGE_DIR=package-msvc"
if exist "%STAGE_DIR%" rmdir /s /q "%STAGE_DIR%"
if not exist "dist" mkdir "dist"

mkdir "%STAGE_DIR%\plugins"
mkdir "%STAGE_DIR%\components"
mkdir "%STAGE_DIR%\include"
mkdir "%STAGE_DIR%\scriptfiles"

if exist "build-msvc\Release\GPS.dll" (
    copy /Y "build-msvc\Release\GPS.dll" "%STAGE_DIR%\plugins\GPS.dll" >nul
)
if exist "build-msvc\components\Release\GPS.dll" (
    copy /Y "build-msvc\components\Release\GPS.dll" "%STAGE_DIR%\components\GPS.dll" >nul
)
if exist "GPS.inc" (
    copy /Y "GPS.inc" "%STAGE_DIR%\include\GPS.inc" >nul
)
if exist "GPS.dat" (
    copy /Y "GPS.dat" "%STAGE_DIR%\scriptfiles\GPS.dat" >nul
)
if exist "Example.pwn" (
    copy /Y "Example.pwn" "%STAGE_DIR%\Example.pwn" >nul
)
if exist "LICENSE" (
    copy /Y "LICENSE" "%STAGE_DIR%\LICENSE" >nul
)

:: Clean up previous files in dist
if exist "dist\GPS-Plugin-Plus-Windows.zip" del /f /q "dist\GPS-Plugin-Plus-Windows.zip"
if exist "dist\plugins" rmdir /s /q "dist\plugins"
if exist "dist\components" rmdir /s /q "dist\components"
if exist "dist\include" rmdir /s /q "dist\include"
if exist "dist\scriptfiles" rmdir /s /q "dist\scriptfiles"

:: Generate zip archive
where tar.exe >nul 2>nul
if not errorlevel 1 (
    tar.exe -a -c -f "dist\GPS-Plugin-Plus-Windows.zip" -C "%STAGE_DIR%" *
) else (
    powershell -Command "Import-Module Microsoft.PowerShell.Archive; Compress-Archive -Path '%STAGE_DIR%\*' -DestinationPath 'dist\GPS-Plugin-Plus-Windows.zip' -Force"
)

:: Clean up staging folder
if exist "%STAGE_DIR%" rmdir /s /q "%STAGE_DIR%"

echo [SUCCESS] Package created: dist\GPS-Plugin-Plus-Windows.zip

:: Optional deployment if target path is passed as argument or env var
set "TARGET_DIR=%~1"
if "%TARGET_DIR%"=="" set "TARGET_DIR=%OMP_SERVER_DIR%"

if not "%TARGET_DIR%"=="" (
    if exist "%TARGET_DIR%" (
        echo.
        echo Deploying to target directory: %TARGET_DIR%
        if exist "build-msvc\components\Release\GPS.dll" if exist "%TARGET_DIR%\components" copy /Y "build-msvc\components\Release\GPS.dll" "%TARGET_DIR%\components\GPS.dll" >nul
        if exist "build-msvc\Release\GPS.dll" if exist "%TARGET_DIR%\plugins" copy /Y "build-msvc\Release\GPS.dll" "%TARGET_DIR%\plugins\GPS.dll" >nul
        if exist "GPS.inc" if exist "%TARGET_DIR%\pawno\include" copy /Y "GPS.inc" "%TARGET_DIR%\pawno\include\GPS.inc" >nul
        if exist "GPS.dat" if exist "%TARGET_DIR%\scriptfiles" copy /Y "GPS.dat" "%TARGET_DIR%\scriptfiles\GPS.dat" >nul
        echo Deployment complete.
    )
)

echo ========================================================
echo  Build finished successfully!
echo ========================================================
