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

:: Stage artifacts locally into dist/
echo [3/3] Staging output files to dist...
if not exist "dist" mkdir "dist"
if not exist "dist\plugins" mkdir "dist\plugins"
if not exist "dist\components" mkdir "dist\components"
if not exist "dist\include" mkdir "dist\include"
if not exist "dist\scriptfiles" mkdir "dist\scriptfiles"

if exist "build-msvc\Release\GPS.dll" (
    copy /Y "build-msvc\Release\GPS.dll" "dist\plugins\GPS.dll" >nul
    echo  - Staged dist\plugins\GPS.dll
)

if exist "build-msvc\components\Release\GPS.dll" (
    copy /Y "build-msvc\components\Release\GPS.dll" "dist\components\GPS.dll" >nul
    echo  - Staged dist\components\GPS.dll
)

if exist "GPS.inc" (
    copy /Y "GPS.inc" "dist\include\GPS.inc" >nul
    echo  - Staged dist\include\GPS.inc
)

if exist "GPS.dat" (
    copy /Y "GPS.dat" "dist\scriptfiles\GPS.dat" >nul
    echo  - Staged dist\scriptfiles\GPS.dat
)

:: Optional deployment if target path is passed as argument or env var
set "TARGET_DIR=%~1"
if "%TARGET_DIR%"=="" set "TARGET_DIR=%OMP_SERVER_DIR%"

if not "%TARGET_DIR%"=="" (
    if exist "%TARGET_DIR%" (
        echo.
        echo Deploying to target directory: %TARGET_DIR%
        if exist "dist\components\GPS.dll" if exist "%TARGET_DIR%\components" copy /Y "dist\components\GPS.dll" "%TARGET_DIR%\components\GPS.dll" >nul
        if exist "dist\plugins\GPS.dll" if exist "%TARGET_DIR%\plugins" copy /Y "dist\plugins\GPS.dll" "%TARGET_DIR%\plugins\GPS.dll" >nul
        if exist "dist\include\GPS.inc" if exist "%TARGET_DIR%\pawno\include" copy /Y "dist\include\GPS.inc" "%TARGET_DIR%\pawno\include\GPS.inc" >nul
        if exist "dist\scriptfiles\GPS.dat" if exist "%TARGET_DIR%\scriptfiles" copy /Y "dist\scriptfiles\GPS.dat" "%TARGET_DIR%\scriptfiles\GPS.dat" >nul
        echo Deployment complete.
    )
)

echo ========================================================
echo  Build finished successfully!
echo ========================================================
