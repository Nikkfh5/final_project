@echo off
chcp 65001 >nul
echo ========================================
echo Deploying project to GitHub
echo ========================================
echo.

cd /d "%~dp0"
echo Current directory: %CD%
echo.

echo Step 1: Copying files...
if not exist gazebo (
    echo Copying gazebo folder...
    xcopy /E /I /Y "C:\Users\v-353\OneDrive\Рабочий стол\N\проект по питону 2025\final_project\gazebo" "gazebo" >nul
    if errorlevel 1 (
        echo ERROR: Failed to copy gazebo folder
        pause
        exit /b 1
    )
    echo gazebo folder copied successfully
) else (
    echo gazebo folder already exists
)

if not exist python (
    echo Copying python folder...
    xcopy /E /I /Y "C:\Users\v-353\OneDrive\Рабочий стол\N\проект по питону 2025\final_project\python" "python" >nul
    if errorlevel 1 (
        echo ERROR: Failed to copy python folder
        pause
        exit /b 1
    )
    echo python folder copied successfully
) else (
    echo python folder already exists
)

if not exist .gitignore (
    echo Copying .gitignore...
    copy /Y "C:\Users\v-353\OneDrive\Рабочий стол\N\проект по питону 2025\final_project\.gitignore" ".gitignore" >nul 2>&1
)

echo.
echo Step 2: Adding files to git...
git add gazebo python .gitignore
if errorlevel 1 (
    echo ERROR: Failed to add files to git
    pause
    exit /b 1
)
echo Files added to git

echo.
echo Step 3: Committing changes...
git commit -m "Add project structure: Gazebo simulation package and Python tracker package"
if errorlevel 1 (
    echo ERROR: Failed to commit
    pause
    exit /b 1
)
echo Changes committed

echo.
echo Step 4: Pushing to GitHub...
git push -u origin main
if errorlevel 1 (
    echo ERROR: Failed to push to GitHub
    echo Please check your GitHub credentials
    pause
    exit /b 1
)
echo.
echo ========================================
echo SUCCESS! Files pushed to GitHub
echo ========================================
pause

