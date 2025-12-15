@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo Working directory: %CD%
echo.
echo Copying files from source directory...
xcopy /E /I /Y "C:\Users\v-353\OneDrive\Рабочий стол\N\проект по питону 2025\final_project\gazebo" "gazebo" >nul
xcopy /E /I /Y "C:\Users\v-353\OneDrive\Рабочий стол\N\проект по питону 2025\final_project\python" "python" >nul
copy /Y "C:\Users\v-353\OneDrive\Рабочий стол\N\проект по питону 2025\final_project\.gitignore" ".gitignore" >nul 2>&1
echo Files copied.
echo.
echo Adding files to git...
git add gazebo python .gitignore
echo.
echo Committing...
git commit -m "Add project structure: Gazebo simulation package and Python tracker package"
echo.
echo Pushing to GitHub...
git push -u origin main
echo.
echo Done!
pause

